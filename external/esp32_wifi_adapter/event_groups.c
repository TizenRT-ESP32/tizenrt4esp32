/******************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************/

/****************************************************************************
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Samsung sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2016 Samsung Electronics, Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <tinyara/config.h>

#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <queue.h>

#include <tinyara/sched.h>
#include <sched/sched.h>

#include "event_groups.h"

#if defined(CONFIG_USE_16_BIT_TICKS)
#define eventCLEAR_EVENTS_ON_EXIT_BIT	0x0100U
#define eventUNBLOCKED_DUE_TO_BIT_SET	0x0200U
#define eventWAIT_FOR_ALL_BITS			0x0400U
#define eventEVENT_BITS_CONTROL_BYTES	0xff00U
#else
#define eventCLEAR_EVENTS_ON_EXIT_BIT	0x01000000UL
#define eventUNBLOCKED_DUE_TO_BIT_SET	0x02000000UL
#define eventWAIT_FOR_ALL_BITS			0x04000000UL
#define eventEVENT_BITS_CONTROL_BYTES	0xff000000UL
#endif

typedef struct xEventGroupDefinition {
	EventBits_t uxEventBits;
	sq_queue_t xTasksWaitingForBits;	/*< List of tasks waiting for a bit to be set. */
	pthread_mutex_t eventGroupMux;	//Mutex required due to SMP
} EventGroup_t;

typedef struct event_node {
	sq_entry_t node;
	pid_t pid;
	TickType_t xItemValue;
} event_node_t;

EventGroupHandle_t xEventGroupCreate(void)
{
	EventGroup_t *pxEventBits;

	/* Allocate the event group. */
	pxEventBits = (EventGroup_t *)malloc(sizeof(EventGroup_t));
	if (pxEventBits != NULL) {
		pxEventBits->uxEventBits = 0;
		sq_init(&(pxEventBits->xTasksWaitingForBits));
		pthread_mutex_init(&pxEventBits->eventGroupMux, NULL);
	} else {
		return NULL;
	}

	return (EventGroupHandle_t) pxEventBits;
}

static BaseType_t prvTestWaitCondition(const EventBits_t uxCurrentEventBits, const EventBits_t uxBitsToWaitFor, const BaseType_t xWaitForAllBits)
{
	BaseType_t xWaitConditionMet = pdFALSE;

	if (xWaitForAllBits == pdFALSE) {
		/* Task only has to wait for one bit within uxBitsToWaitFor to be
		   set.  Is one already set? */
		if ((uxCurrentEventBits & uxBitsToWaitFor) != (EventBits_t) 0) {
			xWaitConditionMet = pdTRUE;
		}
	} else {
		/* Task has to wait for all the bits in uxBitsToWaitFor to be set.
		   Are they set already? */
		if ((uxCurrentEventBits & uxBitsToWaitFor) == uxBitsToWaitFor) {
			xWaitConditionMet = pdTRUE;
		}
	}

	return xWaitConditionMet;
}

static void vTaskPlaceOnUnorderedEventList(sq_queue_t *pxEventList, const TickType_t xItemValue, const TickType_t xTicksToWait)
{
	if (!pxEventList) {
		return;
	}

	/* Store the item value in the event list item.  It is safe to access the
	   event list item here as interrupts won't access the event list item of a
	   task that is not in the Blocked state. */
	event_node_t *pevent_node = (event_node_t *)malloc(sizeof(event_node_t));
	if (!pevent_node) {
		return;
	}

	pevent_node->xItemValue = xItemValue;
	pevent_node->pid = getpid();
	sq_addlast(&pevent_node->node, pxEventList);

	struct tcb_s *ptcp = sched_gettcb(pevent_node->pid);
	if (!ptcp) {
		return;
	}
	//block task
	up_block_task(ptcp, TSTATE_TASK_INACTIVE);
	return;
}

static BaseType_t xTaskRemoveFromUnorderedEventList(event_node_t *pxEventListItem, sq_queue_t *event_list, const TickType_t xItemValue)
{
	BaseType_t xReturn = pdFALSE;

	if (!pxEventListItem || !event_list) {
		return -1;
	}

	/* Store the new item value in the event list. */
	pxEventListItem->xItemValue = xItemValue;

	struct tcb_s *ptcp = sched_gettcb(pxEventListItem->pid);
	struct tcb_s *current_tcb = this_task();

	/* Remove the event list form the event flag.  Interrupts do not access
	   event flags. */

	sq_rem((sq_entry_t *) pxEventListItem, event_list);

	/* Remove the task from the delayed list and add it to the ready list.  The
	   scheduler is suspended so interrupts will not be accessing the ready
	   lists. */

	if (ptcp) {
		sched_removeblocked(ptcp);
		sched_addreadytorun(ptcp);

		if (ptcp->sched_priority >= current_tcb->sched_priority) {
			/* Return true if the task removed from the event list has
			   a higher priority than the calling task.  This allows
			   the calling task to know if it should force a context
			   switch now. */
			xReturn = pdTRUE;
		} else {
			xReturn = pdFALSE;
		}
	}

	return xReturn;
}

static TickType_t uxTaskResetEventItemValue(EventGroup_t *pxEventBits)
{
	TickType_t uxReturn = -1;

	if (!pxEventBits) {
		return uxReturn;
	}

	pid_t pid = getpid();
	event_node_t *pevent_node = (event_node_t *) pxEventBits->xTasksWaitingForBits.head;
	while (pevent_node) {
		if (pevent_node->pid == pid) {
			uxReturn = pevent_node->xItemValue;
			// reset the value, but i have no idea about what value to be set.
			pevent_node->xItemValue = 0;
			break;
		}
		pevent_node = (event_node_t *) pevent_node->node.flink;
	}

	return uxReturn;
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait)
{
	EventGroup_t *pxEventBits = (EventGroup_t *) xEventGroup;
	EventBits_t uxReturn, uxControlBits = 0;
	BaseType_t xWaitConditionMet;

	/* Check the user is not attempting to wait on the bits used by the kernel
	   itself, and that at least one bit is being requested. */

	sched_lock();
	pthread_mutex_lock(&pxEventBits->eventGroupMux);
	{
		const EventBits_t uxCurrentEventBits = pxEventBits->uxEventBits;

		/* Check to see if the wait condition is already met or not. */
		xWaitConditionMet = prvTestWaitCondition(uxCurrentEventBits, uxBitsToWaitFor, xWaitForAllBits);
		if (xWaitConditionMet != pdFALSE) {
			/* The wait condition has already been met so there is no need to
			   block. */
			uxReturn = uxCurrentEventBits;
			xTicksToWait = (TickType_t) 0;

			/* Clear the wait bits if requested to do so. */
			if (xClearOnExit != pdFALSE) {
				pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
			}
		} else if (xTicksToWait == (TickType_t) 0) {
			/* The wait condition has not been met, but no block time was
			   specified, so just return the current value. */
			uxReturn = uxCurrentEventBits;
		} else {
			/* The task is going to block to wait for its required bits to be
			   set.  uxControlBits are used to remember the specified behaviour of
			   this call to xEventGroupWaitBits() - for use when the event bits
			   unblock the task. */
			if (xClearOnExit != pdFALSE) {
				uxControlBits |= eventCLEAR_EVENTS_ON_EXIT_BIT;
			}

			if (xWaitForAllBits != pdFALSE) {
				uxControlBits |= eventWAIT_FOR_ALL_BITS;
			}

			/* Store the bits that the calling task is waiting for in the
			   task's event list item so the kernel knows when a match is
			   found.  Then enter the blocked state. */
			//vTaskPlaceOnUnorderedEventList( &( pxEventBits->xTasksWaitingForBits ), ( uxBitsToWaitFor | uxControlBits ), xTicksToWait );

			/* This is obsolete as it will get set after the task unblocks, but
			   some compilers mistakenly generate a warning about the variable
			   being returned without being set if it is not done. */
			uxReturn = 0;
		}
	}

	pthread_mutex_unlock(&pxEventBits->eventGroupMux);
	sched_unlock();

	if (xTicksToWait != (TickType_t) 0) {
		//task yield
		vTaskPlaceOnUnorderedEventList(&(pxEventBits->xTasksWaitingForBits), (uxBitsToWaitFor | uxControlBits), xTicksToWait);

		/* The task blocked to wait for its required bits to be set - at this
		   point either the required bits were set or the block time expired.  If
		   the required bits were set they will have been stored in the task's
		   event list item, and they should now be retrieved then cleared. */
		uxReturn = uxTaskResetEventItemValue(pxEventBits);

		if ((uxReturn & eventUNBLOCKED_DUE_TO_BIT_SET) == (EventBits_t) 0) {
			pthread_mutex_lock(&pxEventBits->eventGroupMux);
			/* The task timed out, just return the current event bit value. */
			uxReturn = pxEventBits->uxEventBits;

			/* It is possible that the event bits were updated between this
			   task leaving the Blocked state and running again. */
			if (prvTestWaitCondition(uxReturn, uxBitsToWaitFor, xWaitForAllBits) != pdFALSE) {
				if (xClearOnExit != pdFALSE) {
					pxEventBits->uxEventBits &= ~uxBitsToWaitFor;
				}
			}

			pthread_mutex_unlock(&pxEventBits->eventGroupMux);
		} else {
			/* The task unblocked because the bits were set. */
		}
		/* The task blocked so control bits may have been set. */
		uxReturn &= ~eventEVENT_BITS_CONTROL_BYTES;
	}
	return uxReturn;
}

EventBits_t xEventGroupClearBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear)
{
	EventGroup_t *pxEventBits = (EventGroup_t *) xEventGroup;
	EventBits_t uxReturn;

	/* Check the user is not attempting to clear the bits used by the kernel
	   itself. */
	pthread_mutex_lock(&pxEventBits->eventGroupMux);

	/* The value returned is the event group value prior to the bits being
	   cleared. */
	uxReturn = pxEventBits->uxEventBits;

	/* Clear the bits. */
	pxEventBits->uxEventBits &= ~uxBitsToClear;

	pthread_mutex_unlock(&pxEventBits->eventGroupMux);

	return uxReturn;
}

EventBits_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet)
{
	EventBits_t uxBitsToClear = 0, uxBitsWaitedFor, uxControlBits;
	EventGroup_t *pxEventBits = (EventGroup_t *) xEventGroup;
	BaseType_t xMatchFound = pdFALSE;

	/* Check the user is not attempting to set the bits used by the kernel
	   itself. */
	if (!pxEventBits || (uxBitsToSet & eventEVENT_BITS_CONTROL_BYTES) != 0) {
		return -1;
	}

	sched_lock();
	pthread_mutex_lock(&pxEventBits->eventGroupMux);

	/* Set the bits. */
	pxEventBits->uxEventBits |= uxBitsToSet;

	event_node_t *pevent_node = (event_node_t *) pxEventBits->xTasksWaitingForBits.head;
	while (pevent_node) {
		//get the value
		uxBitsWaitedFor = pevent_node->xItemValue;
		xMatchFound = pdFALSE;
		/* Split the bits waited for from the control bits. */
		uxControlBits = uxBitsWaitedFor & eventEVENT_BITS_CONTROL_BYTES;
		uxBitsWaitedFor &= ~eventEVENT_BITS_CONTROL_BYTES;

		if ((uxControlBits & eventWAIT_FOR_ALL_BITS) == (EventBits_t) 0) {
			/* Just looking for single bit being set. */
			if ((uxBitsWaitedFor & pxEventBits->uxEventBits) != (EventBits_t) 0) {
				xMatchFound = pdTRUE;
			}
		} else if ((uxBitsWaitedFor & pxEventBits->uxEventBits) == uxBitsWaitedFor) {
			/* All bits are set. */
			xMatchFound = pdTRUE;
		} else {
			/* Need all bits to be set, but not all the bits were set. */
		}

		if (xMatchFound != pdFALSE) {
			/* The bits match.  Should the bits be cleared on exit? */
			if ((uxControlBits & eventCLEAR_EVENTS_ON_EXIT_BIT) != (EventBits_t) 0) {
				uxBitsToClear |= uxBitsWaitedFor;
			}

			/* Store the actual event flag value in the task's event list
			   item before removing the task from the event list.   The
			   eventUNBLOCKED_DUE_TO_BIT_SET bit is set so the task knows
			   that is was unblocked due to its required bits matching, rather
			   than because it timed out. */

			(void)xTaskRemoveFromUnorderedEventList(pevent_node, &pxEventBits->xTasksWaitingForBits, pxEventBits->uxEventBits | eventUNBLOCKED_DUE_TO_BIT_SET);
		}

		pevent_node = (event_node_t *) pevent_node->node.flink;
	}

	/* Clear any bits that matched when the eventCLEAR_EVENTS_ON_EXIT_BIT
	   bit was set in the control word. */
	pxEventBits->uxEventBits &= ~uxBitsToClear;

	pthread_mutex_unlock(&pxEventBits->eventGroupMux);
	sched_unlock();

	return pxEventBits->uxEventBits;
}

void vEventGroupDelete(EventGroupHandle_t xEventGroup)
{
	EventGroup_t *pxEventBits = (EventGroup_t *) xEventGroup;
	sq_queue_t *pxTasksWaitingForBits = &(pxEventBits->xTasksWaitingForBits);
	event_node_t *pevent_node = NULL;
	event_node_t *pevent_node_temp = NULL;

	sched_lock();
	pthread_mutex_lock(&pxEventBits->eventGroupMux);

	pevent_node = (event_node_t *) pxEventBits->xTasksWaitingForBits.head;
	while (pevent_node) {
		xTaskRemoveFromUnorderedEventList(pevent_node, pxTasksWaitingForBits, eventUNBLOCKED_DUE_TO_BIT_SET);
		pevent_node_temp = pevent_node;
		pevent_node = (event_node_t *) pevent_node_temp->node.flink;
		free(pevent_node_temp);
	}

	pthread_mutex_unlock(&pxEventBits->eventGroupMux);	//Exit mux of event group before deleting it
	free(pxEventBits);

	sched_unlock();
}
