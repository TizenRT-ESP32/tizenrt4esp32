/****************************************************************************
 *
 * Copyright 2016-2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * mm/mm_heap/mm_heapinfo.c
 *
 *   Copyright (C) 2007, 2009, 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/sched.h>
#include <tinyara/mm/mm.h>
#include <tinyara/arch.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#ifdef CONFIG_HEAPINFO_USER_GROUP
#include <string.h>
#include <tinyara/mm/heapinfo_internal.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MM_PIDHASH(pid) ((pid) & (CONFIG_MAX_TASKS - 1))
#define HEAPINFO_INT INT16_MAX
#define HEAPINFO_NONSCHED (INT16_MAX - 1)

#ifdef CONFIG_HEAPINFO_USER_GROUP
struct heapinfo_group_info_s group_info[HEAPINFO_THREAD_NUM];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_MM_HEAPINFO
/****************************************************************************
 * Name: heapinfo_parse
 *
 * Description:
 *   This function walk through heap and displays alloc info.
 ****************************************************************************/
void heapinfo_parse(FAR struct mm_heap_s *heap, int mode, pid_t pid)
{
	struct mm_allocnode_s *node;
	size_t mxordblk = 0;
	int    ordblks  = 0;		/* Number of non-inuse chunks */
	size_t fordblks = 0;		/* Total non-inuse space */
	size_t heap_resource;
	size_t stack_resource;
	size_t nonsched_resource;
	int nonsched_idx;

	/* This nonsched can be 3 types : group resources, freed when child task finished, leak */
	pid_t nonsched_list[CONFIG_MAX_TASKS];
	size_t nonsched_size[CONFIG_MAX_TASKS];

#if CONFIG_MM_REGIONS > 1
	int region;
#else
#define region 0
#endif
	/* initialize the heap, stack and nonsched resource */
	nonsched_resource = 0;
	heap_resource = 0;
	stack_resource = 0;
	for (nonsched_idx = 0; nonsched_idx < CONFIG_MAX_TASKS; nonsched_idx++) {
		nonsched_list[nonsched_idx] = HEAPINFO_NONSCHED;
		nonsched_size[nonsched_idx] = 0;
	}

	/* Visit each region */

#if CONFIG_MM_REGIONS > 1
	for (region = 0; region < heap->mm_nregions; region++)
#endif
	{
		/* Visit each node in the region
		 * Retake the semaphore for each region to reduce latencies
		 */
		mm_takesemaphore(heap);

		if (mode != HEAPINFO_SIMPLE) {
			printf("****************************************************************\n");
			printf("REGION #%d Start=0x%p, End=0x%p, Size=%d\n",
				region,
				heap->mm_heapstart[region],
				heap->mm_heapend[region],
				(int)heap->mm_heapend[region] - (int)heap->mm_heapstart[region] + SIZEOF_MM_ALLOCNODE);
			printf("****************************************************************\n");
			printf("Allocation Info- (Size in Bytes)\n");
			printf("****************************************************************\n");
			printf("  MemAddr |   Size   | Status |    Owner   |  Pid  |\n");
			printf("----------|----------|--------|------------|-------|\n");
		}

		for (node = heap->mm_heapstart[region]; node < heap->mm_heapend[region]; node = (struct mm_allocnode_s *)((char *)node + node->size)) {

			/* Check if the node corresponds to an allocated memory chunk */
			if ((pid == HEAPINFO_PID_ALL || node->pid == pid) && (node->preceding & MM_ALLOC_BIT) != 0) {
				if (mode == HEAPINFO_DETAIL_ALL || mode == HEAPINFO_DETAIL_PID) {
					if (node->pid >= 0) {
						printf("0x%x | %8u |   %c    | 0x%8x | %3d   |\n", node, node->size, 'A', node->alloc_call_addr, node->pid);
					} else {
						printf("0x%x | %8u |   %c    | 0x%8x | %3d(S)|\n", node, node->size, 'A', node->alloc_call_addr, -(node->pid));
					}
				}

#if CONFIG_TASK_NAME_SIZE > 0
				if (node->pid == HEAPINFO_INT && mode != HEAPINFO_SIMPLE) {
					printf("INT Context\n");
				} else if (node->pid < 0 && sched_gettcb((-1) * (node->pid)) != NULL) {
					stack_resource += node->size;
				} else if (sched_gettcb(node->pid) == NULL) {
					nonsched_list[MM_PIDHASH(node->pid)] = node->pid;
					nonsched_size[MM_PIDHASH(node->pid)] += node->size;
					nonsched_resource += node->size;
				} else {
					heap_resource += node->size;
				}
#else
				printf("\n");
#endif
			} else {
				ordblks++;
				fordblks += node->size;
				if (node->size > mxordblk) {
					mxordblk = node->size;
				}
				if (mode == HEAPINFO_DETAIL_ALL || mode == HEAPINFO_DETAIL_FREE) {
					printf("0x%x | %8d |   %c    |            |       |\n", node, node->size, 'F');
				}
			}
		}

		if (mode != HEAPINFO_SIMPLE) {
			printf("** PID(S) in Pid colum means that mem is used for stack of PID\n");
		}
		printf("\n");
		mm_givesemaphore(heap);
	}
#undef region
	printf("\n****************************************************************\n");
	printf("     Summary of Heap Usages (Size in Bytes)\n");
	printf("****************************************************************\n");
	printf("Total                           : %u (100%%)\n", heap->mm_heapsize);
	printf("  - Allocated (Current / Peak)  : %u (%d%%) / %u (%d%%)\n",\
		heap->total_alloc_size, heap->total_alloc_size * 100 / heap->mm_heapsize,\
		heap->peak_alloc_size,  heap->peak_alloc_size * 100 / heap->mm_heapsize);
	printf("  - Free (Current)              : %u (%d%%)\n", fordblks, fordblks * 100 / heap->mm_heapsize);
	printf("  - Reserved                    : %u\n", SIZEOF_MM_ALLOCNODE * 2);

	printf("\n****************************************************************\n");
	printf("     Details of Heap Usages (Size in Bytes)\n");
	printf("****************************************************************\n");
	printf("< Free >\n");
	printf("  - Number of Free Node               : %d\n", ordblks);
	printf("  - Largest Free Node Size            : %u\n", mxordblk);
	printf("\n< Allocation >\n");
	printf("  - Current Size (Alive Allocation) = (1) + (2) + (3)\n");
	printf("     . by Dead Threads (*) (1)        : %u\n", nonsched_resource);
	printf("     . by Alive Threads\n");
	printf("        - Sum of \"STACK\"(**) (2)      : %u\n", stack_resource);
	printf("        - Sum of \"CURR_HEAP\" (3)      : %u\n", heap_resource - SIZEOF_MM_ALLOCNODE);	// Because of above for loop (node < heap->mm_heapend[region];),
													// one of SIZEOF_MM_ALLOCNODE is subtracted.
	printf("** NOTE **\n");
	printf("(*)  Alive allocation by dead threads might be used by others or might be a leakage.\n");
	printf("(**) Only Idle task has a separate stack region,\n");
	printf("  rest are all allocated on the heap region.\n");

	if (mode != HEAPINFO_SIMPLE) {
		printf("\n< by Dead Threads >\n");
		printf(" Pid | Size \n");
		printf("-----|------\n");
		for (nonsched_idx = 0; nonsched_idx < CONFIG_MAX_TASKS; nonsched_idx++) {
			if (nonsched_list[nonsched_idx] != HEAPINFO_NONSCHED) {
				printf("%4d | %5u\n", nonsched_list[nonsched_idx], nonsched_size[nonsched_idx]);
			}
		}
	}

	return;
}
/****************************************************************************
 * Name: heapinfo_update_group
 *
 * Description:
 * Update Peak heap size for Group
 ****************************************************************************/
#ifdef CONFIG_HEAPINFO_USER_GROUP
static void heapinfo_update_group(mmsize_t size, pid_t pid)
{
	int check_idx;
	int group_num;
	int stack_pid;
	struct mm_heap_s *heap = mm_get_heap_info();
	for (check_idx = 0; check_idx < HEAPINFO_THREAD_NUM; check_idx++) {
		if (pid == group_info[check_idx].pid) {
			group_num = group_info[check_idx].group;
			heap->group[group_num].curr_size += size;
			/* Update peak size */
			if (heap->group[group_num].curr_size > heap->group[group_num].peak_size) {
				heap->group[group_num].peak_size = heap->group[group_num].curr_size;
				/* calculate the summation of stacks */
				heap->group[group_num].stack_size = 0;
				for (stack_pid = 0; stack_pid < HEAPINFO_THREAD_NUM; stack_pid++) {
					if (group_info[stack_pid].pid != -1 && group_info[stack_pid].group == group_num) {
						heap->group[group_num].stack_size += group_info[stack_pid].stack_size;
					}
				}
				heap->group[group_num].heap_size = heap->group[group_num].peak_size - heap->group[group_num].stack_size;
			}
			break;
		}
	}
}
#endif
/****************************************************************************
 * Name: heapinfo_add_size
 *
 * Description:
 * Add the allocated size in tcb
 ****************************************************************************/
void heapinfo_add_size(pid_t pid, mmsize_t size)
{
	struct tcb_s *rtcb = sched_gettcb(pid);
	if (rtcb) {
		rtcb->curr_alloc_size += size;
		rtcb->num_alloc_free++;
		if (rtcb->curr_alloc_size > rtcb->peak_alloc_size) {
			rtcb->peak_alloc_size = rtcb->curr_alloc_size;
		}
	}
}

/****************************************************************************
 * Name: heapinfo_subtract_size
 *
 * Description:
 * Subtract the allocated size in tcb
 ****************************************************************************/
void heapinfo_subtract_size(pid_t pid, mmsize_t size)
{
	struct tcb_s *rtcb = sched_gettcb(pid);

	if (rtcb) {
		rtcb->curr_alloc_size -= size;
		rtcb->num_alloc_free--;
	}
}

/****************************************************************************
 * Name: heapinfo_update_total_size
 *
 * Description:
 * Calculate the total allocated size and update the peak allocated size for heap
 ****************************************************************************/
void heapinfo_update_total_size(struct mm_heap_s *heap, mmsize_t size, pid_t pid)
{
	heap->total_alloc_size += size;
	if (heap->total_alloc_size > heap->peak_alloc_size) {
		heap->peak_alloc_size = heap->total_alloc_size;
	}
#ifdef CONFIG_HEAPINFO_USER_GROUP
	heapinfo_update_group(size, pid);
#endif
}
/****************************************************************************
 * Name: heapinfo_update_node
 *
 * Description:
 * Adds pid and malloc caller return address to mem chunk
 ****************************************************************************/
void heapinfo_update_node(FAR struct mm_allocnode_s *node, mmaddress_t caller_retaddr)
{
	node->alloc_call_addr = caller_retaddr;
	node->reserved = 0;
	if (up_interrupt_context() == true) {
		/* update pid as HEAPINFO_INT(-1) if allocation is from INT context */
		node->pid = HEAPINFO_INT;
	} else {
		node->pid = getpid();
	}
	return;
}

/****************************************************************************
 * Name: heapinfo_exclude_stacksize
 *
 * Description:
 * when create a stack, subtract the stacksize from parent
 ****************************************************************************/
void heapinfo_exclude_stacksize(void *stack_ptr)
{
	struct mm_allocnode_s *node;
	struct tcb_s *rtcb;

	node = (struct mm_allocnode_s *)(stack_ptr - SIZEOF_MM_ALLOCNODE);
	rtcb = sched_gettcb(node->pid);

	ASSERT(rtcb);
	rtcb->curr_alloc_size -= node->size;

#ifdef CONFIG_HEAPINFO_USER_GROUP
	int check_idx;
	int group_num;
	struct mm_heap_s *heap = mm_get_heap_info();

	for (check_idx = 0; check_idx <= heap->max_group; check_idx++) {
		if (node->pid == group_info[check_idx].pid) {
			group_num = group_info[check_idx].group;
			heap->group[group_num].curr_size -= node->size;
			break;
		}
	}
#endif
}

#ifdef CONFIG_HEAPINFO_USER_GROUP
/****************************************************************************
 * Name: heapinfo_update_group_info
 *
 * Description:
 * when create or release task/thread, check that the task/thread is 
 * in group list
 ****************************************************************************/
void heapinfo_update_group_info(pid_t pid, int group, int type)
{
	int info_idx;
	struct tcb_s *tcb;
	struct mm_heap_s *heap = mm_get_heap_info();
	switch (type) {
	case HEAPINFO_INIT_INFO:
		for (info_idx = 0; info_idx < HEAPINFO_THREAD_NUM; info_idx++) {
			group_info[info_idx].pid = pid;
			group_info[info_idx].group = group;
			group_info[info_idx].stack_size = 0;
		}
		break;
	case HEAPINFO_ADD_INFO:
		if (group > heap->max_group) {
			heap->max_group = group;
		}
		for (info_idx = 0; info_idx < HEAPINFO_THREAD_NUM; info_idx++) {
			if (group_info[info_idx].pid <= 0) {
				group_info[info_idx].pid = pid;
				group_info[info_idx].group = group;
				tcb = sched_gettcb(pid);
				if (tcb) {
					group_info[info_idx].stack_size = tcb->adj_stack_size;
					heapinfo_update_group(tcb->adj_stack_size, pid);
				}
				break;
			}
		}
		break;
	case HEAPINFO_DEL_INFO:
		for (info_idx = 0; info_idx < HEAPINFO_THREAD_NUM; info_idx++) {
			if (pid == group_info[info_idx].pid) {
				heapinfo_update_group((-1) * group_info[info_idx].stack_size, pid);

				group_info[info_idx].pid = -1;
				group_info[info_idx].group = -1;
				group_info[info_idx].stack_size = 0;
				break;
			}
		}
		break;
	default:
		break;
	}
}

/****************************************************************************
 * Name: heapinfo_check_group_list
 *
 * Description:
 * check that task/thread is in group list
 ****************************************************************************/
void heapinfo_check_group_list(pid_t pid, char *name)
{
	char *thread_list = CONFIG_HEAPINFO_USER_GROUP_LIST;

	int group_num = 0;

	char *name_start;
	char *name_end;
	int name_length;
	name_start = name_end = thread_list;

	while (*name_start != '\0') {
		if (*name_end == '/' || *name_end == ',' || *name_end == '\0') {
			name_length = name_end - name_start;
			if (name_length == strlen(name)) {
				if (strncmp(name_start, name, name_end - name_start) == 0) {
					heapinfo_update_group_info(pid, group_num, HEAPINFO_ADD_INFO);
					break;
				}
			}
			if (*name_end == '/') {
				group_num++;
			} else if (*name_end == '\0') {
				name_start = name_end;
				continue;
			}
			name_end++;
			name_start = name_end;
		} else {
			name_end++;
		}
	}
}
#endif /* CONFIG_HEAPINFO_USER_GROUP */
#endif
