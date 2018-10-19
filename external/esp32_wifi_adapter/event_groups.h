/*
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

	***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#ifndef EVENT_GROUPS_H
#define EVENT_GROUPS_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


/* Type definitions. */

typedef _int8_t int8_t;
typedef _uint8_t uint8_t;

typedef _int16_t int16_t;
typedef _uint16_t uint16_t;

typedef _int32_t int32_t;
typedef _uint32_t uint32_t;

#ifdef __INT64_DEFINED
typedef _int64_t int64_t;
typedef _uint64_t uint64_t;
#endif

#define portCHAR		int8_t
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		int32_t
#define portSHORT		int16_t
#define portSTACK_TYPE	uint8_t
#define portBASE_TYPE	int

typedef portSTACK_TYPE			StackType_t;
typedef portBASE_TYPE			BaseType_t;
typedef unsigned portBASE_TYPE	UBaseType_t;

#if( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#endif


#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )
#define pdPASS          ( pdTRUE )
#define pdFAIL          ( pdFALSE)
#define OSI_FUNCS_TIME_BLOCKING  0xffffffff

/**
 * An event group is a collection of bits to which an application can assign a
 * meaning.  For example, an application may create an event group to convey
 * the status of various CAN bus related events in which bit 0 might mean "A CAN
 * message has been received and is ready for processing", bit 1 might mean "The
 * application has queued a message that is ready for sending onto the CAN
 * network", and bit 2 might mean "It is time to send a SYNC message onto the
 * CAN network" etc.  A task can then test the bit values to see which events
 * are active, and optionally enter the Blocked state to wait for a specified
 * bit or a group of specified bits to be active.  To continue the CAN bus
 * example, a CAN controlling task can enter the Blocked state (and therefore
 * not consume any processing time) until either bit 0, bit 1 or bit 2 are
 * active, at which time the bit that was actually active would inform the task
 * which action it had to take (process a received message, send a message, or
 * send a SYNC).
 *
 * The event groups implementation contains intelligence to avoid race
 * conditions that would otherwise occur were an application to use a simple
 * variable for the same purpose.  This is particularly important with respect
 * to when a bit within an event group is to be cleared, and when bits have to
 * be set and then tested atomically - as is the case where event groups are
 * used to create a synchronisation point between multiple tasks (a
 * 'rendezvous').
 *
 */



/**
 * event_groups.h
 *
 * Type by which event groups are referenced.  For example, a call to
 * xEventGroupCreate() returns an EventGroupHandle_t variable that can then
 * be used as a parameter to other event group functions.
 *
 * \ingroup EventGroup
 */
typedef void * EventGroupHandle_t;

/*
 * The type that holds event bits always matches TickType_t - therefore the
 * number of bits it holds is set by configUSE_16_BIT_TICKS (16 bits if set to 1,
 * 32 bits if set to 0.
 *
 * \ingroup EventGroup
 */
typedef TickType_t EventBits_t;

/**
 * Create a new event group.
 *
 * Internally, within the FreeRTOS implementation, event groups use a [small]
 * block of memory, in which the event group's structure is stored.  If an event
 * groups is created using xEventGroupCreate() then the required memory is
 * automatically dynamically allocated inside the xEventGroupCreate() function.
 * (see http://www.freertos.org/a00111.html).  If an event group is created
 * using xEventGropuCreateStatic() then the application writer must instead
 * provide the memory that will get used by the event group.
 * xEventGroupCreateStatic() therefore allows an event group to be created
 * without using any dynamic memory allocation.
 *
 * Although event groups are not related to ticks, for internal implementation
 * reasons the number of bits available for use in an event group is dependent
 * on the configUSE_16_BIT_TICKS setting in FreeRTOSConfig.h.  If
 * configUSE_16_BIT_TICKS is 1 then each event group contains 8 usable bits (bit
 * 0 to bit 7).  If configUSE_16_BIT_TICKS is set to 0 then each event group has
 * 24 usable bits (bit 0 to bit 23).  The EventBits_t type is used to store
 * event bits within an event group.
 *
 * @return If the event group was created then a handle to the event group is
 * returned.  If there was insufficient FreeRTOS heap available to create the
 * event group then NULL is returned.  See http://www.freertos.org/a00111.html
 *
 * Example usage:
 * @code{c}
 * 	// Declare a variable to hold the created event group.
 * 	EventGroupHandle_t xCreatedEventGroup;
 *
 * 	// Attempt to create the event group.
 * 	xCreatedEventGroup = xEventGroupCreate();
 *
 * 	// Was the event group created successfully?
 * 	if( xCreatedEventGroup == NULL )
 * 	{
 * 		// The event group was not created because there was insufficient
 * 		// FreeRTOS heap available.
 * 	}
 * 	else
 * 	{
 * 		// The event group was created.
 * 	}
 * @endcode
 * \ingroup EventGroup
 */

EventGroupHandle_t xEventGroupCreate( void );




/**
 * [Potentially] block to wait for one or more bits to be set within a
 * previously created event group.
 *
 * This function cannot be called from an interrupt.
 *
 * @param xEventGroup The event group in which the bits are being tested.  The
 * event group must have previously been created using a call to
 * xEventGroupCreate().
 *
 * @param uxBitsToWaitFor A bitwise value that indicates the bit or bits to test
 * inside the event group.  For example, to wait for bit 0 and/or bit 2 set
 * uxBitsToWaitFor to 0x05.  To wait for bits 0 and/or bit 1 and/or bit 2 set
 * uxBitsToWaitFor to 0x07.  Etc.
 *
 * @param xClearOnExit If xClearOnExit is set to pdTRUE then any bits within
 * uxBitsToWaitFor that are set within the event group will be cleared before
 * xEventGroupWaitBits() returns if the wait condition was met (if the function
 * returns for a reason other than a timeout).  If xClearOnExit is set to
 * pdFALSE then the bits set in the event group are not altered when the call to
 * xEventGroupWaitBits() returns.
 *
 * @param xWaitForAllBits If xWaitForAllBits is set to pdTRUE then
 * xEventGroupWaitBits() will return when either all the bits in uxBitsToWaitFor
 * are set or the specified block time expires.  If xWaitForAllBits is set to
 * pdFALSE then xEventGroupWaitBits() will return when any one of the bits set
 * in uxBitsToWaitFor is set or the specified block time expires.  The block
 * time is specified by the xTicksToWait parameter.
 *
 * @param xTicksToWait The maximum amount of time (specified in 'ticks') to wait
 * for one/all (depending on the xWaitForAllBits value) of the bits specified by
 * uxBitsToWaitFor to become set.
 *
 * @return The value of the event group at the time either the bits being waited
 * for became set, or the block time expired.  Test the return value to know
 * which bits were set.  If xEventGroupWaitBits() returned because its timeout
 * expired then not all the bits being waited for will be set.  If
 * xEventGroupWaitBits() returned because the bits it was waiting for were set
 * then the returned value is the event group value before any bits were
 * automatically cleared in the case that xClearOnExit parameter was set to
 * pdTRUE.
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *    const TickType_t xTicksToWait = 100 / portTICK_PERIOD_MS;
 *
 * 		// Wait a maximum of 100ms for either bit 0 or bit 4 to be set within
 * 		// the event group.  Clear the bits before exiting.
 * 		uxBits = xEventGroupWaitBits(
 * 					xEventGroup,	// The event group being tested.
 * 					BIT_0 | BIT_4,	// The bits within the event group to wait for.
 * 					pdTRUE,			// BIT_0 and BIT_4 should be cleared before returning.
 * 					pdFALSE,		// Don't wait for both bits, either bit will do.
 * 					xTicksToWait );	// Wait a maximum of 100ms for either bit to be set.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// xEventGroupWaitBits() returned because both bits were set.
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// xEventGroupWaitBits() returned because just BIT_0 was set.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// xEventGroupWaitBits() returned because just BIT_4 was set.
 * 		}
 * 		else
 * 		{
 * 			// xEventGroupWaitBits() returned because xTicksToWait ticks passed
 * 			// without either BIT_0 or BIT_4 becoming set.
 * 		}
 *    }
 * @endcode{c}
 * \ingroup EventGroup
 */
EventBits_t xEventGroupWaitBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToWaitFor, const BaseType_t xClearOnExit, const BaseType_t xWaitForAllBits, TickType_t xTicksToWait );

/**
 * Clear bits within an event group.  This function cannot be called from an
 * interrupt.
 *
 * @param xEventGroup The event group in which the bits are to be cleared.
 *
 * @param uxBitsToClear A bitwise value that indicates the bit or bits to clear
 * in the event group.  For example, to clear bit 3 only, set uxBitsToClear to
 * 0x08.  To clear bit 3 and bit 0 set uxBitsToClear to 0x09.
 *
 * @return The value of the event group before the specified bits were cleared.
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *
 * 		// Clear bit 0 and bit 4 in xEventGroup.
 * 		uxBits = xEventGroupClearBits(
 * 								xEventGroup,	// The event group being updated.
 * 								BIT_0 | BIT_4 );// The bits being cleared.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// Both bit 0 and bit 4 were set before xEventGroupClearBits() was
 * 			// called.  Both will now be clear (not set).
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// Bit 0 was set before xEventGroupClearBits() was called.  It will
 * 			// now be clear.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// Bit 4 was set before xEventGroupClearBits() was called.  It will
 * 			// now be clear.
 * 		}
 * 		else
 * 		{
 * 			// Neither bit 0 nor bit 4 were set in the first place.
 * 		}
 *    }
 * @endcode
 * \ingroup EventGroup
 */
EventBits_t xEventGroupClearBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToClear );


/**
 * Set bits within an event group.
 * This function cannot be called from an interrupt.  xEventGroupSetBitsFromISR()
 * is a version that can be called from an interrupt.
 *
 * Setting bits in an event group will automatically unblock tasks that are
 * blocked waiting for the bits.
 *
 * @param xEventGroup The event group in which the bits are to be set.
 *
 * @param uxBitsToSet A bitwise value that indicates the bit or bits to set.
 * For example, to set bit 3 only, set uxBitsToSet to 0x08.  To set bit 3
 * and bit 0 set uxBitsToSet to 0x09.
 *
 * @return The value of the event group at the time the call to
 * xEventGroupSetBits() returns.  There are two reasons why the returned value
 * might have the bits specified by the uxBitsToSet parameter cleared.  First,
 * if setting a bit results in a task that was waiting for the bit leaving the
 * blocked state then it is possible the bit will be cleared automatically
 * (see the xClearBitOnExit parameter of xEventGroupWaitBits()).  Second, any
 * unblocked (or otherwise Ready state) task that has a priority above that of
 * the task that called xEventGroupSetBits() will execute and may change the
 * event group value before the call to xEventGroupSetBits() returns.
 *
 * Example usage:
 * @code{c}
 *    #define BIT_0	( 1 << 0 )
 *    #define BIT_4	( 1 << 4 )
 *
 *    void aFunction( EventGroupHandle_t xEventGroup )
 *    {
 *    EventBits_t uxBits;
 *
 * 		// Set bit 0 and bit 4 in xEventGroup.
 * 		uxBits = xEventGroupSetBits(
 * 							xEventGroup,	// The event group being updated.
 * 							BIT_0 | BIT_4 );// The bits being set.
 *
 * 		if( ( uxBits & ( BIT_0 | BIT_4 ) ) == ( BIT_0 | BIT_4 ) )
 * 		{
 * 			// Both bit 0 and bit 4 remained set when the function returned.
 * 		}
 * 		else if( ( uxBits & BIT_0 ) != 0 )
 * 		{
 * 			// Bit 0 remained set when the function returned, but bit 4 was
 * 			// cleared.  It might be that bit 4 was cleared automatically as a
 * 			// task that was waiting for bit 4 was removed from the Blocked
 * 			// state.
 * 		}
 * 		else if( ( uxBits & BIT_4 ) != 0 )
 * 		{
 * 			// Bit 4 remained set when the function returned, but bit 0 was
 * 			// cleared.  It might be that bit 0 was cleared automatically as a
 * 			// task that was waiting for bit 0 was removed from the Blocked
 * 			// state.
 * 		}
 * 		else
 * 		{
 * 			// Neither bit 0 nor bit 4 remained set.  It might be that a task
 * 			// was waiting for both of the bits to be set, and the bits were
 * 			// cleared as the task left the Blocked state.
 * 		}
 *    }
 * @endcode{c}
 * \ingroup EventGroup
 */
EventBits_t xEventGroupSetBits( EventGroupHandle_t xEventGroup, const EventBits_t uxBitsToSet );



/**
 *
 * Delete an event group that was previously created by a call to
 * xEventGroupCreate().  Tasks that are blocked on the event group will be
 * unblocked and obtain 0 as the event group's value.
 *
 * @param xEventGroup The event group being deleted.
 */
void vEventGroupDelete( EventGroupHandle_t xEventGroup );



/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* EVENT_GROUPS_H */


