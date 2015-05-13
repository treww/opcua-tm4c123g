/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
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

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

#ifdef __GNUC__
  #ifdef __MINGW32__
	  #include "mmsystem.h"
  #else
    #include <errno.h>
    #include <pthread.h>
    #include <sched.h>
    #include <stdlib.h>
  #endif
#else
	#pragma comment(lib, "winmm.lib")
#endif

#define portMAX_INTERRUPTS				( ( uint32_t ) sizeof( uint32_t ) * 8UL ) /* The number of bits in an uint32_t. */
#define portNO_CRITICAL_NESTING 		( ( uint32_t ) 0 )

/*
 * Created as a high priority thread, this function uses a timer to simulate
 * a tick interrupt being generated on an embedded target.  In this Windows
 * environment the timer does not achieve anything approaching real time
 * performance though.
 */
#ifdef __linux__
static void* prvSimulatedPeripheralTimer( void* lpParameter );
#else // windows
static DWORD WINAPI prvSimulatedPeripheralTimer( LPVOID lpParameter );
#endif


/*
 * Process all the simulated interrupts - each represented by a bit in
 * ulPendingInterrupts variable.
 */
static void prvProcessSimulatedInterrupts( void );

/*
 * Interrupt handlers used by the kernel itself.  These are executed from the
 * simulated interrupt handler thread.
 */
static uint32_t prvProcessYieldInterrupt( void );
static uint32_t prvProcessTickInterrupt( void );

/*
 * Called when the process exits to let Windows know the high timer resolution
 * is no longer required.
 */

#ifdef __linux__
static int32_t prvEndProcess( int32_t dwCtrlType );
#else // windows
static BOOL WINAPI prvEndProcess( DWORD dwCtrlType );
#endif

/*-----------------------------------------------------------*/

/* The WIN32 simulator runs each task in a thread.  The context switching is
managed by the threads, so the task stack does not have to be managed directly,
although the task stack is still used to hold an xThreadState structure this is
the only thing it will ever hold.  The structure indirectly maps the task handle
to a thread handle. */
typedef struct
{
	/* Handle of the thread that executes the task. */
#ifdef __linux__
  pthread_t pvThread;
  TaskFunction_t pxCode;
  void *pvParameters;
#else
	void *pvThread;
#endif

} xThreadState;

/* Simulated interrupts waiting to be processed.  This is a bit mask where each
bit represents one interrupt, so a maximum of 32 interrupts can be simulated. */
static volatile uint32_t ulPendingInterrupts = 0UL;

/* An event used to inform the simulated interrupt processing thread (a high
priority thread that simulated interrupt processing) that an interrupt is
pending. */
#ifdef __linux__
static pthread_mutex_t InterruptMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t* pvInterruptMutex = &InterruptMutex;
static pthread_cond_t InterruptCond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t* pvInterruptEvent = &InterruptCond;

static pthread_mutex_t StartMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t StartCond = PTHREAD_COND_INITIALIZER;

int MinPriority = 0;
int MaxPriority = 0;

static void Initialize(void)
{
  if (MaxPriority)
    return;

  MinPriority = sched_get_priority_min(SCHED_FIFO);
  MaxPriority = sched_get_priority_max(SCHED_FIFO);
}

static void AbortIfFail(int error, char* file, int line)
{
  if (error)
  {
    printf("%s:%d: %s (%d)", file, line, strerror(error), error);
    abort();
  }
}

#else
static void *pvInterruptEvent = NULL;
#endif

/* Mutex used to protect all the simulated interrupt variables that are accessed
by multiple threads. */
#ifdef __linux__
static pthread_mutex_t InterruptEventMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t* pvInterruptEventMutex = &InterruptEventMutex;
#else
static void *pvInterruptEventMutex = NULL;
#endif

/* The critical nesting count for the currently executing task.  This is
initialised to a non-zero value so interrupts do not become enabled during
the initialisation phase.  As each task has its own critical nesting value
ulCriticalNesting will get set to zero when the first task runs.  This
initialisation is probably not critical in this simulated environment as the
simulated interrupt handlers do not get created until the FreeRTOS scheduler is
started anyway. */
static uint32_t ulCriticalNesting = 9999UL;

/* Handlers for all the simulated software interrupts.  The first two positions
are used for the Yield and Tick interrupts so are handled slightly differently,
all the other interrupts can be user defined. */
static uint32_t (*ulIsrHandler[ portMAX_INTERRUPTS ])( void ) = { 0 };

/* Pointer to the TCB of the currently executing task. */
extern void *pxCurrentTCB;

/* Used to ensure nothing is processed during the startup sequence. */
static BaseType_t xPortRunning = pdFALSE;

/*-----------------------------------------------------------*/
#ifdef __linux__
static void* prvSimulatedPeripheralTimer( void* lpParameter )
{
    /* Just to prevent compiler warnings. */
    ( void ) lpParameter;

    struct timespec t = {portTICK_PERIOD_MS /1000, portTICK_PERIOD_MS * 1000000};
    int ret = 0;
    for( ;; )
    {
      /* Wait until the timer expires and we can access the simulated interrupt
      variables.  *NOTE* this is not a 'real time' way of generating tick
      events as the next wake time should be relative to the previous wake
      time, not the time that Sleep() is called.  It is done this way to
      prevent overruns in this very non real time simulated/emulated
      environment. */
      do
      {
        ret = nanosleep(&t, &t);
      }
      while (ret == EINTR);

      configASSERT( xPortRunning );

      AbortIfFail(pthread_mutex_lock(pvInterruptEventMutex), __FILE__, __LINE__);

      /* The timer has expired, generate the simulated tick event. */
      ulPendingInterrupts |= ( 1 << portINTERRUPT_TICK );

      /* The interrupt is now pending - notify the simulated interrupt
      handler thread. */
      if( ulCriticalNesting == 0 )
      {
        AbortIfFail(pthread_cond_signal(pvInterruptEvent),__FILE__, __LINE__);
      }

      /* Give back the mutex so the simulated interrupt handler unblocks
      and can access the interrupt handler variables. */
      AbortIfFail(pthread_mutex_unlock( pvInterruptEventMutex ), __FILE__, __LINE__);
      t.tv_sec = portTICK_PERIOD_MS /1000;
      t.tv_nsec = portTICK_PERIOD_MS * 1000000;
    }

    #ifdef __GNUC__
      /* Should never reach here - MingW complains if you leave this line out,
      MSVC complains if you put it in. */
      return 0;
    #endif
}
#else // windows
static DWORD WINAPI prvSimulatedPeripheralTimer( LPVOID lpParameter )
{
TickType_t xMinimumWindowsBlockTime;
TIMECAPS xTimeCaps;

	/* Set the timer resolution to the maximum possible. */
	if( timeGetDevCaps( &xTimeCaps, sizeof( xTimeCaps ) ) == MMSYSERR_NOERROR )
	{
		xMinimumWindowsBlockTime = ( TickType_t ) xTimeCaps.wPeriodMin;
		timeBeginPeriod( xTimeCaps.wPeriodMin );

		/* Register an exit handler so the timeBeginPeriod() function can be
		matched with a timeEndPeriod() when the application exits. */
		SetConsoleCtrlHandler( prvEndProcess, TRUE );
	}
	else
	{
		xMinimumWindowsBlockTime = ( TickType_t ) 20;
	}

	/* Just to prevent compiler warnings. */
	( void ) lpParameter;

	for( ;; )
	{
		/* Wait until the timer expires and we can access the simulated interrupt
		variables.  *NOTE* this is not a 'real time' way of generating tick
		events as the next wake time should be relative to the previous wake
		time, not the time that Sleep() is called.  It is done this way to
		prevent overruns in this very non real time simulated/emulated
		environment. */
		if( portTICK_PERIOD_MS < xMinimumWindowsBlockTime )
		{
			Sleep( xMinimumWindowsBlockTime );
		}
		else
		{
			Sleep( portTICK_PERIOD_MS );
		}

		configASSERT( xPortRunning );

		WaitForSingleObject( pvInterruptEventMutex, INFINITE );

		/* The timer has expired, generate the simulated tick event. */
		ulPendingInterrupts |= ( 1 << portINTERRUPT_TICK );

		/* The interrupt is now pending - notify the simulated interrupt
		handler thread. */
		if( ulCriticalNesting == 0 )
		{
			SetEvent( pvInterruptEvent );
		}

		/* Give back the mutex so the simulated interrupt handler unblocks
		and can	access the interrupt handler variables. */
		ReleaseMutex( pvInterruptEventMutex );
	}

	#ifdef __GNUC__
		/* Should never reach here - MingW complains if you leave this line out,
		MSVC complains if you put it in. */
		return 0;
	#endif
}
#endif

/*-----------------------------------------------------------*/
#ifdef __linux
static int prvEndProcess( int32_t dwCtrlType )
{
  //TODO
  return pdPASS;
}
#else // windows
static BOOL WINAPI prvEndProcess( DWORD dwCtrlType )
{
TIMECAPS xTimeCaps;

	( void ) dwCtrlType;

	if( timeGetDevCaps( &xTimeCaps, sizeof( xTimeCaps ) ) == MMSYSERR_NOERROR )
	{
		/* Match the call to timeBeginPeriod( xTimeCaps.wPeriodMin ) made when
		the process started with a timeEndPeriod() as the process exits. */
		timeEndPeriod( xTimeCaps.wPeriodMin );
	}

	return pdPASS;
}
#endif

/*-----------------------------------------------------------*/


#ifdef __linux__

static void* prvTaskThread(void* pvParams)
{
  xThreadState *pxThreadState = pvParams;
  if (!xPortRunning)
  {
    AbortIfFail(pthread_mutex_lock(&StartMutex), __FILE__, __LINE__);
    while (!xPortRunning)
      AbortIfFail(pthread_cond_wait(&StartCond, &StartMutex), __FILE__, __LINE__);
    AbortIfFail(pthread_mutex_unlock(&StartMutex), __FILE__, __LINE__);
  }
  pxThreadState->pxCode(pxThreadState->pvParameters);
  return NULL;
}

static pthread_t StartThread(int priority, void* (*func)(void*), void* pxThreadState)
{
  pthread_attr_t attr = {0};
  AbortIfFail(pthread_attr_init(&attr), __FILE__, __LINE__);
  AbortIfFail(pthread_attr_setschedpolicy(&attr, SCHED_FIFO), __FILE__, __LINE__);
  struct sched_param param;
  param.sched_priority = priority;
  AbortIfFail(pthread_attr_setschedparam(&attr, &param), __FILE__, __LINE__);
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(1, &cpuset);
  AbortIfFail(pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset), __FILE__, __LINE__);
  pthread_t pid = 0;
  AbortIfFail(pthread_create(&pid, &attr, func, pxThreadState), __FILE__, __LINE__);
  AbortIfFail(pthread_attr_destroy(&attr), __FILE__, __LINE__);
  return pid;
}

#endif


StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
xThreadState *pxThreadState = NULL;
int8_t *pcTopOfStack = ( int8_t * ) pxTopOfStack;

  Initialize();

/* In this simulated case a stack is not initialised, but instead a thread
	is created that will execute the task being created.  The thread handles
	the context switching itself.  The xThreadState object is placed onto
	the stack that was created for the task - so the stack buffer is still
	used, just not in the conventional way.  It will not be used for anything
	other than holding this structure. */
	pxThreadState = ( xThreadState * ) ( pcTopOfStack - sizeof( xThreadState ) );
  pxThreadState->pxCode = pxCode;
	pxThreadState->pvParameters = pvParameters;

	/* Create the thread itself. */
#ifdef __linux__
	pxThreadState->pvThread = StartThread(MinPriority, prvTaskThread, pxThreadState);
#else
	pxThreadState->pvThread = CreateThread( NULL, 0, ( LPTHREAD_START_ROUTINE ) pxCode, pvParameters, CREATE_SUSPENDED, NULL );
	configASSERT( pxThreadState->pvThread );
	SetThreadAffinityMask( pxThreadState->pvThread, 0x01 );
	SetThreadPriorityBoost( pxThreadState->pvThread, TRUE );
	SetThreadPriority( pxThreadState->pvThread, THREAD_PRIORITY_IDLE );
#endif
	return ( StackType_t * ) pxThreadState;
}
/*-----------------------------------------------------------*/

#ifdef __linux__
BaseType_t xPortStartScheduler( void )
{
  Initialize();

  pthread_t pvHandle = pthread_self();
  int32_t lSuccess = pdPASS;
  xThreadState *pxThreadState;

  /* Install the interrupt handlers used by the scheduler itself. */
  vPortSetInterruptHandler( portINTERRUPT_YIELD, prvProcessYieldInterrupt );
  vPortSetInterruptHandler( portINTERRUPT_TICK, prvProcessTickInterrupt );

  /* Set the priority of this thread such that it is above the priority of
  the threads that run tasks.  This higher priority is required to ensure
  simulated interrupts take priority over tasks. */
  struct sched_param priority;
  priority.sched_priority = MaxPriority;
  AbortIfFail(pthread_setschedparam(pthread_self(), SCHED_FIFO, &priority), __FILE__, __LINE__);

  /* Start the thread that simulates the timer peripheral to generate
  tick interrupts.  The priority is set below that of the simulated
  interrupt handler so the interrupt event mutex is used for the
  handshake / overrun protection. */

  pvHandle = StartThread((MinPriority + MaxPriority) / 2, prvSimulatedPeripheralTimer, NULL);
  /* Start the highest priority task by obtaining its associated thread
  state structure, in which is stored the thread handle. */
  ulCriticalNesting = portNO_CRITICAL_NESTING;

  // Revert priory of this thead to one higher than task thread. In this case thread with timer
  // will get time and will generate ticks but task thread
  AbortIfFail(pthread_setschedprio(pthread_self(),  MinPriority + 2), __FILE__, __LINE__);

  /* Handle all simulated interrupts - including yield requests and
  simulated ticks. */
  prvProcessSimulatedInterrupts();

  /* Would not expect to return from prvProcessSimulatedInterrupts(), so should
  not get here. */
  return 0;
}
#else // windows
BaseType_t xPortStartScheduler( void )
{
void *pvHandle;
int32_t lSuccess = pdPASS;
xThreadState *pxThreadState;

	/* Install the interrupt handlers used by the scheduler itself. */
	vPortSetInterruptHandler( portINTERRUPT_YIELD, prvProcessYieldInterrupt );
	vPortSetInterruptHandler( portINTERRUPT_TICK, prvProcessTickInterrupt );

	/* Create the events and mutexes that are used to synchronise all the
	threads. */
	pvInterruptEventMutex = CreateMutex( NULL, FALSE, NULL );
	pvInterruptEvent = CreateEvent( NULL, FALSE, FALSE, NULL );

	if( ( pvInterruptEventMutex == NULL ) || ( pvInterruptEvent == NULL ) )
	{
		lSuccess = pdFAIL;
	}

	/* Set the priority of this thread such that it is above the priority of
	the threads that run tasks.  This higher priority is required to ensure
	simulated interrupts take priority over tasks. */
	pvHandle = GetCurrentThread();
	if( pvHandle == NULL )
	{
		lSuccess = pdFAIL;
	}

	if( lSuccess == pdPASS )
	{
		if( SetThreadPriority( pvHandle, THREAD_PRIORITY_NORMAL ) == 0 )
		{
			lSuccess = pdFAIL;
		}
		SetThreadPriorityBoost( pvHandle, TRUE );
		SetThreadAffinityMask( pvHandle, 0x01 );
	}

	if( lSuccess == pdPASS )
	{
		/* Start the thread that simulates the timer peripheral to generate
		tick interrupts.  The priority is set below that of the simulated
		interrupt handler so the interrupt event mutex is used for the
		handshake / overrun protection. */
		pvHandle = CreateThread( NULL, 0, prvSimulatedPeripheralTimer, NULL, 0, NULL );
		if( pvHandle != NULL )
		{
			SetThreadPriority( pvHandle, THREAD_PRIORITY_BELOW_NORMAL );
			SetThreadPriorityBoost( pvHandle, TRUE );
			SetThreadAffinityMask( pvHandle, 0x01 );
		}

		/* Start the highest priority task by obtaining its associated thread
		state structure, in which is stored the thread handle. */
		pxThreadState = ( xThreadState * ) *( ( size_t * ) pxCurrentTCB );
		ulCriticalNesting = portNO_CRITICAL_NESTING;

		/* Bump up the priority of the thread that is going to run, in the
		hope that this will assist in getting the Windows thread scheduler to
		behave as an embedded engineer might expect. */
		ResumeThread( pxThreadState->pvThread );

		/* Handle all simulated interrupts - including yield requests and
		simulated ticks. */
		prvProcessSimulatedInterrupts();
	}

	/* Would not expect to return from prvProcessSimulatedInterrupts(), so should
	not get here. */
	return 0;
}
#endif

/*-----------------------------------------------------------*/

static uint32_t prvProcessYieldInterrupt( void )
{
	return pdTRUE;
}
/*-----------------------------------------------------------*/

static uint32_t prvProcessTickInterrupt( void )
{
uint32_t ulSwitchRequired;

	/* Process the tick itself. */
	configASSERT( xPortRunning );
	ulSwitchRequired = ( uint32_t ) xTaskIncrementTick();

	return ulSwitchRequired;
}

static void prvProcessSimulatedInterrupts( void )
{
uint32_t ulSwitchRequired, i;
xThreadState *pxThreadState;

#ifndef __linux__
void *pvObjectList[ 2 ];

	/* Going to block on the mutex that ensured exclusive access to the simulated
	interrupt objects, and the event that signals that a simulated interrupt
	should be processed. */
	pvObjectList[ 0 ] = pvInterruptEventMutex;
	pvObjectList[ 1 ] = pvInterruptEvent;
#endif
	/* Create a pending tick to ensure the first task is started as soon as
	this thread pends. */
	ulPendingInterrupts |= ( 1 << portINTERRUPT_TICK );

#ifdef __linux__
#else
	SetEvent( pvInterruptEvent );
#endif

	xPortRunning = pdTRUE;

	AbortIfFail(pthread_mutex_lock(&StartMutex), __FILE__, __LINE__);
	AbortIfFail(pthread_cond_broadcast(&StartCond), __FILE__, __LINE__);
	AbortIfFail(pthread_mutex_unlock(&StartMutex), __FILE__, __LINE__);

	for(;;)
	{
#ifdef __linux__
	  AbortIfFail(pthread_mutex_lock(pvInterruptMutex), __FILE__, __LINE__);
	  AbortIfFail(pthread_cond_wait(pvInterruptEvent, pvInterruptMutex), __FILE__, __LINE__);
	  AbortIfFail(pthread_mutex_lock(pvInterruptEventMutex), __FILE__, __LINE__);
#else
		WaitForMultipleObjects( sizeof( pvObjectList ) / sizeof( void * ), pvObjectList, TRUE, INFINITE );
#endif

		/* Used to indicate whether the simulated interrupt processing has
		necessitated a context switch to another task/thread. */
		ulSwitchRequired = pdFALSE;

		/* For each interrupt we are interested in processing, each of which is
		represented by a bit in the 32bit ulPendingInterrupts variable. */
		for( i = 0; i < portMAX_INTERRUPTS; i++ )
		{
			/* Is the simulated interrupt pending? */
			if( ulPendingInterrupts & ( 1UL << i ) )
			{
				/* Is a handler installed? */
				if( ulIsrHandler[ i ] != NULL )
				{
					/* Run the actual handler. */
					if( ulIsrHandler[ i ]() != pdFALSE )
					{
						ulSwitchRequired |= ( 1 << i );
					}
				}

				/* Clear the interrupt pending bit. */
				ulPendingInterrupts &= ~( 1UL << i );
			}
		}

		if( ulSwitchRequired != pdFALSE )
		{
			void *pvOldCurrentTCB;

			pvOldCurrentTCB = pxCurrentTCB;

			/* Select the next task to run. */
			vTaskSwitchContext();

			/* If the task selected to enter the running state is not the task
			that is already in the running state. */
			if( pvOldCurrentTCB != pxCurrentTCB )
			{
				/* Suspend the old thread. */
				pxThreadState = ( xThreadState *) *( ( size_t * ) pvOldCurrentTCB );
#ifdef __linux__
				// Down old thread priority.
				AbortIfFail(pthread_setschedprio(pxThreadState->pvThread, MinPriority), __FILE__, __LINE__);
#else
        SuspendThread( pxThreadState->pvThread );
#endif

				/* Obtain the state of the task now selected to enter the
				Running state. */
				pxThreadState = ( xThreadState * ) ( *( size_t *) pxCurrentTCB );
#ifdef __linux__
				// raise next thread priority. OS will select this thread to run next.
				AbortIfFail(pthread_setschedprio(pxThreadState->pvThread, MinPriority + 1), __FILE__, __LINE__);
#else
				ResumeThread( pxThreadState->pvThread );
#endif
			}
		}
#ifdef __linux__
		AbortIfFail(pthread_mutex_unlock(pvInterruptEventMutex), __FILE__, __LINE__);
		AbortIfFail(pthread_mutex_unlock(pvInterruptMutex), __FILE__, __LINE__);
#else
		ReleaseMutex( pvInterruptEventMutex );
#endif
	}
}

/*-----------------------------------------------------------*/
#ifdef __linux__
void vPortDeleteThread( void *pvTaskToDelete )
{

}
#else
void vPortDeleteThread( void *pvTaskToDelete )
{
xThreadState *pxThreadState;
uint32_t ulErrorCode;

	/* Remove compiler warnings if configASSERT() is not defined. */
	( void ) ulErrorCode;

	/* Find the handle of the thread being deleted. */
	pxThreadState = ( xThreadState * ) ( *( size_t *) pvTaskToDelete );

	/* Check that the thread is still valid, it might have been closed by
	vPortCloseRunningThread() - which will be the case if the task associated
	with the thread originally deleted itself rather than being deleted by a
	different task. */
	if( pxThreadState->pvThread != NULL )
	{
		WaitForSingleObject( pvInterruptEventMutex, INFINITE );

		ulErrorCode = TerminateThread( pxThreadState->pvThread, 0 );
		configASSERT( ulErrorCode );

		ulErrorCode = CloseHandle( pxThreadState->pvThread );
		configASSERT( ulErrorCode );

		ReleaseMutex( pvInterruptEventMutex );
	}
}
#endif

/*-----------------------------------------------------------*/
void vPortCloseRunningThread( void *pvTaskToDelete, volatile BaseType_t *pxPendYield )
{
xThreadState *pxThreadState;
#ifdef __linux__
  pthread_t pvThread = 0;
#else
void *pvThread;
#endif
uint32_t ulErrorCode;

	/* Remove compiler warnings if configASSERT() is not defined. */
	( void ) ulErrorCode;

	/* Find the handle of the thread being deleted. */
	pxThreadState = ( xThreadState * ) ( *( size_t *) pvTaskToDelete );
	pvThread = pxThreadState->pvThread;

	/* Raise the Windows priority of the thread to ensure the FreeRTOS scheduler
	does not run and swap it out before it is closed.  If that were to happen
	the thread would never run again and effectively be a thread handle and
	memory leak. */
#ifdef __linux__
	AbortIfFail(pthread_setschedprio(pvThread, 1), __FILE__, __LINE__);
#else
	SetThreadPriority( pvThread, THREAD_PRIORITY_ABOVE_NORMAL );
#endif

	/* This function will not return, therefore a yield is set as pending to
	ensure a context switch occurs away from this thread on the next tick. */
	*pxPendYield = pdTRUE;

	/* Mark the thread associated with this task as invalid so
	vPortDeleteThread() does not try to terminate it. */
#ifdef __linux__
  pxThreadState->pvThread = 0;
#else
	pxThreadState->pvThread = NULL;
#endif

	/* Close the thread. */
#ifndef __linux__
	ulErrorCode = CloseHandle( pvThread );
#endif
	configASSERT( ulErrorCode );

#ifdef __linux__
	pthread_exit(NULL);
#else
	ExitThread( 0 );
#endif
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
#ifdef __linux__
  exit(0);
#else // widows
	/* This function IS NOT TESTED! */
	TerminateProcess( GetCurrentProcess(), 0 );
#endif
}

/*-----------------------------------------------------------*/
void vPortGenerateSimulatedInterrupt( uint32_t ulInterruptNumber )
{
	configASSERT( xPortRunning );

	if( ( ulInterruptNumber < portMAX_INTERRUPTS ) && ( pvInterruptEventMutex != NULL ) )
	{
		/* Yield interrupts are processed even when critical nesting is non-zero. */
#ifdef __linux__
	  AbortIfFail(pthread_mutex_lock( pvInterruptEventMutex ), __FILE__, __LINE__);
#else
		WaitForSingleObject( pvInterruptEventMutex, INFINITE );
#endif

		ulPendingInterrupts |= ( 1 << ulInterruptNumber );

		/* The simulated interrupt is now held pending, but don't actually process it
		yet if this call is within a critical section.  It is possible for this to
		be in a critical section as calls to wait for mutexes are accumulative. */
		if( ulCriticalNesting == 0 )
		{
#ifdef __linux
		  AbortIfFail(pthread_mutex_lock( pvInterruptMutex ), __FILE__, __LINE__);
		  AbortIfFail(pthread_cond_signal( pvInterruptEvent ), __FILE__, __LINE__);
		  AbortIfFail(pthread_mutex_unlock( pvInterruptMutex ), __FILE__, __LINE__);
#else
			SetEvent( pvInterruptEvent );
#endif
		}

#ifdef __linux__
		AbortIfFail(pthread_mutex_unlock(pvInterruptEventMutex), __FILE__, __LINE__);
#else
		ReleaseMutex( pvInterruptEventMutex );
#endif
	}
}

/*-----------------------------------------------------------*/

void vPortSetInterruptHandler( uint32_t ulInterruptNumber, uint32_t (*pvHandler)( void ) )
{
	if( ulInterruptNumber < portMAX_INTERRUPTS )
	{
		if( pvInterruptEventMutex != NULL )
		{
#ifdef __linux__
		  AbortIfFail(pthread_mutex_lock(pvInterruptEventMutex), __FILE__, __LINE__);
#else
			WaitForSingleObject( pvInterruptEventMutex, INFINITE );
#endif

			ulIsrHandler[ ulInterruptNumber ] = pvHandler;

#ifdef __linux__
			AbortIfFail(pthread_mutex_unlock(pvInterruptEventMutex), __FILE__, __LINE__);
#else
			ReleaseMutex( pvInterruptEventMutex );
#endif
		}
		else
		{
			ulIsrHandler[ ulInterruptNumber ] = pvHandler;
		}
	}
}

/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	if( xPortRunning == pdTRUE )
	{
		/* The interrupt event mutex is held for the entire critical section,
		effectively disabling (simulated) interrupts. */
#ifdef __linux__
	  AbortIfFail(pthread_mutex_lock(pvInterruptEventMutex), __FILE__, __LINE__);
#else // widows
		WaitForSingleObject( pvInterruptEventMutex, INFINITE );
#endif
		ulCriticalNesting++;
	}
	else
	{
		ulCriticalNesting++;
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
int32_t lMutexNeedsReleasing;

	/* The interrupt event mutex should already be held by this thread as it was
	obtained on entry to the critical section. */

	lMutexNeedsReleasing = pdTRUE;

	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		if( ulCriticalNesting == ( portNO_CRITICAL_NESTING + 1 ) )
		{
			ulCriticalNesting--;

			/* Were any interrupts set to pending while interrupts were
			(simulated) disabled? */
			if( ulPendingInterrupts != 0UL )
			{
				configASSERT( xPortRunning );
#ifdef __linux__
				AbortIfFail(pthread_mutex_lock(pvInterruptMutex), __FILE__, __LINE__);
				AbortIfFail(pthread_cond_signal(pvInterruptEvent), __FILE__, __LINE__);
#else // widows
				SetEvent( pvInterruptEvent );
#endif
				/* Mutex will be released now, so does not require releasing
				on function exit. */
				lMutexNeedsReleasing = pdFALSE;
#ifdef __linux__
        AbortIfFail(pthread_mutex_unlock(pvInterruptMutex), __FILE__, __LINE__);
#else // widows
				ReleaseMutex( pvInterruptEventMutex );
#endif
			}
		}
		else
		{
			/* Tick interrupts will still not be processed as the critical
			nesting depth will not be zero. */
			ulCriticalNesting--;
		}
	}

	if( pvInterruptEventMutex )
	{
		if( lMutexNeedsReleasing == pdTRUE )
		{
			configASSERT( xPortRunning );
#ifdef __linux__
			AbortIfFail(pthread_mutex_unlock( pvInterruptEventMutex ), __FILE__, __LINE__);
#else // widows
			ReleaseMutex( pvInterruptEventMutex );
#endif
		}
	}
}
/*-----------------------------------------------------------*/

