/* proj3_starter_app - ECE 544 Project #3 Application template
 *
 * Copyright (c) 2012, 2013, 2014, 2015 Portland State University by Roy Kravitz.  All rights reserved.
 *
 * Author:		<your name>
 * Date:		<the date>
 * Revision:	2.1
 *
 * Revision History:
 * -----------------
 * 	21-Feb-2012		RK		Created this program from the xilkernel sample application produced by SDK
 * 	16-May-2013		RK		Updated the program for Nexys3 board and revised homework 2 assignment
 *	12-May-2014		RK		Minor changes to ease support for both Nexys 3 and Nexys 4.  No functional changes
 *  xx-xxx-xxxx		xx		<your comment>
 *
 * 	Description:
 * 	------------
 * 	This program implements ECE 544 Project #3.  It is a Xilinx-based design that is meant to be a learning tool
 * 	for incorporating several of the key functions provided by Xilkernel.   The application sets up 4 independent
 * 	threads.  Thread 1 periodically reads the switches on the FPGA development board and sends an update to the LED thread.
 * 	Thread 2 responds to pushbutton press interrupts and sends an update to the LED thread whenever the state of
 * 	any of the pushbuttons on the FPGA development board change.   Thread 3 writes the new pushbutton and switch state to the
 * 	LEDs.   The three threads communicate via a message queue.  Thread 1 communicates to its interrupt handler through
 * 	the use of a semaphore.  The application also makes use of the Xilinx watchdog timer.
 * 	The 4th thread (the master thread) sets up the MicroBlaze, initializes the peripherals, creates the other threads and
 * 	the semaphore and then enters a main loop where it wakes up periodically to reset the WDT.  Status messages are sent
 * 	via STDOUT through the USB serial port which can be connected to a PC running a terminal emulator such as putty.
 *
 * <your additional comments>
 *
 */

// Includes
#include "xmk.h"
#include "os_config.h"
#include "config/config_param.h"
#include "sys/ksched.h"
#include "sys/init.h"
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <semaphore.h>
#include <sys/intr.h>
#include <sys/timer.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>

#include "xparameters.h"
#include "platform_config.h"
#include "platform.h"
#include "stdbool.h"
#include "xgpio.h"
#include "xwdttb.h"
#include "xtmrctr.h"
#include "xstatus.h"


// Declarations
// MOST OF THESE MAY NEED TO BE MODIFIED FOR YOUR SYSTEM'S xparameters.h
#define BTN_GPIO_DEVICEID		XPAR_BTNS_5BIT_DEVICE_ID
#define SW_GPIO_DEVICEID		XPAR_SW_16BIT_DEVICE_ID
#define LED_GPIO_DEVICEID		XPAR_LEDS_16BIT_DEVICE_ID
#define INTC_DEVICEID			XPAR_INTC_0_DEVICE_ID
#define WDT_DEVICEID			XPAR_WDTTB_0_DEVICE_ID
#define TMRCTR0_DEVICEID		XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR1_DEVICEID		XPAR_TMRCTR_1_DEVICE_ID

#define TMRCTR0_INTR_NUM		XPAR_INTC_0_TMRCTR_0_VEC_ID
#define TMRCTR1_INTR_NUM		XPAR_INTC_0_TMRCTR_1_VEC_ID
#define BTN_GPIO_INTR_NUM		XPAR_MICROBLAZE_0_AXI_INTC_BTNS_5BIT_IP2INTC_IRPT_INTR
#define WDT_INTR_NUM			XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR
// macro functions
#define MIN(a, b)  				( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  				( ((a) >= (b)) ? (a) : (b) )

// Peripheral instances
XGpio BTNInst, SWInst, LEDInst;
XTmrCtr TMRCTR1Inst;
XWdtTb WDTInst;

// Message passing variables
typedef struct					// LED message definition
{
	int msg_src;
	int msg_value;
} t_LED_message, *t_LED_messageptr;

const int led_msg_key = 1;		// message key for LED message queue
struct msqid_ds	led_msgstats;	// statistics from message queue

// Synchronization variables
sem_t btn_press_sema;			// semaphore between clock tick ISR and the clock main thread
volatile u32 btn_state;			// button state - shared between button handler and button thread

// Function declarations
void* master_thread(void *arg);
void* button_thread(void *arg);
void* switches_thread(void *arg);
void* leds_thread(void *arg);
void  button_handler(void);
void  wdt_handler(void);
XStatus init_peripherals(void);


int main()
{
    XStatus sts;

    // initialize the platform and the peripherals
    init_platform();
    sts = init_peripherals();
    if (sts != XST_SUCCESS)
    {
    	xil_printf("FATAL ERROR: Could not initialize the peripherals\n\r");
		xil_printf("Please power cycle or reset the system\n\r");
		return -1;
    }
    
	// check if WDT expired and caused the reset - if so, don't start
	if (XWdtTb_IsWdtExpired(&WDTInst))
	{
	 	// it's true, the WDT expired.
	 	//***** INSERT YOUR WDT RECOVERY CODE HERE ******//
	}

    // Initialize xilkernel
    xilkernel_init();

    // Create the master thread
    xmk_add_static_thread(master_thread, 0);
    
    // Start the kernel
    xilkernel_start();
    
    // Should never be reached
    cleanup_platform();
    
    return 0;
}


// The master thread
void* master_thread(void *arg)
{
    pthread_t button;
    pthread_t switches;
    pthread_t leds;

    pthread_attr_t attr;
    struct sched_param spar;

    int ret;

    xil_printf("----------------------------------------------------------------------------\r\n");
    xil_printf("ECE 544 Project 3 Starter Application \r\n");
    xil_printf("----------------------------------------------------------------------------\r\n");
    xil_printf("This Xilkernel based application reads the buttons and switches on the FPGA \r\n"
               "development board and displays them on the LEDs.  Even though the application is\r\n"
               "simple it uses several of the synchronization and interprocess communication\r\n"
               "capabilities offered in the Xilkernel\r\n\r\n"
               "To demonstrate, press any of the buttons and/or flip switches on the board.\r\n"
               "The current state of the buttons and switches should be displayed on the LEDs\r\n");
    xil_printf("----------------------------------------------------------------------------\r\n\r\n\r\n");;

    xil_printf("MASTER: Master Thread Starting\r\n");

    // set the priority of all but the master thread to 1.  The master thread runs at priority 0
    // because it is responsible for tickling the WDT.
    pthread_attr_init (&attr);              
    spar.sched_priority = 1;
    pthread_attr_setschedparam(&attr, &spar);

    // create the threads
    ret = pthread_create (&button, &attr, (void*) button_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "button thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    ret = pthread_create (&switches, &attr, (void*) switches_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
    ret = pthread_create (&leds, &attr, (void*) leds_thread, NULL);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }

    // initialize the button press semaphore
	ret = sem_init (&btn_press_sema, 0, 0);
    if (ret != 0)
    {
    	xil_printf("ERROR (%d) IN MASTER THREAD: could not initialize %s\r\n", errno, "button press semaphore");
    	xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
    	return (void*) -3;
    }
	else
	{
		xil_printf ("MASTER: Button press semaphore has been initialized\n\r");
	}
	
	// Register the interrupt handlers
    ret = register_int_handler(WDT_INTR_NUM, (void*) wdt_handler, NULL);
    if (ret != XST_SUCCESS)
    {
    	return (void*) -4;
    }
    ret = register_int_handler(BTN_GPIO_INTR_NUM, (void*) button_handler, NULL);
    if (ret != XST_SUCCESS)
    {
    	return (void*) -4;
    }

    // Enable interrupts and start the WDT...we're off to the races
    enable_interrupt(BTN_GPIO_INTR_NUM);
	enable_interrupt(WDT_INTR_NUM);
	xil_printf("MASTER: Interrupts have been enabled\r\n");
	
	XWdtTb_Start(&WDTInst);
	xil_printf("MASTER: Watchdog timer has been started\r\n");

	// master thread main loop
	while(1)
	{
		//***** INSERT YOUR MASTER THREAD CODE HERE ******//
	}
	
	return NULL;
}

// The button thread
void* button_thread(void *arg)
{
	//***** INSERT YOUR BUTTON THREAD CODE HERE ******//

	return NULL;
}

// The switches thread
void* switches_thread(void *arg)
{
	//***** INSERT YOUR SWITCHES THREAD CODE HERE ******//
	
	return NULL;
}

// The leds thread
void* leds_thread(void *arg)
{
	//***** INSERT YOUR LEDS THREAD CODE HERE ******//
	
	return NULL;
}


// init_peripherals() - Initializes the peripherals
XStatus init_peripherals(void)
{

	//***** INSERT YOUR PERIPHERAL INITIALIZATION CODE HERE ******//

	return XST_SUCCESS;
}

// button press interrupt handler
void button_handler(void)
{
	//***** INSERT YOUR BUTTON PRESS INTERRUPT HANDLER CODE HERE *****//

	acknowledge_interrupt(BTN_GPIO_INTR_NUM);
}

// WDT interrupt handler
void wdt_handler(void)
{
	//***** INSERT YOUR WATCHDOG TIMER INTERRUPT HANDLER CODE HERE *****//

	acknowledge_interrupt(WDT_INTR_NUM);
}

