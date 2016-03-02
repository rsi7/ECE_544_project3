/* proj3_starter_app - ECE 544 Project #3 Application template
 *
 * Copyright: 2016 Portland State University
 *
 * Author: Rehan Iqbal
 * Date: February 28, 2016
 *
 *  Description:
 *  ------------
 *  This program implements ECE 544 Project #3.  It is a Xilinx-based design that is meant to be a learning tool
 *  for incorporating several of the key functions provided by Xilkernel.   The application sets up 4 independent
 *  threads.  Thread 1 periodically reads the switches on the FPGA development board and sends an update to the LED thread.
 *  Thread 2 responds to pushbutton press interrupts and sends an update to the LED thread whenever the state of
 *  any of the pushbuttons on the FPGA development board change.   Thread 3 writes the new pushbutton and switch state to the
 *  LEDs.   The three threads communicate via a message queue.  Thread 1 communicates to its interrupt handler through
 *  the use of a semaphore.  The application also makes use of the Xilinx watchdog timer.
 *  The 4th thread (the master thread) sets up the MicroBlaze, initializes the peripherals, creates the other threads and
 *  the semaphore and then enters a main loop where it wakes up periodically to reset the WDT.  Status messages are sent
 *  via STDOUT through the USB serial port which can be connected to a PC running a terminal emulator such as putty.
 *
 */

/****************************************************************************/
/***************************** Include Files ********************************/
/****************************************************************************/

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
#include "xintc.h"

/****************************************************************************/
/************************** Constant Definitions ****************************/
/****************************************************************************/

// Device ID

#define BTN_GPIO_DEVICEID       XPAR_BTNS_5BIT_DEVICE_ID
#define SW_GPIO_DEVICEID        XPAR_SW_16BIT_DEVICE_ID
#define LED_GPIO_DEVICEID       XPAR_LEDS_16BIT_DEVICE_ID
#define INTC_DEVICEID           XPAR_INTC_0_DEVICE_ID
#define WDT_DEVICEID            XPAR_WDTTB_0_DEVICE_ID
#define TMRCTR0_DEVICEID        XPAR_TMRCTR_0_DEVICE_ID
#define INTC_DEVICEID           XPAR_INTC_0_DEVICE_ID

// Interrupt numbers

#define TMRCTR0_INTR_NUM        XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR
#define BTN_GPIO_INTR_NUM       XPAR_MICROBLAZE_0_AXI_INTC_BTNS_5BIT_IP2INTC_IRPT_INTR
#define WDT_INTR_NUM            XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

 // Miscellaneous

 #define GPIO_CHANNEL_1         1
 #define MSK_CLEAR_INTR_CH1     0x00000001
 #define MSK_ENABLE_INTR_CH1    0x00000001
 #define MSK_LED_16BIT_OUTPUT   0x0000
 #define MSK_SW_16BIT_INPUT     0xFFFF
 #define MSK_PBTNS_5BIT_INPUT   0x1F

/****************************************************************************/
/***************** Macros (Inline Functions) Definitions ********************/
/****************************************************************************/

#define MIN(a, b)               ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)               ( ((a) >= (b)) ? (a) : (b) )

/****************************************************************************/
/************************** Variable Definitions ****************************/
/****************************************************************************/

XGpio       BTNInst;
XGpio       SWInst;
XGpio       LEDInst;
XTmrCtr     TMRCTR0Inst;
XWdtTb      WDTInst;
XIntc       IntrptCtlrInst;

/****************************************************************************/
/*************************** Typdefs & Structures ***************************/
/****************************************************************************/

// LED message structure

typedef struct {
    int msg_src;
    int msg_value;
} t_LED_message, *t_LED_messageptr;

const int led_msg_key = 1;      // message key for LED message queue
struct msqid_ds led_msgstats;   // statistics from message queue

// Synchronization variables

sem_t btn_press_sema;           // semaphore between clock tick ISR and the clock main thread
volatile u32 btn_state;         // button state - shared between button handler and button thread


/****************************************************************************/
/************************** Function Prototypes *****************************/
/****************************************************************************/

void* master_thread(void *arg);
void* button_thread(void *arg);
void* switches_thread(void *arg);
void* leds_thread(void *arg);
void  button_handler(void);
void  wdt_handler(void);
XStatus init_peripherals(void);

/****************************************************************************/
/***************************** Global Variables *****************************/
/****************************************************************************/

bool                sys_run;                // flag for determining whether system is running
bool                force_crash;            // flag for intentionally crashing the system
unsigned int        button_state;           // global variable to hold button values

/****************************************************************************/
/************************** MAIN PROGRAM ************************************/
/****************************************************************************/

int main() {

    XStatus sts;

    // initialize the platform and the peripherals

    init_platform();
    sts = init_peripherals();

    if (sts != XST_SUCCESS) {

        xil_printf("FATAL ERROR: Could not initialize the peripherals\n\r");
        xil_printf("Please power cycle or reset the system\n\r");
        return -1;
    }

    else {
        xil_printf("\nInitialization of the peripherals was a success!\n\n");
    }

    // check if WDT expired and caused the reset - if so, don't start

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

/****************************************************************************/
/************************** MASTER THREAD ***********************************/
/****************************************************************************/

void* master_thread(void *arg) {

    pthread_t button;
    pthread_t switches;
    pthread_t leds;

    pthread_attr_t attr;
    struct sched_param spar;

    int ret;
    unsigned int ticks;

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

    // set the priority of all but the master thread to 1
    // master thread runs at priority 0 b/c it tickles the WDT

    pthread_attr_init(&attr);
    spar.sched_priority = 1;
    pthread_attr_setschedparam(&attr, &spar);

    // create the button thread

    ret = pthread_create (&button, &attr, (void*) button_thread, NULL);

    if (ret != 0) {

        xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "button thread");
        xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
        return (void*) -3;
    }

    else {
        xil_printf("MASTER: Button thread created\r\n");
    }

    // create the switches thread

    ret = pthread_create (&switches, &attr, (void*) switches_thread, NULL);

    if (ret != 0) {

        xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
        xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
        return (void*) -3;
    }

    else {
        xil_printf("MASTER: Switches thread created\r\n");
    }

    // create the LEDs thread

    ret = pthread_create (&leds, &attr, (void*) leds_thread, NULL);

    if (ret != 0) {

        xil_printf("ERROR (%d) IN MASTER THREAD: could not launch %s\r\n", ret, "switches thread");
        xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
        return (void*) -3;
    }

    else {
        xil_printf("MASTER: LEDs thread created\r\n");
    }

    // initialize the button press semaphore

    ret = sem_init (&btn_press_sema, 0, 0);

    if (ret != 0) {
        xil_printf("ERROR (%d) IN MASTER THREAD: could not initialize %s\r\n", errno, "button press semaphore");
        xil_printf("FATAL ERROR: Master Thread Terminating\r\n");
        return (void*) -3;
    }

    else {
        xil_printf ("MASTER: Button press semaphore has been initialized\n\r");
    }

    // register the watchdog interrupt handler

    ret = register_int_handler(WDT_INTR_NUM, (void*) wdt_handler, NULL);

    if (ret != XST_SUCCESS) {
        return (void*) -4;
    }

    else {
        xil_printf("MASTER: WDT interrupt handler created successfully\r\n");
    }

    // register the button interrupt handler

    ret = register_int_handler(BTN_GPIO_INTR_NUM, (void*) button_handler, NULL);

    if (ret != XST_SUCCESS) {
        return (void*) -4;
    }
    else {
        xil_printf("MASTER: Button interrupt handler created successfully\r\n");
    }

    // enable interrupts and start the WDT...we're off to the races

    enable_interrupt(BTN_GPIO_INTR_NUM);
    enable_interrupt(WDT_INTR_NUM);

    xil_printf("MASTER: Interrupts have been enabled\r\n");

    XWdtTb_Start(&WDTInst);
    xil_printf("MASTER: Watchdog timer has been started\r\n");

    // master thread main loop

    while(1) {

        sys_run = true;
        ticks = xget_clock_ticks();
        xil_printf("MASTER: %d ticks have elapsed\r\n", ticks);
        sleep(1000);
    }

    return NULL;
}

/****************************************************************************/
/************************** BUTTON THREAD ***********************************/
/****************************************************************************/

void* button_thread(void *arg) {

    while (1) {

        yield();
    }

    return NULL;
}

/****************************************************************************/
/************************* SWITCHES THREAD **********************************/
/****************************************************************************/

void* switches_thread(void *arg) {

    unsigned int sw = 0x00;

    while (1) {

        sw = XGpio_DiscreteRead(&SWInst, GPIO_CHANNEL_1);
        yield();
    }

    return NULL;
}

/****************************************************************************/
/*************************** LEDS THREAD ************************************/
/****************************************************************************/

void* leds_thread(void *arg) {

    unsigned int leds = 0x00;

    while (1) {

        XGpio_DiscreteWrite(&LEDInst, GPIO_CHANNEL_1, button_state); 
        yield();
    }

    return NULL;

}

/****************************************************************************/
/************************* INIT PERIPHERALS *********************************/
/****************************************************************************/

XStatus init_peripherals(void) {

    int status;             // status from Xilinx Lib calls

    // initialize the button GPIO instance

    status = XGpio_Initialize(&BTNInst, BTN_GPIO_DEVICEID);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Failed to initialize buttons!\r\n");
        return XST_FAILURE;
    }

    XGpio_InterruptGlobalEnable(&BTNInst);
    XGpio_InterruptEnable(&BTNInst, MSK_ENABLE_INTR_CH1);
    XGpio_SetDataDirection(&BTNInst, GPIO_CHANNEL_1, MSK_PBTNS_5BIT_INPUT);

    // initialize the switches GPIO instance

    status = XGpio_Initialize(&SWInst, SW_GPIO_DEVICEID);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Failed to initialize switches!\r\n");
        return XST_FAILURE;
    }

    XGpio_SetDataDirection(&SWInst, GPIO_CHANNEL_1, MSK_SW_16BIT_INPUT);

    // initialize the LEDs GPIO instance

    status = XGpio_Initialize(&LEDInst, LED_GPIO_DEVICEID);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Failed to initialize LEDs!\r\n");
        return XST_FAILURE;
    }

    XGpio_SetDataDirection(&LEDInst, GPIO_CHANNEL_1, MSK_LED_16BIT_OUTPUT);

    // initialize the watchdog timer and timebase driver 
    // so that it is ready to use

    status = XWdtTb_Initialize(&WDTInst,WDT_DEVICEID);

    if (status != XST_SUCCESS) {
        xil_printf("ERROR: Failed to initialize Watchdog!\r\n");
        return XST_FAILURE;
    }

    // successfully initialized... time to return

    return XST_SUCCESS;
}

/****************************************************************************/
/************************** BUTTON HANDLER **********************************/
/****************************************************************************/

void button_handler(void) {

    button_state = XGpio_DiscreteRead(&BTNInst, GPIO_CHANNEL_1);
    xil_printf("BTN HANDLER: button_state = %d\r\n", button_state);
    XGpio_InterruptClear(&BTNInst, MSK_CLEAR_INTR_CH1);
    acknowledge_interrupt(BTN_GPIO_INTR_NUM);
}

/****************************************************************************/
/*************************** WDT HANDLER ************************************/
/****************************************************************************/

void wdt_handler(void) {

    if (force_crash) {
         xil_printf("WATCHDOG: Forced crash on next timeout....\r\n");
    }

    else if (sys_run) {
        xil_printf("WATCHDOG: Resetting the timer....\r\n");
        XWdtTb_RestartWdt(&WDTInst);  
    }

    else {
        xil_printf("WATCHDOG: System crash on next timeout...\r\n");
    }

    sys_run = false;
    acknowledge_interrupt(WDT_INTR_NUM);
}
