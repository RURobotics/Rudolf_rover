/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <stdlib.h> // for atoi
#include <string.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#define BUF_SIZE        255
#define TIMEOUT_S       0.5
#define BAUDRATE        115200

// function declarations
void on_pause_press();
void on_pause_release();


/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
			fprintf(stderr,"ERROR: failed to init buttons\n");
			return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);
	rc_button_get_state(RC_BTN_PIN_MODE);
	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	//*** MAIN START ****/
	char* test_str = "Speed\n";
	int bytes = strlen(test_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];
	int ret; // return value
	int bus = 2; // which bus to use
	//char str[100];

	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(bus, BAUDRATE, TIMEOUT_S, 0,1,0)){
			printf("Failed to rc_uart_init%d\n", bus);
			return -1;
	}
	// Flush
	rc_uart_flush(bus);
	rc_uart_write(bus, (uint8_t*)test_str, bytes);


	printf("\nPress and release pause button to turn green LED on and off\n");
	printf("hold pause button down for 2 seconds to exit\n");

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
			rc_uart_flush(bus);
		}
		else{
			
			// printf( "Enter a value :");
   			// gets(str);
			// bytes = strlen(str);
			
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
			// Send
			if(rc_button_get_state(RC_BTN_PIN_MODE))
			{
				test_str = "Speed\n";
				bytes = strlen(test_str); // get number of bytes in test string
				//rc_uart_flush(bus);
				rc_uart_write(bus, (uint8_t*)test_str, bytes);
				// Read
			
				memset(buf,0,sizeof(buf));
				ret = rc_uart_read_line(bus, buf, sizeof(buf));
				if(ret<0) fprintf(stderr,"Error reading bus\n");
				else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
				else printf("%s\n",buf);

				test_str = "Fix\n";
				bytes = strlen(test_str); // get number of bytes in test string
				//rc_uart_flush(bus);
				rc_uart_write(bus, (uint8_t*)test_str, bytes);
				// Read
			
				memset(buf,0,sizeof(buf));
				ret = rc_uart_read_line(bus, buf, sizeof(buf));
				if(ret<0) fprintf(stderr,"Error reading bus\n");
				else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
				else printf("%s\n",buf);
				// always sleep at some point
				rc_usleep(100000);
			}
			

			
			//else printf("Received %d bytes: %s \n", ret, buf);
		}
		
	}

	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	// close
	rc_uart_flush(bus);
	rc_uart_close(bus);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
