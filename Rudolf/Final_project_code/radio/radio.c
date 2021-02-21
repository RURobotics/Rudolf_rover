#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#define BUF_SIZE_RADIO        10
#define TIMEOUT_S_RADIO       0.5
#define BAUDRATE_RADIO        57600
#define bus_RADIO				1		 // which bus to use for gps uart
#define UART_CHAR		'x'		// The first char of the data
#define MIN_BYTES		10		// The min bytes of the data

void radio_send(char* send_data);

int main()
{

    // make sure another instance isn't P_RUNNING
	// if return value is -3 then a background process is P_RUNNING with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}
	// make PID file to indicate your project is P_RUNNING
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	 // prepare to run
    rc_set_state(RUNNING);

    // char* send_str = "Hello from the other side!\n";
	// int bytes = strlen(send_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE_RADIO];
	int ret; // return value
	int turn = 0;
	int speed = 0;	
	int pakki = 0;
	int byt = 0;
	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(bus_RADIO, BAUDRATE_RADIO, TIMEOUT_S_RADIO, 0,1,0)){
			printf("Failed to rc_uart_init%d\n", bus_RADIO);
			return -1;
	}
	rc_usleep(10000);
	// Flush
	rc_uart_flush(bus_RADIO);	

	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
    // or holding down the pause button for more than the quit timeout period

	while(rc_get_state()!=EXITING)
	{	
		byt = rc_uart_bytes_available(bus_RADIO);
		if(byt > 0)	printf("BYTES: %d\n", byt);
		if(byt >= MIN_BYTES)
		{
			pakki++;
			// READ
			printf("Pakki nr %d: ", pakki);
			memset(buf,0,sizeof(buf));
			ret = rc_uart_read_line(bus_RADIO, buf, sizeof(buf));
			if(ret<0) fprintf(stderr,"Error reading bus\n");
			else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
			else printf("Received %d bytes: %s \n", ret, buf);
			if(buf[0]=='x')
			{
				//printf("You have mail!\n");
				turn = (int)buf[1];
				if(turn>0)
				{
					printf("Trun: %d\n",turn);
				}
				speed = (int)buf[2];
				if(turn>0)
				{
					printf("Speed: %d\n",speed);
				}
			}
		}else rc_uart_flush(bus_RADIO);
		
	}
   
    rc_uart_flush(bus_RADIO);
    rc_uart_close(bus_RADIO);
    rc_remove_pid_file();	// remove PId file LAST
}

void radio_send(char* send_data)
{
	int bytes = strlen(send_data); // get number of bytes in test string
	rc_uart_flush(bus_RADIO);
	printf("Sending: %s\n",send_data);
	rc_uart_write(bus_RADIO, (uint8_t*)send_data, bytes);
	return;
}