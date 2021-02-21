/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */
// Alti: Altitude in meters above MSL ");
// Angle: Course in degrees from true north ");
// Date: GMT day, month, year ");
// Time: GMT hours, minutes, seconds, milliseconds ");
// Fix: Sata data");
// Sats: Number of satellites in use")

#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#define BUF_SIZE        255
#define TIMEOUT_S       0.5
#define BAUDRATE        115200
#define bus				2		 // which bus to use for gps uart


// function declarations
void on_pause_press();
void clean_exit();
int power_servo();
int setup();
void rover_loop();
void test_run();
int UART_start();
void wasd_drive();
void drive(double esc, double servo);

typedef enum p_mode_t{
        NONE,
        POWERON,
        POWEROFF
} p_mode_t;
// disable power for servos use mode = POWERON; to run on
p_mode_t mode = NONE;

//** Servo p thread **/
static int running;
static double ESC_turn; // global variable for normalized pulse ESC_turn to send
static double Servo_turn; // global variable for normalized pulse width to send

// background thread to send pulses at 50hz to servo
static void* __send_servo_pulses(__attribute__ ((unused)) void *params)
{
        while(running){
				rc_servo_send_pulse_normalized(1, ESC_turn);
                rc_servo_send_pulse_normalized(2, Servo_turn);
                rc_usleep(16666);
        }
        return 0;
}
//** end **//

char *convert(uint8_t *a)
{
  char* buffer2;
  int i;

  buffer2 = malloc(9);
  if (!buffer2)
    return NULL;

  buffer2[8] = 0;
  for (i = 0; i <= 7; i++)
    buffer2[7 - i] = (((*a) >> i) & (0x01)) + '0';

  puts(buffer2);
  return buffer2;
}

int main()
{
	
	if(setup() == -1) return -1;
	// ** pthread initialize ** //
	pthread_t send_servo_pulse_thread;
	//** Initialize PRU and make sure power rail is OFF
    if(rc_servo_init()) return -1;    
    rc_servo_power_rail_en(0);
	if(UART_start() == -1) return -1;
	//** Green for system running **//
	rc_led_set(RC_LED_GREEN, 1);
	rc_led_set(RC_LED_RED, 0);
	//** Start the power to the servo
	if(power_servo()==-1)
	{
		fprintf(stderr,"ERROR: failed to start power to servos\n");
		return -1;
	} 
	// ** Start pthread **/
	running = 1;
	ESC_turn = 0;	
	Servo_turn = -0.3;
	if(rc_pthread_create(&send_servo_pulse_thread, __send_servo_pulses, (void*) NULL, SCHED_OTHER,0)==-1){
			return -1;
	}	

	//*** MAIN LOOP START ****/
	rover_loop();
	//test_run();


	running = 0;
	rc_pthread_timed_join(send_servo_pulse_thread, NULL, 1.0); // wait for it to stop
	rc_set_state(EXITING);
	clean_exit();
	return 0;
}

void rover_loop()
{

	// double speed = 0.0;
	// double turn = 0.0;
	// double sp_step = 0.01;
	// double tu_step = 0.03;
	
	//UART
	char* send_str = "Speed\n";
	int bytes = strlen(send_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];	// aka unsigned char * array! 
	rc_uart_flush(bus);
	int ret; // return value
	int sats_stats = 0;
	bool use_gps = false;	//
	// 6407.3901N, 2155.6000W
	uint8_t fix[] = {6+48,4+48,48,7+48,46,3+48,9+48,48,1+48,78,44,32,
							2+48,1+48,5+48,5+48,46,6+48,48,48,48,87};

	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 1);
	
	if(use_gps)
	{
		printf("GPS check!\n");
		while(sats_stats < 4)
		{
			rc_uart_flush(bus);
			
			send_str = "Sats\n";
			// Write
			bytes = strlen(send_str);
			rc_uart_write(bus, (uint8_t*)send_str, bytes);
			// Read
			memset(buf,0,sizeof(buf));
			ret = rc_uart_read_line(bus, buf, sizeof(buf));
			if(ret<0) fprintf(stderr,"Error reading bus\n");
			else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
			else printf("%s\n",buf);
			sats_stats = buf[4]-'0';
			printf("Number of sats: %d \n", sats_stats);
			rc_usleep(2000000);
			if(sats_stats > 50) printf("Error in sats\n");
		}
	}
	// How to read gps to double
	// char s1[9];
	// char *ptr;
	// //double s2[9];
	// while(1)
	// {
	// 	getchar();
	// 	printf("Fix: %s\n",fix);
	// 	memcpy(s1,fix,9);
	// 	double hit = strtod(s1,&ptr);
	// 	printf("Hit: %f \n", hit);
	// 	printf("Rest is: %s \n", ptr);
	// }

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_uart_flush(bus);
		printf("Asking GPS for Time: ");
		send_str = "Time\n";
		// Write
		bytes = strlen(send_str);
		rc_uart_write(bus, (uint8_t*)send_str, bytes);
		// Read
		memset(buf,0,sizeof(buf));
		ret = rc_uart_read_line(bus, buf, sizeof(buf));
		if(ret<0) fprintf(stderr,"Error reading bus\n");
		else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
		else printf("%s\n",buf);
		//write
		rc_uart_flush(bus);
		printf("Asking GPS for Sats: ");
		send_str = "Sats\n";
		// Write
		bytes = strlen(send_str);
		rc_uart_write(bus, (uint8_t*)send_str, bytes);
		// Read
		memset(buf,0,sizeof(buf));
		ret = rc_uart_read_line(bus, buf, sizeof(buf));
		if(ret<0) fprintf(stderr,"Error reading bus\n");
		else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
		else printf("%s\n",buf);



		break;
		
	
	}
	return;
}

//** functions ***//


int setup()
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
	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();
	return 0;
}

int UART_start()
{
	char* test_str = "Time\n";
	int bytes = strlen(test_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];
	int ret; // return value

	// disable canonical (0), 1 stop bit (1), disable parity (0)
	if(rc_uart_init(bus, BAUDRATE, TIMEOUT_S, 0,1,0)){
			printf("Failed to rc_uart_init%d\n", bus);
			return -1;
	}
	// Flush
	rc_uart_flush(bus);
	printf("Sending test string: Time\n");
	rc_uart_write(bus, (uint8_t*)test_str, bytes);

	memset(buf,0,sizeof(buf));
	ret = rc_uart_read_line(bus, buf, sizeof(buf));
	if(ret<0){
		fprintf(stderr,"Error reading bus\n");
		return -1;
	}
	else if(ret==0) {
		printf("timeout reached, %d bytes read\n", ret); 
		return -1;
	}
	else printf("%s\n",buf);
	return 0;
}

int power_servo()
{
	// enable power for servos
	mode = POWERON;
	// if power has been requested, make sure battery is connected!
	if(mode == POWERON){
		// read adc to make sure battery is connected
		if(rc_adc_init()){
				fprintf(stderr,"ERROR: failed to run rc_adc_init()\n");
				return -1;
		}
		if(rc_adc_batt()<5.5){
				fprintf(stderr,"ERROR: battery disconnected or insufficiently charged to drive motors\n");
				return -1;
		}
		rc_adc_cleanup();
		if(rc_servo_power_rail_en(1)){
				fprintf(stderr,"failed to enable power rail\n");
				return -1;
		}
	}
	// turn on power
    printf("Turning On 6V Servo Power Rail\n");
	return 1;
}
void clean_exit()
{
	printf("Cleaning up and exiting program\n");
	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	// turn off power rail and cleanup
	rc_servo_power_rail_en(0);
	printf("Servo power OFF\n");
	rc_servo_cleanup();

	
	// close
	rc_uart_flush(bus);
	rc_uart_close(bus);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	
	printf("Clean exit\n");
}

void drive(double esc, double servo)
{
	/**
	*	35° CCW and 35°° CW
	*
	*
	*
	**/
	//double step = 0.01;
	double max = 1.0;
	double min = -1.0;
	
	double esc_zero = 0;
	double esc_min = -0.3;
	double esc_max = 0.3;
	

	double servo_zero = -0.30;
	double servo_min = -0.9;
	double servo_max = 0.3;

	if((servo<-0.01) && (servo>min))	// TURN RIGHT
	{
		//printf("RIGHT TURN \n\r");		
		if((servo+servo_zero)<servo_min)
		{
			Servo_turn = servo_min;	
		} 	
		else
		{
			Servo_turn = servo+servo_zero;
		}
	}
	else if((servo>0.01) && (servo<max))	// TURN LEFT
	{
		// printf("LEFT TURN \n\r");
		if((servo+servo_zero)>servo_max)
		{
			 Servo_turn = servo_max;
		}
		else
		{
			Servo_turn = servo+servo_zero;
		}	
	}
	else
	{
		printf("STOP TURN \n\r");
		Servo_turn = servo_zero;
	}


	if((esc<-0.01) && (esc>min))	// TURN RIGHT
	{
		//printf("RIGHT TURN \n\r");		
		if((esc+esc_zero)<esc_min)
		{
			ESC_turn = esc_min;	
		} 	
		else
		{
			ESC_turn = esc+esc_zero;
		}
	}
	else if((esc>0.01) && (esc<max))	// TURN LEFT
	{
		// printf("LEFT TURN \n\r");
		if((esc+esc_zero)>esc_max)
		{
			 ESC_turn = esc_max;
		}
		else
		{
			ESC_turn = esc+esc_zero;
		}	
	}
	else
	{
		printf("STOP RED \n\r");
		ESC_turn = esc_zero;
	}
	printf("Speed: %f\n\r",ESC_turn);
	printf("Turn: %f\n\r",Servo_turn);
	return;
}

void wasd_drive()
{
	
	//double turn;
	//double speed;
	double step = 0.05;
	double max = 1.0;
	double servo_of = -0.3;
	double esc_min = -0.60;
	double esc_max = 0.60;
	// uint64_t time_0 = rc_nanos_since_epoch();
	// printf("Hi \n\r");
	// if((rc_nanos_since_epoch()-time_0)>350000000)
	// {
	// 	printf("Hæ \n\r");
	// 	if(Servo_turn<0)
	// 	{
	// 		Servo_turn = Servo_turn	+step;
	// 	}
	// 	else if(Servo_turn>0)
	// 	{
	// 		Servo_turn = Servo_turn - step;
	// 	}
	// 	else Servo_turn = 0;
	// 	if(ESC_turn<0)
	// 	{
	// 		ESC_turn = ESC_turn	+step;
	// 	}
	// 	else if(ESC_turn>0)
	// 	{
	// 		ESC_turn = ESC_turn - step;
	// 	}
	// 	else ESC_turn = 0;
	// 	time_0 = rc_nanos_since_epoch();		
		
	// } 
	char keyput = getchar();	
	if(keyput == 'x') rc_set_state(EXITING);
	switch(keyput){
		case 'w':
			ESC_turn = ESC_turn + step;
			if(ESC_turn>esc_max) ESC_turn = esc_max;
			break;
		case 's':
			ESC_turn = ESC_turn - step;
			if(ESC_turn<esc_min) ESC_turn = esc_min;
			break;
		case 'a':
			Servo_turn = Servo_turn+step;
			if(Servo_turn>(max+servo_of)) Servo_turn = (max+servo_of);
			break;
		case 'd':
			Servo_turn = Servo_turn - step;
			if(Servo_turn<-max) Servo_turn = -max;
			break;
		case ' ':
			printf("STOP\n\r");
			ESC_turn = 0;
			break;
		default:
			printf("Use WASD to move\n\r");
			printf("f to stop program\n\r");
			break;		
	}

	printf("Speed: %f\n\r",ESC_turn);
	printf("Turn: %f\n\r",Servo_turn);
	
	return;
}

void test_run()
{
	//UART
	char* send_str = "Speed\n";
	int bytes = strlen(send_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];
	rc_uart_flush(bus);
	int ret; // return value
	// Input
	//double val;
	

	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 1);
	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_uart_flush(bus);
		printf("Asking GPS for Time: ");
		send_str = "Time\n";
		// Write
		bytes = strlen(send_str);
		rc_uart_write(bus, (uint8_t*)send_str, bytes);
		// Read
		memset(buf,0,sizeof(buf));
		ret = rc_uart_read_line(bus, buf, sizeof(buf));
		if(ret<0) fprintf(stderr,"Error reading bus\n");
		else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
		else printf("%s\n",buf);
		//write
		rc_uart_flush(bus);
		printf("Asking GPS for Sats: ");
		send_str = "Sats\n";
		// Write
		bytes = strlen(send_str);
		rc_uart_write(bus, (uint8_t*)send_str, bytes);
		// Read
		memset(buf,0,sizeof(buf));
		ret = rc_uart_read_line(bus, buf, sizeof(buf));
		if(ret<0) fprintf(stderr,"Error reading bus\n");
		else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
		else printf("%s\n",buf);


		double stop = 1.01;
		double speed = 0.0;
		double turn = 0.0;
		double sp_step = 0.01;
		double tu_step = 0.03;
		system ("/bin/stty raw -echo"); //system call to change to raw mode
		drive(speed,turn);
		printf("press anykey to start and hold");
		
		while(rc_get_state()!=EXITING)
		{
			//wasd_drive();
			char c = getchar();
			if(c == ' ') break;
			printf("Set turn: %f\n\r",turn);
			drive(speed,turn);
			//rc_usleep(500000);
			speed = speed + sp_step;
			turn = turn + tu_step;
			if(turn > stop) break;
		}
		speed = 0.0;
		turn = 0.0;
		while(rc_get_state()!=EXITING)
		{
			//wasd_drive();
			char c = getchar();
			if(c == ' ') break;
			printf("Set turn: %f\n\r",turn);
			drive(speed,turn);
			//rc_usleep(500000);
			speed = speed - sp_step;
			turn = turn - tu_step;
			if(turn < -stop) break;
		}
		system ("/bin/stty cooked echo");
		break;
		//
		// printf("Enter an ESC val: ");
		// scanf("%f", &val);
		// if(val>1) break;
		// ESC_turn = val;
		// printf("Enter an Servo val: ");
		// scanf("%f", &val);
		// if(val>1) break;
		// Servo_turn = val;		
	
	}
	return;
}