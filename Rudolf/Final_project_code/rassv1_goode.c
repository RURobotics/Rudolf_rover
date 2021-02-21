/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */
// Alti: Altitude in meters above MSL ");
// Angle: Course in degrees from true north ");
// Date: GMT day, month, year ");
// Time: GMT hours, MINutes, seconds, milliseconds ");
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
#define I2C_BUS 		2

// function declarations
int setup();
int UART_start();
int power_servo();
int mpu_start();
void on_pause_press();
void clean_exit();
void rover_loop();
void test_run();
void wasd_drive();
void drive(double esc, double servo);
double get_head();
bool sensor(double summa_0, double summa_1, int counter);
double get_gps(double *lon, double *lat);
double ToRad(const  double degree);
double ToDeg(double radian);
double Heading( double nLat1, double nLon1, double nLat2, double nLon2);
double Distance( double nLat1, double nLon1, double nLat2, double nLon2 );
double degToTurn(double degrees);

typedef enum p_mode_t{
        NONE,
        POWERON,
        POWEROFF
} p_mode_t;
// disable power for servos use mode = POWERON; to run on
p_mode_t mode = NONE;

// static data for Tait Bryan
static rc_mpu_data_t data;

//** GLOBAL RUN SETTINGS **//
bool USE_GPS = true;	//
bool USE_SERVO = true;	// ESC AND SERVO
bool USE_MPU = true;

//** STATIC GLOBAL variable***
// Driveing variables
const  double MAX = 1.0;
const  double MIN = -1.0;
// ESC
const  double ESC_ZERO = 0;
const  double ESC_MIN = -0.3;
const  double ESC_MAX = 0.3;
// SERVO
const  double SERVO_ZERO = -0.30;
const  double SERVO_MIN = -0.9;
const  double SERVO_MAX = 0.3;
// SENSOR
const  double MAX_SENSOR_VOLT = 0.4;
// TIMERS ms*(ms/ns)
const  uint64_t SENSOR_TIMER =  10*1000000;
const  uint64_t GPS_TIMER =  3000*1000000;
const  uint64_t DRIVE_TIMER = 100*10000000;
// Distance
const double PI = 3.14159265359;




//** Servo p thread **/
static int P_RUNNING;
static double ESC_TURN; // global variable for normalized pulse ESC_TURN to send
static double SERVO_TURN; // global variable for normalized pulse width to send
// background thread to send pulses at 60hz to servo
static void* __send_servo_pulses(__attribute__ ((unused)) void *params)
{
        while(P_RUNNING){
				rc_servo_send_pulse_normalized(1, ESC_TURN);
                rc_servo_send_pulse_normalized(2, SERVO_TURN);
                rc_usleep(16666);
        }
        return 0;
}
//** end **//

// char *convert(uint8_t *a)
// {
//   char* buffer2;
//   int i;

//   buffer2 = malloc(9);
//   if (!buffer2)
//     return NULL;

//   buffer2[8] = 0;
//   for (i = 0; i <= 7; i++)
//     buffer2[7 - i] = (((*a) >> i) & (0x01)) + '0';

//   puts(buffer2);
//   return buffer2;
// }

int main()
{
	
	if(setup() == -1) return -1;
	// ** pthread initialize ** //
	pthread_t send_servo_pulse_thread;
	if(USE_SERVO){		
		//** Initialize PRU and make sure power rail is OFF
		if(rc_servo_init()) return -1;    
    	rc_servo_power_rail_en(0);
		//** Start the power to the servo
		if(power_servo()==-1)
		{
			fprintf(stderr,"ERROR: failed to start power to servos\n");
			return -1;
		}
		// ** Start pthread **/
		P_RUNNING = 1;
		ESC_TURN = ESC_ZERO;	
		SERVO_TURN = SERVO_ZERO;
		if(rc_pthread_create(&send_servo_pulse_thread, __send_servo_pulses, (void*) NULL, SCHED_OTHER,0)==-1){
				return -1;
		}	 
	}
    if(USE_GPS){
		if(UART_start() == -1) return -1;
	}
	if(USE_MPU){
		if(mpu_start() == -1) return -1;
	}
	//** Green for system P_RUNNING **//
	rc_led_set(RC_LED_GREEN, 1);
	rc_led_set(RC_LED_RED, 0);
	
	

	//*** MAIN LOOP START ****/
	
	rover_loop();
	//test_run();
	// double head = 0.0;
	// int k = 0;
	// for(k=0;k<10;k++){
	// 	rc_usleep(1000000);
	// 	head = get_head();
	// 	printf(" Brayan is : %.2f\n",head);
		
	// }

	// EXITING
	rc_set_state(EXITING);
	if(USE_SERVO){
		P_RUNNING = 0;
		rc_pthread_timed_join(send_servo_pulse_thread, NULL, 1.0); // wait for it to stop

	}	
	clean_exit();
	return 0;
}


//*** FUN

void rover_loop()
{
	// Sensor
	double summa_0 = 0;
	double summa_1 = 0;
	int sensor_count = 0;
	uint64_t  sensor_time = rc_nanos_since_epoch();
	// GPS
	double lon = 0.0;
	double lat = 0.0;
	//6407.3901N, 2155.600
	double go_lat = 64.073901;
	double go_lon = -21.55600;
	uint64_t  gps_time = rc_nanos_since_epoch();
	// GPS Heading
	double gps_heading = 0.0;
	double gps_distance = 1000.0;
	// ROVER
	double rover_heading = 0.0;
	double drive_time = 0.0;
	double speed = 0.17;
	double turn = 0.0;
	//UART
	char* send_str = "Time\n";
	int bytes = strlen(send_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];	// aka unsigned char * array! 
	rc_uart_flush(bus);
	int ret; // return value
	int sats_stats = 0;	

	rc_led_set(RC_LED_GREEN, 1);
	rc_led_set(RC_LED_RED, 1);
	
	//
	if(USE_GPS)
	{
		printf("GPS check! Press x to stop\n");
		while(sats_stats < 6)
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
			sats_stats = buf[0]-'0';
			printf("Number of sats: %d \n", sats_stats);
			rc_usleep(2000000);
			if(sats_stats > 50) printf("Error in sats\n");
			char keyput = getchar();	
			if(keyput == 'x')
			{
				rc_set_state(EXITING);
				break;
			} 
		}
		get_gps(&lat,&lon);
		printf("Lat: %f \n", lat);
		printf("Lon: %f \n", lon);
		
	}
	

	if(rc_get_state()!=EXITING)
	{
		printf("PRESS ANY KAY TO SET HOME POINT AND START\n OR X TO STOP\n");
		drive(0.0,0.0);
		char keyput = getchar();	
		if(keyput == 'x') rc_set_state(EXITING);
		get_gps(&lat,&lon);
		const double home_lat = lat;
		const double home_lon = lon;
		printf("Home lat: %f \n", home_lat);
		printf("Home lon: %f \n", home_lon);
	}

	// Keep looPIng until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		if(rc_get_state()==RUNNING){
			// Sensor data
			// summa_0 = summa_0 + rc_adc_read_volt(0);
			// summa_1 = summa_1 + rc_adc_read_volt(1);
			// sensor_count++;
			// if((rc_nanos_since_epoch()-sensor_time)>SENSOR_TIMER)
			// {				
			// 	if(sensor(summa_0, summa_1, sensor_count))
			// 	{
			// 		rc_set_state(PAUSED);
			// 	}
			// 	summa_0 = 0;
			// 	summa_1 = 0;
			// 	sensor_count = 0;
			// 	sensor_time = rc_nanos_since_epoch();
			// }
			// if(rc_get_state()==PAUSED)
			// {
			// 	break;
			// }
			// GPS data
			if((rc_nanos_since_epoch()-gps_time)>GPS_TIMER)
			{
				get_gps(&lat,&lon);
				printf("Lat: %f \n", lat);
				printf("Lon: %f \n", lon);

				rc_usleep(100);// let the arduino have a nice time
				send_str = "Runing Sats\n";
				// Write
				bytes = strlen(send_str);
				rc_uart_write(bus, (uint8_t*)send_str, bytes);
				// Read
				memset(buf,0,sizeof(buf));	// zero buffer
				ret = rc_uart_read_line(bus, buf, sizeof(buf));
				sats_stats = buf[0]-'0';
				printf("Number of sats: %d \n", sats_stats);
				if(sats_stats < 6)
				{
					drive(0.0, 0.0);	// STOP
					printf("Lost sats. GPS check! Press x to stop\n");
					while(sats_stats < 6)
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
						sats_stats = buf[0]-'0';
						printf("Number of sats: %d \n", sats_stats);
						rc_usleep(2000000);
						if(sats_stats > 50) printf("Error in sats\n");
						char keyput = getchar();	
						if(keyput == 'x') rc_set_state(EXITING);
					}
				}
				// Calculate gps_heading
				gps_heading = Heading(lat,lon,go_lat,go_lon);	// in 0-360°
				gps_distance = Distance(lat, lon, go_lat, go_lon);	// in meters
				printf("Distance to point: %.2f\n",gps_distance);
			}

			if(gps_distance < 3)
			{
				printf("\n\nI'M HERE BABY!!!!\n\n");
				rc_set_state(EXITING);
				break;
			}
			// Rover data

			rover_heading = get_head();
			printf("GET HEAD: %f\n", rover_heading);
			turn = degToTurn(rover_heading-gps_heading);
			printf("Give turn: %f\n", turn);
			// calc rover driveing
			if((rc_nanos_since_epoch()-drive_time)>DRIVE_TIMER)
			{
				drive(speed,turn);
			}

		}
		else if(rc_get_state()==PAUSED)
		{
			// sensor stop
			drive(0.07,SERVO_ZERO);
			rc_set_state(RUNNING);
		}
		// STOPA SERVO OG ESC

		//break;
		
	
	}
	return;
}

//** functions TOOLS***//
double get_gps(double *lat, double *lon)
{	
	// LAt 				LONG
	// 6407.3901N, 2155.6000W
	// [9] N , [9]W
	// uint8_t fix[] = {6+48,4+48,48,7+48,46,3+48,9+48,48,1+48,78,44,32,
	// 						2+48,1+48,5+48,5+48,46,6+48,48,48,48,87};
	//UART
	char* send_str = "Fix\n";	// Ask for GPS data
	int bytes = strlen(send_str); // get number of bytes in test string
	uint8_t buf[BUF_SIZE];	// aka unsigned char * array! 
	int ret; // return value
	// SEND
	rc_uart_flush(bus);
	rc_uart_write(bus, (uint8_t*)send_str, bytes);

	// READ
	memset(buf,0,sizeof(buf));
	ret = rc_uart_read_line(bus, buf, sizeof(buf));
	if(ret<0) fprintf(stderr,"Error reading bus\n");
	else if(ret==0) printf("timeout reached, %d bytes read\n", ret);
	else printf("%s\n",buf);

	char temp_lat[9];
	char temp_lon[9];
	char *ptr;
	//printf("Get_gps Fix: %s\n",buf);
	for(int i=0; i<(int)sizeof(buf)-1; i++)
	{
		if(buf[i] == 'N'-'0') break;
		temp_lat[i] = buf[i];
	}
	*lat = strtod(temp_lat,&ptr)/100.0;
	//printf("Fix is now: %s\n",fix);
	for(int i=0; i<(int)sizeof(buf)-1; i++)
	{
		if(buf[i] == 'N'-'0' || buf[i] == 'W'-'0') break;
		temp_lon[i] = buf[i];		
	}
	*lon = -strtod(temp_lon,&ptr)/100.0;
	
	return 0;
}

bool sensor(double summa_0, double summa_1, int counter)
{
	double voltage_0 = summa_0/(counter);
	double voltage_1 = summa_1/(counter);
	// summa_0 = 0;
	// summa_1 = 0;
	//printf("Herna er counter orðið: %i \n",counter);
	// counter = 0;
	printf(" adc_0 |%6.2f |\n", voltage_0);
	printf(" adc_1 |%6.2f |\n", voltage_1);
	if(voltage_0 > MAX_SENSOR_VOLT || voltage_1 > MAX_SENSOR_VOLT){
		printf("ónei, danger!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		rc_led_set(RC_LED_USR1,1);
		rc_led_set(RC_LED_USR3,0);
		return true;
	}
	else{
		rc_led_set(RC_LED_USR1,0);
		rc_led_set(RC_LED_USR3,1);
	}
	return false;
} 

int setup()
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

	return 0;
}

int UART_start()
{
	char* send_str = "Time\n";
	int bytes = strlen(send_str); // get number of bytes in test string
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
	rc_uart_write(bus, (uint8_t*)send_str, bytes);

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

	if(USE_SERVO) rc_servo_power_rail_en(0);
	printf("Servo power OFF\n");
	rc_servo_cleanup();
	// MPU
	rc_mpu_power_off();	
	// close
	if(USE_GPS) rc_uart_flush(bus);
	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_uart_close(bus);
	rc_led_cleanup();
	rc_adc_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove PId file LAST
	
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


	if((servo<-0.01) && (servo>MIN))	// TURN RIGHT
	{
		//printf("RIGHT TURN \n\r");		
		if((servo+SERVO_ZERO)<SERVO_MIN)
		{
			SERVO_TURN = SERVO_MIN;	
		} 	
		else
		{
			SERVO_TURN = servo+SERVO_ZERO;
		}
	}
	else if((servo>0.01) && (servo<MAX))	// TURN LEFT
	{
		// printf("LEFT TURN \n\r");
		if((servo+SERVO_ZERO)>SERVO_MAX)
		{
			 SERVO_TURN = SERVO_MAX;
		}
		else
		{
			SERVO_TURN = servo+SERVO_ZERO;
		}	
	}
	else
	{
		printf("STOP TURN \n\r");
		SERVO_TURN = SERVO_ZERO;
	}


	if((esc<-0.01) && (esc>MIN))	// TURN RIGHT
	{
		//printf("RIGHT TURN \n\r");		
		if((esc+ESC_ZERO)<ESC_MIN)
		{
			ESC_TURN = ESC_MIN;	
		} 	
		else
		{
			ESC_TURN = esc+ESC_ZERO;
		}
	}
	else if((esc>0.01) && (esc<MAX))	// TURN LEFT
	{
		// printf("LEFT TURN \n\r");
		if((esc+ESC_ZERO)>ESC_MAX)
		{
			 ESC_TURN = ESC_MAX;
		}
		else
		{
			ESC_TURN = esc+ESC_ZERO;
		}	
	}
	else
	{
		printf("STOP RED \n\r");
		ESC_TURN = ESC_ZERO;
	}
	printf("Speed: %f\n\r",ESC_TURN);
	printf("Turn: %f\n\r",SERVO_TURN);
	return;
}

void wasd_drive()
{
	
	//double turn;
	//double speed;
	double step = 0.05;
	//double MAX = 1.0;
	double servo_of = -0.3;
	//double ESC_MIN = -0.60;
	//double ESC_MAX = 0.60;
	// uint64_t time_0 = rc_nanos_since_epoch();
	// printf("Hi \n\r");
	// if((rc_nanos_since_epoch()-time_0)>350000000)
	// {
	// 	printf("Hæ \n\r");
	// 	if(SERVO_TURN<0)
	// 	{
	// 		SERVO_TURN = SERVO_TURN	+step;
	// 	}
	// 	else if(SERVO_TURN>0)
	// 	{
	// 		SERVO_TURN = SERVO_TURN - step;
	// 	}
	// 	else SERVO_TURN = 0;
	// 	if(ESC_TURN<0)
	// 	{
	// 		ESC_TURN = ESC_TURN	+step;
	// 	}
	// 	else if(ESC_TURN>0)
	// 	{
	// 		ESC_TURN = ESC_TURN - step;
	// 	}
	// 	else ESC_TURN = 0;
	// 	time_0 = rc_nanos_since_epoch();		
		
	// } 
	char keyput = getchar();	
	if(keyput == 'x') rc_set_state(EXITING);
	switch(keyput){
		case 'w':
			ESC_TURN = ESC_TURN + step;
			if(ESC_TURN>ESC_MAX) ESC_TURN = ESC_MAX;
			break;
		case 's':
			ESC_TURN = ESC_TURN - step;
			if(ESC_TURN<ESC_MIN) ESC_TURN = ESC_MIN;
			break;
		case 'a':
			SERVO_TURN = SERVO_TURN+step;
			if(SERVO_TURN>(MAX+servo_of)) SERVO_TURN = (MAX+servo_of);
			break;
		case 'd':
			SERVO_TURN = SERVO_TURN - step;
			if(SERVO_TURN<-MAX) SERVO_TURN = -MAX;
			break;
		case ' ':
			printf("STOP\n\r");
			ESC_TURN = 0;
			break;
		default:
			printf("Use WASD to move\n\r");
			printf("f to stop program\n\r");
			break;		
	}

	printf("Speed: %f\n\r",ESC_TURN);
	printf("Turn: %f\n\r",SERVO_TURN);
	
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
	// Keep looPIng until state changes to EXITING
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
		// ESC_TURN = val;
		// printf("Enter an Servo val: ");
		// scanf("%f", &val);
		// if(val>1) break;
		// SERVO_TURN = val;		
	
	}
	return;
}
int mpu_start()
{
	// start with default config and modify based on options // SETJA Í SETUP FALL
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = 1;
	if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
	}
	return 0;	
}

double get_head()
{
	printf(" FusedTaitBryan(deg) Z direction |\n");
	double heading = (((double)((int)((-1.0*data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG+360.0)*100) %36000) )/100);
	printf("%6.2f \n", heading);
	return heading;
}

 double ToRad(const  double degree) 
{ 
	// cmath library in C++ 
	// defines the constant 
	// M_PI as the value of 
	// PI accurate to 1e-30 
	double one_deg = M_PI / 180.0; 
	return (one_deg * degree); 
} 

double ToDeg(double radian){
        double deg = radian*180/M_PI;
        return deg;
}

double Heading( double nLat1, double nLon1, double nLat2, double nLon2) {
    double lat1 = ToRad(nLat1);
    double lon1 = ToRad(nLon1);
    double lat2 = ToRad(nLat2);
    double lon2 = ToRad(nLon2);

    //==================Heading Formula Calculation================//

    double y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1);
    double x = sin(lon2-lon1)*cos(lat2);
    double brng = atan2f(x,y);
    brng = ToDeg(brng);// radian to degrees
	//printf("brng raw %f \n",brng);
	//printf("brng raw %i \n",(int)brng);
    brng = ( ((int)brng + 360) % 360 ); 
    return brng;

}

double Distance( double nLat1, double nLon1, double nLat2, double nLon2 )
{
     double nRadius = 6371.0; // Earth's radius in Kilometers
    // Get the difference between our two points
    // then convert the difference into radians
 
     double nDLat = ToRad(nLat2 - nLat1);
     double nDLon = ToRad(nLon2 - nLon1);
 
    // Here is the new line
    nLat1 =  ToRad(nLat1);
    nLat2 =  ToRad(nLat2);
 
     double nA = pow ( sin(nDLat/2.0), 2.0 ) + cos(nLat1) * cos(nLat2) * pow ( sin(nDLon/2.0), 2.0 );
     double nC = 2.0 * atan2( sqrt(nA), sqrt( 1.0 - nA ));
     double nD = (nRadius * nC) * 1000;
 
    return nD; // Return our calculated distance
}
double degToTurn(double degrees)
{	
	degrees = ((double)((int)((degrees+360)*100)%36000))/100;
	if(degrees >= 0 && degrees < 180){ 	//Beygja litið til hægri
		return (SERVO_ZERO+(SERVO_MIN-SERVO_ZERO)*2.0*degrees/360.0);
	}
	else if(degrees >= 180 && degrees < 360){//Beygja full til vinstri
		return (SERVO_ZERO + (SERVO_MAX-SERVO_ZERO)*2.0*(360-degrees)/360.0);
	}
	else{
		return SERVO_ZERO;
	}
}	
