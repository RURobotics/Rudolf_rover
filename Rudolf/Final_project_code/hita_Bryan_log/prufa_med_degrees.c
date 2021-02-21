#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <stdlib.h> // for atoi() and exit()
#include <time.h> //for GMT time

#define I2C_BUS 2


static rc_mpu_data_t data;
const int timi_us = 5000000; //5sek steps
const int steps =  2880; // 360/30 min logging


int main() {
        // start with default config and modify based on options // SETJA Í SETUP
	
	FILE *f = fopen("file.txt", "w");
	if (f == NULL)
	{
	    printf("Error opening file!\n");
	    exit(1);
	}
	if(rc_kill_existing_process(2.0)<-2) return -1;
	// start with both LEDs off
        if(rc_led_set(RC_LED_GREEN, 0)==-1){
                fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
                return -1;
        }
        if(rc_led_set(RC_LED_RED, 0)==-1){
                fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
                return -1;
        }
	rc_usleep(500000);
	rc_led_set(RC_LED_GREEN,1);
	rc_make_pid_file();
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = 1;
	//conf.orient = ORIENTATION_X_FORWARD;
	conf.dmp_fetch_accel_gyro=1;
	if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }
	rc_usleep(100000);
	
	time_t rawtime;
	struct tm *info;
	time(&rawtime);
	info = gmtime(&rawtime);
	int hour = info->tm_hour;
	int min = info->tm_min;
	int sec = info->tm_sec;
	double fused_taitBryan =0;
	rc_mpu_read_temp(&data);
	rc_usleep(500000);
        fprintf(f, "    Time   |  sec | Bryanraw|  Bryan |  Temp  |\n");
	int k = 0;
	//double prog  = 0.1;
	while(k<steps){
		rc_usleep(timi_us/2);
		sec = sec + (timi_us/1000000);
		if(sec>59){
			sec = sec%60;
			min = min + 1;
		}
		if(min>59){
			min = min%60;
			hour = hour + 1;
		}
		fused_taitBryan = ((double)( (int)( (-1.0*data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG+360.0) *100) %36000) )/100;
	        fprintf(f, " %2d:%2d:%2d  | %4i | %6.1f  | %6.2f | %6.2f |\n", hour, min, sec, \
							k*(timi_us/1000000),\
							data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG,\
							fused_taitBryan ,\
							data.temp);
	       rc_usleep(timi_us/2);        
		k = k+1;	
		rc_mpu_read_temp(&data);
        }
	
	//printf("%6.2f \n", ( ((double)( (int)( (-1.0*data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG+360.0) *100) %36000) )/100));
        rc_mpu_power_off();
        printf("hættur \n");
	fclose(f);
	rc_led_set(RC_LED_GREEN,0);
        rc_led_set(RC_LED_RED,1);
	rc_led_cleanup();
        rc_remove_pid_file();
        return 0;
}
