#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <stdlib.h> // for atoi() and exit()


#define I2C_BUS 2


static rc_mpu_data_t data;

int main()
{

        // start with default config and modify based on options // SETJA Í SETUP FALL
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
	conf.enable_magnetometer = 1;
	if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }
	int k = 0;
	for(k=0;k<10;k++){
	rc_usleep(1000000);
	 printf(" FusedTaitBryan(deg) Z direction |\n");
	 printf("%6.2f \n", ( ((double)( (int)( (-1.0*data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG+360.0) *100) %36000) )/100));
	}

        rc_mpu_power_off();
        printf("\n");
        //fflush(stdout);
        return 0;
}
