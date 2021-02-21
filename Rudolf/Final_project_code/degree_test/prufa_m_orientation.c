#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <stdlib.h> // for atoi() and exit()
//#include <rc/mpu.h>
//#include <rc/time.h>

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

static rc_mpu_data_t data;

// Global Variables
static int running = 0;
static int silent_mode = 0;
static rc_mpu_data_t data;

static void __print_data(void)
{
        printf("\r");
        printf(" ");
        // print fused TaitBryan Angles
	//printf(" FusedTaitBryan(deg) |"); 
	printf("%6.1f %6.1f %6.1f |",   data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
	                                data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                        data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
	fflush(stdout);
        return;
}

static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

//int main(int argc, char *argv[])
int main()
{

	//int c, sample_rate, priority;
        //int show_something = 0; // set to 1 when any show data option is given.
        // start with default config and modify based on options
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	conf.enable_magnetometer = 1;
	// set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running = 1;

	// now set up the imu for dmp interrupt operation
        if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }

	// write labels for what data will be printed and associate the interrupt
        // function to print data immediately after the header.
	 printf("  FusedTaitBryan(deg) | \n");
	if(!silent_mode) rc_mpu_set_dmp_callback(&__print_data);
        //now just wait, print_data() will be called by the interrupt
        while(running)  rc_usleep(100000);
        // shut things down
        rc_mpu_power_off();
        printf("\n");
        fflush(stdout);
        return 0;
}
