/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

//GLOBAL VARIABLE
const double SERVO_MAX = 0.3;
const double SERVO_MIN = -0.9;
const double SERVO_ZERO = -0.3;

// function declarations
double degToTurn(double degrees);

int main()
{

	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}


	rc_make_pid_file();
	
	printf("Program has setup and started\n");

	double test_degrees[7] = {30.0,60.0, 90.0, 180.0, 270.0, 300.0 , 359.9};

	int k;
	for(k =0;k<7;k++){
	double turn = degToTurn(test_degrees[k]);
	printf("%f eru :",test_degrees[k]);
	printf("%f sem beygja\n",turn);
	}

	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
double degToTurn(double degrees)
{	
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

