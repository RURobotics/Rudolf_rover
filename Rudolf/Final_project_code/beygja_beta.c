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

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}


	rc_make_pid_file();
	
	printf("Program has setup and started");

	double test_degrees[3] = {30, 180, 340};

	int k;
	for(k =0;k<3;k++)
	double turn = degToTurn(test_degrees[k]);
	printf("%f eru :",test_degree[k]);
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
	if(degrees >180){ 	//Beygja til vinstri
	degrees = degrees - 360;
	
	}
	else{			//Beygja til hægri
	
	}
	
	return turn;
}

