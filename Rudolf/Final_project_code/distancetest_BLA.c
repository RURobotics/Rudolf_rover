#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

 float ToRad(const  float degree) 
{ 
	// cmath library in C++ 
	// defines the constant 
	// M_PI as the value of 
	// pi accurate to 1e-30 
	float one_deg = M_PI / 180.0; 
	return (one_deg * degree); 
} 

float CalculateDistance( float nLat1, float nLon1, float nLat2, float nLon2 )
{
     float nRadius = 6371.0; // Earth's radius in Kilometers
    // Get the difference between our two points
    // then convert the difference into radians
 
     float nDLat = ToRad(nLat2 - nLat1);
     float nDLon = ToRad(nLon2 - nLon1);
 
    // Here is the new line
    nLat1 =  ToRad(nLat1);
    nLat2 =  ToRad(nLat2);
 
     float nA = pow ( sin(nDLat/2.0), 2.0 ) + cos(nLat1) * cos(nLat2) * pow ( sin(nDLon/2.0), 2.0 );
 
     float nC = 2.0 * atan2( sqrt(nA), sqrt( 1.0 - nA ));
     float nD = (nRadius * nC) * 1000;
 
    return nD; // Return our calculated distance
}

int main () {
     float lat1 = 64.1255933333;
     float long1 = -21.9260233333;

     float lat2 = 64.1240216667;
     float long2 = -21.9266683333; 

     float distance = CalculateDistance(lat1,long1,lat2,long2);
     double dist2=0;
     dist2 = (double)distance;
    printf("%d \n",dist2);
    printf("Distance is %f m\n", (distance));

    return 0;
}