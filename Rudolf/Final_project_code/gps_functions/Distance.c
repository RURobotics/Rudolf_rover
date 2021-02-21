#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

const double pi = 3.14159265359;
 double ToRad(const  double degree) 
{ 
	// cmath library in C++ 
	// defines the constant 
	// M_PI as the value of 
	// pi accurate to 1e-30 
    
	double one_deg = pi / 180.0; 
	return (one_deg * degree); 
} 

double CalculateDistance( double nLat1, double nLon1, double nLat2, double nLon2 )
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

int main () {
    //points from gpsdata2 outside HR
    double lat1 = 64.1255933333;
    double lon1 = -21.9260233333;
    double lat2 = 64.1240216667;
    double lon2 = -21.9266683333; 

    //Start and End reykjavik runway
    //64.121264, -21.936162
    //64.136255, -21.939014
    double la1 = 64.121264;
    double lo1 = -21.936162;
    double la2 = 64.136255;
    double lo2 = -21.939014;

    double distance = CalculateDistance(lat1,lon1,lat2,lon2);
    printf("Distance is %f m\n", (distance));

    double distance2 = CalculateDistance(la1,lo1,la2,lo2);
    printf("Distance of runway is %f \n", distance2);
    return 0;
}