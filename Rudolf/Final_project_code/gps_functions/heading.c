#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

 double ToRad(const  double degree) 
{ 
	// cmath library in C++ 
	// defines the constant 
	// M_PI as the value of 
	// pi accurate to 1e-30 
	double one_deg = M_PI / 180.0; 
	return (one_deg * degree); 
} 

double ToDeg(double radian){
        double deg = radian*180/M_PI;
        return deg;
}

double heading( double nLat1, double nLon1, double nLat2, double nLon2) {
    double lat1 = ToRad(nLat1);
    double lon1 = ToRad(nLon1);
    double lat2 = ToRad(nLat2);
    double lon2 = ToRad(nLon2);

    //==================Heading Formula Calculation================//

    double y = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1);
    double x = sin(lon2-lon1)*cos(lat2);
    double brng = atan2f(x,y);
    brng = ToDeg(brng);// radian to degrees
	printf("brng raw %f \n",brng);
	printf("brng raw %i \n",(int)brng);
    brng = ( ((int)brng + 360) % 360 ); 
    return brng;

}

int main () {
    //From south end of runway to north end of runway. Should be approx 355 - 0 deg.
    //maybe try formula from here https://www.movable-type.co.uk/scripts/latlong.html?from=47.80423,-120.03866&to=47.830481,-120.00987
    double la1 = 64.121264;
    double lo1 = -21.936162;
    double la2 = 64.136255;
    double lo2 = -21.939014;

    double h2 = heading(la1,lo1,la2,lo2);
    
    printf("Heading of runway is: %.2f" , h2);
    return 0;
}