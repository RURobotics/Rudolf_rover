#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7
// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
//SoftwareSerial BBBserial(4,5);
Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
bool GPSECHO = false;

void setup()
{  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  delay(5000);
  //BBBserial.begin(115200);
  //delay(5000);
  Serial.println("Adafruit GPS library basic test!");  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}
String command = " ";
void loop()                     // run over and over again
{  
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // This is the part where we check if the BBBlue is sending data to the arduion
  if(Serial.available()){        
        command = Serial.readStringUntil('\n'); // The data MUST END ON NEW LINE \n
        if(command == "Time" || command == "Ti"){
            //Serial.print("Time: ");
            if (GPS.hour < 10) { Serial.print('0'); }
            Serial.print(GPS.hour, DEC); Serial.print(':');
            if (GPS.minute < 10) { Serial.print('0'); }
            Serial.print(GPS.minute, DEC); Serial.print(':');
            if (GPS.seconds < 10) { Serial.print('0'); }
            Serial.print(GPS.seconds, DEC); Serial.print('.');
            if (GPS.milliseconds < 10) {
              Serial.print("00");
            } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
              Serial.print("0");
            }
            Serial.println(GPS.milliseconds);
        }else if(command == "Hour"){
          if (GPS.hour < 10) { Serial.print('0'); }
            Serial.println(GPS.hour, DEC);
        }else if(command == "Min"){
          if (GPS.minute < 10) { Serial.print('0'); }
            Serial.println(GPS.minute, DEC);
        }else if(command == "Sec"){
          if (GPS.seconds < 10) { Serial.print('0'); }
            Serial.println(GPS.seconds, DEC);                           
        }else if(command == "Date"){
          //Serial.print("Date: ");
          Serial.print(GPS.day, DEC); Serial.print('/');
          Serial.print(GPS.month, DEC); Serial.print("/20");
          Serial.println(GPS.year, DEC);         
        }else if(command == "Fix" || command == "Fi" || command == "F"){
          //Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); 
          Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); 
          Serial.println(GPS.lon);
        }else if(command == "Speed" || command == "Sp"){
          //Serial.print("Speed m/s: "); 
          Serial.println(GPS.speed/1.944); // It is knots...
        }else if(command == "Angle" || command == "An"){
          //Serial.print("Angle: "); 
          Serial.println(GPS.angle); ///< Course in degrees from true north          
        }else if(command == "Alti"){
          //Serial.print("Altitude: "); 
          Serial.println(GPS.altitude);
        }else if(command == "Sats" || command == "Sat" || command == "Sa" || command == "S"){
          //Serial.print("Satellites: "); 
          Serial.println((int)GPS.satellites);
        }else if(command == "Help"){
          Serial.println("Alti: Altitude in meters above MSL ");
          Serial.println("Angle: Course in degrees from true north ");
          Serial.println("Date: GMT day, month, year ");
          Serial.println("Time: GMT hours, minutes, seconds, milliseconds ");
          Serial.println("Fix: Sata data");
          Serial.println("Sats: Number of satellites in use");          
        }else if(command == "GPSECHO"){
            GPSECHO = !GPSECHO;
        }
        else{
          Serial.print("Call \"Help\" for commands..");
          Serial.print("What is this? "); Serial.println(command);
        }
        command = "";
   }
}
