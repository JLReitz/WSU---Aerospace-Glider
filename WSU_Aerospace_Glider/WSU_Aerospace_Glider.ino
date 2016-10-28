/*
 * WSU Aerospace Club - Payload Team
 * Autonomous Glider Program
 * 
 * A program implemented to autonomously guide a glider from its release from the WSU Aerospace Club's rocket in-air back to the launch site.
 * 
 * Designed and written by Jensen Reitz
 * 
 * - Utilizing Tinyduino and various TinySheilds, including the TinySheildGPS, by TinyCircuits.com
 * - Special thanks to Ben Rose of TinyCurcuits for his help in converting the GNRMC strings output by the TinySheild GPS into GPRMC strings that are usable by various tracking 
 *   programs
 * - Thanks also to Dave Reitz for helping with the PID laws
 */

#include "SoftwareSerial256.h"
#include <math.h>
#include <Servo.h>


//**************************************** Destination Latitude and Longitude ***********************************************
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double destination_Latitude = 0, destination_Longitude = 0; //Change these values to match the destination's location//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// The Arduino pins used by the GPS module
const int GPS_ONOFFPin = A3;
const int GPS_SYSONPin = A2;
const int GPS_RXPin = A1;
const int GPS_TXPin = A0;
const int GPSBaud = 9600;

//GPS coordinate info
bool gps_Valid = false; //If any of the coordinate points are currently unavailable, the value is set to false
double gps_Lat = -1, gps_Long = -1, gps_Course = -1; //Stored in units of Arcseconds
char gps_Lat_Direction = '\0', gps_Long_Direction = '\0';
String gps_String = "\0";

//Destination info
const double dest_Lat = destination_Latitude * 3600, dest_Long = destination_Longitude * 3600;

//Time-related info
int currentTime = 0, pastTime = 0; //milliseconds

//PID terms
double kp = 1, kr = 1, ki = 1, t = .1, tau = 1; //Constants - These values are defaulted to 1, but should be optimized to increase PID effeciency
double dB_old = 0, rateCmd_old = 0, intCmd_old = 0; //Past-time values

// Set which sentences should be enabled on the GPS module
char nmea[] = {'0'/*GPGGA*/, '0'/*GNGLL*/, '0'/*GNGSA*/, '0'/*GPGSV/GLGSV*/, '1'/*GNRMC*/, '0'/*GNVTG*/, '0'/*not supported*/, '0'/*GNGNS*/};

// The GPS connection is attached with a software serial port
SoftwareSerial Gps_serial(GPS_RXPin, GPS_TXPin);

Servo rudder;

void setup()
{
  Gps_serial.begin(GPSBaud);
  Serial.begin(115200);

  //Initialize the rudder and set it to straight
  rudder.attach(9);
  incrementRudder(0);
  
  // Init the GPS Module to wake mode
  pinMode(GPS_SYSONPin, INPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  pinMode(GPS_ONOFFPin, OUTPUT);
  delay(100);
  
  Serial.print("Attempting to wake GPS module.. ");
  while (digitalRead( GPS_SYSONPin ) == LOW )
  {
    // Need to wake the module
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(100);
  }
  Serial.println("done.");
  
  delay(100);

  //Written by Ben Rose
  char command[] = "$PSRF103,00,00,00,01*xx\r\n";
  for (int i = 0; i < 8; i++) 
  {
    command[10] = i + '0';
    command[16] = nmea[i];
    int c = 1;
    
    byte checksum = command[c++];
    
    while (command[c] != '*')
      checksum ^= command[c++];
      
    command[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
    command[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));
    
    Gps_serial.print(command);
    
    delay(20);
  }
}

void loop() 
{
  //Do nothing until the NMEA string is valid
  while(Gps_serial.read()!='$')
  {
  }
  while(Gps_serial.available()<5);
  {
  }

  //Read GPS NMEA string from SoftwareSerial and insert into a character string
  Gps_serial.read();
  Gps_serial.read(); //skip two characters

  char c = Gps_serial.read();
    
  //determine senetence type
  if (c == 'R' || c == 'G') 
  {
    c = Gps_serial.read();

    if(c=='M')
    {
      logNMEA(1);

      //Serial.print("\nRaw Data: "); //Uncomment to see raw NMEA data
      //Serial.print(gps_String);
  
      //Parse gps_string for GPS data
      parse_GPS(gps_String);
  
      //printInfo(); //Uncomment for debugging

      //Serial.print("Distance to Destination: ");         //Uncomment for debugging
      //Serial.print(distanceToDest(dest_lat, dest_long));
      //Serial.println("m");
    }
  }

  currentTime = millis();

  //Adjust and run through the PID every 10ms
  if(currentTime-pastTime>=10)
  {
    //PID calculations and rudder adjustments happen here
    double dB = 0, intCmd = 0, rateCmd = 0, totalCmd;

    //Calculate dB
    dB = targetAngle(dest_Lat, dest_Long) - toRadians(gps_Course);
    dB = adjust(1, dB);

    rateCmd = rateCommand(dB, dB_old, rateCmd_old, t, tau);
    intCmd = intCommand(dB, dB_old, rateCmd_old, t);

    totalCmd = -(dB*kp)+(rateCmd*kr)+(intCmd*ki); //Must negate due to the inverse-turning properties of the rudder

    //Set the recorded values to past-time
    dB_old = dB;
    rateCmd_old = rateCmd;
    intCmd_old = intCmd;
    pastTime = currentTime;

    //Correct rudder position with totalCmd
    incrementRudder(totalCmd);
  }
}

//
// Custom Functions ************************************************************************************************************************************************************
//

void printInfo()
{
  if(gps_Valid)
  {
    Serial.print("Latitude: ");
    Serial.print(gps_Lat);
    Serial.print("* ");
    Serial.print(gps_Lat_Direction);
    Serial.print(", ");
    Serial.print(toRadians(gps_Lat));
    Serial.println("rad");

    Serial.print("Longitide: ");
    Serial.print(gps_Long);
    Serial.print("* ");
    Serial.print(gps_Long_Direction);
    Serial.print(", ");
    Serial.print(toRadians(gps_Long));
    Serial.println("rad)");

    Serial.print("Course: ");
    Serial.print(gps_Course);
    Serial.print("*, ");
    Serial.print(toRadians(gps_Course));
    Serial.println("rad");
  }
  else
  {
    Serial.println("Latitude: INVALID");
    Serial.println("Longitude: INVALID");
    Serial.println("Course: INVALID");
  }
}

void parse_GPS(String const gps_string) //float &gps_Lat, float &gps_Long, float &gps_Elev
{
  int a = 0, b, i = 0;
  String str_arr[13];

  b = gps_string.indexOf(',', a);

  //Parse the information into str_arr
  do
  {
    if(a==b) //If there is no data between two commas
      str_arr[i] = "";
    else
      str_arr[i] = gps_string.substring(a, b);

    //Increments the indecies of the next-found commas
    a = b+1;
    b = gps_string.indexOf(',', a);

    if(a==b) //If a and b are still located on the same indecie (happens when there is no data between commas)
      b = gps_string.indexOf(',', b);

    i++;
    
  }while(i<13); //The data included in an NMEA string is always 13 different substrings seperated by commas

  if(str_arr[2]=="A") //If the validity stamp equals 'A', then the data is updated and valid for reading
  {
    //Read in Latitude and Longitude
    if(str_arr[3]=="" || str_arr[4]=="" || str_arr[5]=="" || str_arr[6]=="" || str_arr[8]=="")
    {
      //Set the info as invalid
      gps_Valid = false;
      
      //Set Latitude as invalid
      gps_Lat = 0;
      gps_Lat_Direction = '\0';
  
      //Set Longitude as invalid
      gps_Long = 0;
      gps_Long_Direction = '\0';
  
      //Set Course as invalid
      gps_Course = 0;
    }
    else
    {
      //Read in latitude
      int a;
      String lat_deg, lat_min;

      //Find the separation point between degrees and minutes
      a = str_arr[3].indexOf('.') - 2;

      //Grab the degrees and minutes of the location
      lat_deg = str_arr[3].substring(0, a);
      lat_min = str_arr[3].substring(a);

      //Grab the cardinal-latitude information
      gps_Lat_Direction = str_arr[4].charAt(0);

      //Convert the degrees and minutes into arcseconds
      if(gps_Lat_Direction=='N')
        gps_Lat = toRadians((lat_deg.toInt()) + (lat_min.toFloat()/60));
      else if(gps_Lat_Direction=='S')
        gps_Lat = -(toRadians((lat_deg.toInt()) + (lat_min.toFloat()/60)));

      //Read in Longitude
      int b;
      String long_deg, long_min;

      //Find the separation point between degrees and minutes
      b = str_arr[5].indexOf('.') - 2;

      //Grab the degrees and minutes of the location
      long_deg = str_arr[5].substring(0, b);
      long_min = str_arr[5].substring(b);

      //Grab the cardinal-longitude information 
      gps_Long_Direction = str_arr[6].charAt(0);
  
      if(gps_Long_Direction=='E')
        gps_Long = toRadians((long_deg.toInt()) + (long_min.toFloat()/60));
      else if(gps_Long_Direction=='W')
        gps_Long = -(toRadians((long_deg.toInt()) + (long_min.toFloat()/60)));

      //Read in course
      gps_Course = toRadians(str_arr[8].toFloat());

      gps_Valid = true;
    }
  }
  else
  {
    //Set the info as invalid
    gps_Valid = false;
      
    //Set Latitude as invalid
    gps_Lat = 0;
    gps_Lat_Direction = '\0';
  
    //Set Longitude as invalid
    gps_Long = 0;
    gps_Long_Direction = '\0';
  
    //Set Course as invalid
    gps_Course = 0;
  }
}

//Written by Ben Rose
void logNMEA(int type) 
{
  uint8_t buffer[100];
  
  buffer[0] = '$';
  buffer[1] = 'G';
  buffer[2] = 'P';
  
  if (type == 1) 
  {
    buffer[3] = 'R';
    buffer[4] = 'M';
  } else if (type == 2) 
  {
    buffer[3] = 'G';
    buffer[4] = 'G';
  }
  
  int counter = 5;
  char c = 0;
  
  while (!Gps_serial.available());
  {}
  
  c = Gps_serial.read();
  
  while (c != '*') 
  {
    buffer[counter++] = c;
    while (!Gps_serial.available());
    c = Gps_serial.read();
  }
  
  buffer[counter++] = c;
  
  while (!Gps_serial.available());
  {}
  
  c = Gps_serial.read();
  buffer[counter++] = c;
  
  while (!Gps_serial.available());
  {}
  
  c = Gps_serial.read();
  buffer[counter++] = c;
  buffer[counter++] = 0x0D;
  buffer[counter++] = 0x0A;
  buffer[counter] = '\0';


  c = 1;
  
  byte checksum = buffer[c++];
  
  while (buffer[c] != '*')
    checksum ^= buffer[c++];
    
  buffer[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
  buffer[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));

  gps_String = (char *)buffer;
}

void incrementRudder(int cmd)
{
  int i=rudder.read();
  
  for(i; i!=90+cmd;)
  {
    rudder.write(i);

    if(i<90+cmd)
      i++;
    else if(i>90+cmd)
      i--;

    delay(5);
  }
}

double intCommand(double en, double en_old, double rn_old, double dT)
{
  return rn_old+(en+en_old)*(dT/2);
}

double rateCommand(double en, double en_old, double rn_old, double dT, double tau)
{
  return (((2*tau)/dT)*(en-en_old)+((2*tau-dT)/dT)*rn_old)*(dT/(2*tau+dT));
}

//Returns the target angle of the glider in radians
double targetAngle(double dest_lat, double dest_long)
{
  const double radius = 6371000;
  double dNorth, dEast, dLat, dLong, target;

  dLat = dest_lat - gps_Lat;
  dLong = dest_long - gps_Long;

  dNorth = dLat*radius;
  dEast = dLong*radius*cos(dest_lat);

  return atan2(dEast, dNorth);
}

double distanceToDest(double dest_lat, double dest_long)
{
  const double radius = 6371000;
  
  double distance_Lat = dest_lat - gps_Lat, distance_Long = dest_long - gps_Long;
  double dNorth = radius * distance_Lat, dEast = radius * distance_Long * cos(dest_lat);

  return sqrt((dNorth*dNorth)+(dEast*dEast));
}

double adjust(int i, double angle)
{
  double adjusted = 0;
  
  if(i) //Convert angles over 180* to negative equivalent
  {
    if(angle>3.14)
      adjusted = angle - (2*3.14);
  }
  else //Convert negative angles under -180* to their positive equivalent
  {
    if(angle<-3.14)
      adjusted = angle + (2*3.14);
  }

  return adjusted;
}

double toRadians(double convert)
{
  return ((convert*3.14159)/180);
}
