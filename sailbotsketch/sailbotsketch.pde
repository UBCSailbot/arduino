//sailbotsketch
//Created by David Lee on Dec 13, 2012

//based on
//UBCsailBot2012_2_14_5
//Ver. 2.14.5
//Last update by John K. June 9, 2012

#include <FastSerial.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <APM_RC.h>
#include <Wire.h>
#include <math.h>
#include <AP_GPS.h>         // ArduPilot GPS libray
#include <PIDv1.h>  
#include <avr/pgmspace.h>



#define rudder_output 0
#define sheet_output 1
#define RC_sail 1200
#define Read_GUI_Data_Challenge_Finished 1200


double rudder_centre = 1519;
double rudder_increment = 8.97;
double sheet_end = 1932;
double sheet_increment = 9.53736;
         
int leewayCor;
  
enum sailByCourse {  
  compassMethod,
  cogMethod,
  apparentWindMethod  
};  

int sheet_setting[8][4] = {
   {95,100,100,100}
  ,{90,86,86,86}
  ,{78,73,73,73}
  ,{57,54,54,54}
  ,{40,38,38,38}
  ,{29,29,29,29}
  ,{25,18,18,18}
  ,{35,20,20,35}
}; 

int mode;
enum{RC_MODE, AUTO_MODE};
int sheet_percentage=0;
String parsedData[5];
int sailByCourse;          

double Setpoint,Input,Output; //PID variables 
PID rudder(&Input,&Output,&Setpoint,1,0.05,0.25,DIRECT);//used to Initialize PID
double course;
double current_heading;
int COG;
int intSOG;
double SOG;
int apparentWind = 0;
double appWindAvg=0;
int numberSatelites;

long update_timer = 0;
long windTimer = 0;
long gpsTimer = 0;

struct Waypoint{
  long latitude;
  long longitude;
};

Waypoint *current_position = new Waypoint;


int radio_in[8];      // current values from the transmitter - microseconds
int radio_out[2];     // Output......0(rudder)  1(sheet winch)
int  pilot_switch = 1200;  
int  data_input_switch = 1200;

FastSerialPort0(Serial);    //USB port
FastSerialPort1(Serial1);   // GPS port (except for GPS_PROTOCOL_IMU)


GPS         *g_gps;
HEMISPHERE_GPS_NMEA     g_gps_driver(&Serial1);

//***********************************************************************************************************************************

void setup()
{

  delay(500);          //Wait at least 500 milli-seconds for device initialization
  Wire.begin();        // join i2c bus (address optional for master)

  Serial.begin(57600, 128, 128); 
  Serial1.begin(57600, 128, 128); 
  
  pinMode(A1, INPUT); //encoder pin
  digitalWrite(A1, HIGH);

  g_gps = &g_gps_driver;
  g_gps->init(); 

  APM_RC.Init();                        // Radio Initialization
  pilot_switch = 968;
  initPID();
}

//***********************************************************************************************************************************

void loop()
{
    read_radio();
    if(pilot_switch < RC_sail || data_input_switch < Read_GUI_Data_Challenge_Finished ) {     // This needs more testing
      mode=RC_MODE;
      rc_sail(); 
    }
    else{
      mode=AUTO_MODE;
      pi_sail();
    }
 
}

//***********************************************************************************************************************************

void read_radio(void)
{   
        for (int y = 0; y < 4; y++)  
            radio_in[y] = APM_RC.InputCh(y);    
        
        data_input_switch = radio_in[2];
        pilot_switch = radio_in[3]; 

}

//***********************************************************************************************************************************

void update_GPS(void)
{
    
    g_gps->update();
    current_position -> latitude = g_gps->latitude;
    current_position -> longitude = g_gps->longitude;  
    COG = ((double)(g_gps -> ground_course ))/100;
    intSOG = (int)g_gps->ground_speed; // in cm/s
    SOG = ((double)intSOG)/100;
    current_heading = ((double)(g_gps -> true_heading))/100; 
    numberSatelites = g_gps -> hemisphereSatelites;  
 
}

//***********************************************************************************************************************************
void update_ApprentWind(void)  {

  apparentWind = readEncoder();
  averageApprentWind();
  
}

//from http://rpg.dosmage.net/project/sailboat/_m_a3_8pde_source.html
int readEncoder(){
   const int  MA3_OFFSET=0;//this is the absolute offset
   double sensorValue = analogRead(A1);
   int degreeValue = convertTo360(sensorValue);
   return degreeValue;
}

int convertTo360 (double sensorValue){
    int result= (int)-1*sensorValue*360/1024;
    if(result < -180){
       result += 360;
     }
     return result;
}

void averageApprentWind() {
  //**TODO
  int diff = abs(appWindAvg-apparentWind);
  if (diff>=180 && appWindAvg<apparentWind){
    appWindAvg+=360;
  }
  else if (diff>=180 && appWindAvg>apparentWind){
    apparentWind+=360;
  }
  appWindAvg=0.999*(double)appWindAvg+0.001*(double)apparentWind;
  if (appWindAvg>180){
    appWindAvg-=360;
  }
  else if (appWindAvg<-180){
    appWindAvg+=360;
  }
}




//***********************************************************************************************************************************
void emergencySail() {
  while(pilot_switch < RC_sail) {
     read_radio();   
     APM_RC.OutputCh(rudder_output, radio_in[rudder_output]);           
     APM_RC.OutputCh(sheet_output, radio_in[sheet_output]);   
  }  
}



//***********************************************************************************************************************************

void rc_sail() {
  
   float rcSheetPercent;  
   
   read_radio();   
   APM_RC.OutputCh(rudder_output, radio_in[rudder_output]);           
   APM_RC.OutputCh(sheet_output, radio_in[sheet_output]);   
   
   rcSheetPercent = (sheet_end - radio_in[sheet_output])/sheet_increment;
   sheet_percentage = pow(rcSheetPercent,0.625) * 5.62 ;
   
   update_GPS();
   update_ApprentWind();
   printTelemetryData();       
            
}

void printTelemetryData(){
   char guiDataRC[200]; 
   char cogStr[10];
   char current_headingStr[10];
   char sogStr[10];
   if(millis() - update_timer >= 50) {
  
       update_timer = millis();
       dtostrf(SOG, 7, 2, sogStr);          
       dtostrf(COG, 7, 0,cogStr );     
       dtostrf(current_heading, 7, 1,current_headingStr );  
        
       sprintf(guiDataRC,"%d, %11ld, %11ld, %8s, %8s, %8d, %8d, %8d, %8d, %8d, %8s, %8d", 
           mode, current_position -> longitude, current_position -> latitude,cogStr,current_headingStr,apparentWind, 
           (int)appWindAvg,sheet_percentage,g_gps -> hemisphereSatelites,g_gps->hdop, sogStr, (int)Output);  
                                                                                                                                                                                                                                                                                              
       Serial.println(guiDataRC);                            
    }                                                            

  
}
//***********************************************************************************************************************************

void pi_sail() {
  
  read_data_fromPi();
  update_GPS();
  update_ApprentWind();  
  printTelemetryData();
  
  adjust_sheets(sheet_percentage);
  steer();
}

 
//***********************************************************************************************************************************

void read_data_fromPi () {

   #define INLENGTH 250
   char inString[INLENGTH + 1] ;
   int inCount; 
   inCount = 0;
   while(Serial.available() > 0)  {                                                 
     inString[inCount++] = Serial.read();                                           
     delay(2);   // Changed from 1 May 17, 2012 by JK. 
   }
   inString[inCount] = '\0';
   if (inCount>1){
     parsePiData(inString);
   }

}


void parsePiData(char charArray[]){
   const char *ptr = charArray;
   char field [ 32 ]; //arbitrary 32 char limit
   int n;

   for (int count=0; sscanf(ptr, "%31[^,]%n", field, &n) == 1; count++)
   {
      //Serial.println(field);
      parsedData[count]=field;
      
      ptr += n; /* advance the pointer by the number of characters read */
      if ( *ptr != ',' )
      {
         break; /* didn't find an expected delimiter, done? */
      }
      ++ptr; /* skip the delimiter */
   }
   
   if(parsedData[0].equalsIgnoreCase("ADJUST_SHEETS")){
     sheet_percentage=parsedData[1].toInt();
   }
   else if(parsedData[0].equalsIgnoreCase("STEER")){
    sailByCourse = parsedData[1].toInt();
    course = parsedData[2].toInt();
   }
   else if(parsedData[0].equalsIgnoreCase("TACK")){
     short weather = parsedData[1].toInt();
     boolean starboard = parsedData[2].toInt();
     tack(weather,starboard);
   }
   else if(parsedData[0].equalsIgnoreCase("GYBE")){
     //TODO
   }
}
//***********************************************************************************************************************************
void initPID() {
  
  rudder.SetMode(AUTOMATIC);
  
  //**TODO** test these constants
  int  rudderLimit=20;
  double kp = 1.2;
  double ki = 0.05;
  double kd = 0;
  double pidInterval = 200;
  
  rudder.SetOutputLimits(-rudderLimit,rudderLimit);
  rudder.SetTunings(kp, ki, kd);                     //set PID Constants    
  rudder.SetSampleTime(pidInterval);   
  
}


void adjust_sheets(int sheet_percent) {
   int altRcPercent;       
   altRcPercent = pow( sheet_percent,1.74) * 0.033 ;
   APM_RC.OutputCh(sheet_output,sheet_end - altRcPercent*sheet_increment ); 
}

//***********************************************************************************************************************************

void steer() {
  calculate_PID_input(sailByCourse);
  rudder.Compute();          //PID calculates rudder Output correction
  APM_RC.OutputCh(rudder_output, Output*rudder_increment + rudder_centre);
}


//***********************************************************************************************************************************

void calculate_PID_input(int sailByCourse) {

    int difference; 
    Input = 0;
       
    switch(sailByCourse) {
   
    case compassMethod: difference = course - current_heading;
    break;
    
    case cogMethod: difference = course - COG;
    break;
 
    case apparentWindMethod: difference = appWindAvg - course;
    break;      

    }    
    
    if (difference > 180 || difference < -180) {
        if (difference < 0) {
            difference += 360;
        }
        else {
            difference -= 360;
        }
        Setpoint = 0;
        Input -= difference;    
        }
        else {
            Setpoint = 0;
            Input -= difference;        
        }        
}



//***********************************************************************************************************************************
// TACKING FUNCTIONS
//***********************************************************************************************************************************
void tack(short weather, boolean starboard) { //based off longDistanceRaceTack from 2012
  int tack_rudder_angle=80; //average value from last year
  int H = ((double)tack_rudder_angle/100)*45;   
  int baseRudderTime;
  int preTackRudderAngle;
  
  Serial.println("$FTack ");
  
  preTackRudderAngle = leewayCor;   //**TODO se this value
  
  switch(weather) {
    case 0: baseRudderTime = 1500;
            break;  
    case 1: baseRudderTime = 1300;
            break;          
    case 2: baseRudderTime = 1200;
            break;
    case 3: baseRudderTime = 1100;
            break;             
  } 
  

  if (!starboard) H = -H;
   
  if (starboard) {   
     preTackRudderAngle =  -preTackRudderAngle;    
     
  }
    
 
  adjust_sheets(95); //power up for the tack
  APM_RC.OutputCh(rudder_output,((preTackRudderAngle*rudder_increment) + rudder_centre));    
  waitForSpecifiedDuration(2000);


  APM_RC.OutputCh(rudder_output,((0.5*H*rudder_increment) + rudder_centre));  
  waitForSpecifiedDuration(baseRudderTime);

  APM_RC.OutputCh(rudder_output, ((1.2*H*rudder_increment) + rudder_centre)); 
  waitForSpecifiedDuration(baseRudderTime*2);
  
  waitForSpecifiedDuration(baseRudderTime);
 
  updateAverageApparentWindAfterTack();
}

void updateAverageApparentWindAfterTack(){ 
 int apparentCount = 0;
 int apparentTotal = 0;
 long apparentTimer = millis();
  
 while(apparentCount < 10) {
   
   if(millis() - apparentTimer >= 50) {
     
      update_ApprentWind(); 
      apparentTotal += apparentWind;
      apparentCount++;
      apparentTimer = millis();
   }
 }
 apparentWind = apparentTotal/10;

 for(int i  = 0; i < 120 ; i++)                           
      averageApprentWind();


}

void checkForRCOverride(){
    read_radio();
    if(pilot_switch < RC_sail) {
        emergencySail();
    }
}

void waitForSpecifiedDuration(int duration){
  long timer = 0;
  long startTimer = millis();
  while(timer < duration){  
    timer = millis() - startTimer;
	checkForRCOverride();
  }
}
