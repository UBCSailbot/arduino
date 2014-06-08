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
#define RESET_INSTRUCTIONS 1200



//======RC Calibration Settings

double rudder_centre = 1519+100; //11 degree offset on 2013 rudder
double rudder_increment = 8.97;

// RC Value when sheets are all the way out
double sheet_fully_out = 1900;
// RC Value when sheets are all the way in
double sheet_fully_in = 1319;

double sheet_increment = (sheet_fully_out - sheet_fully_in)/100.0;

enum sailByCourse {  
  compassMethod,
  cogMethod,
  apparentWindMethod,
  holdCourseMethod  
};  

int sheet_setting[8][4] = {
  {
    95,100,100,100  }
  ,{
    90,86,86,86  }
  ,{
    78,73,73,73  }
  ,{
    57,54,54,54  }
  ,{
    40,38,38,38  }
  ,{
    29,29,29,29  }
  ,{
    25,18,18,18  }
  ,{
    35,20,20,35  }
}; 

int mode;
enum{
  RC_MODE, AUTO_MODE};
int sheet_percentage=0;
const int PARSED_DATA_COUNT = 5;
String parsedData[PARSED_DATA_COUNT];
int sailByCourse;          

double Input,Output; 
double Setpoint=0;
PID rudder(&Input,&Output,&Setpoint,1,0.05,0.25,DIRECT);//used to Initialize PID
double course;
double current_heading;
int COG;
int intSOG;
double SOG;
int apparentWind = 0;
double appWindAvg=0;
double DEFAULT_WIND_AVERAGE_CONSTANT = .999;
int numberSatelites;
int rudderAngle=0;
double battery_voltage_1;
double battery_voltage_2;

long update_timer = 0;
long windTimer = 0;
long gpsTimer = 0;
int  ENCODER_OFFSET=-161;
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
  // Pin A1 is the wind sensor
  pinMode(A1, INPUT);
  // Pin A0 is the voltage reader for the circuit
  pinMode(A0, INPUT);
  // Pin A2 is the voltage reader for the winch
  pinMode(A2, INPUT);
  digitalWrite(A1,HIGH);
  Serial.begin(57600, 128, 128); 
  Serial1.begin(57600, 128, 128); 
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
  
  
  if(pilot_switch < RC_sail) {     // This needs more testing
    mode=RC_MODE;
    rc_sail(); 
  }
  else{
    if (mode==RC_MODE){ //reset PID when switched into Auto
      sailByCourse=holdCourseMethod;//garbage value to stop PID From running
    }
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

  // Cap at sheet_fully_out if it tries to go over
  if(radio_in[sheet_output] > sheet_fully_out)
  {
    radio_in[sheet_output] = sheet_fully_out;
  }

  // Cap at sheet_fully_in if it tries to go under
  if(radio_in[sheet_output] < sheet_fully_in)
  {
    radio_in[sheet_output] = sheet_fully_in;
  }
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
  averageApprentWind(DEFAULT_WIND_AVERAGE_CONSTANT);
}

//from http://rpg.dosmage.net/project/sailboat/_m_a3_8pde_source.html
int readEncoder(){
  double sensorValue = analogRead(A1);
  //Serial.println(sensorValue);
  int degreeValue = convertTo360(sensorValue);
  return degreeValue;
}

int convertTo360 (double sensorValue){
  int result= (int)-1*sensorValue*360/1023;
  result+=ENCODER_OFFSET;
  if(result < -180){
    result += 360;
  }
  return -result;
}

void averageApprentWind(double k) {
  //**TODO
  int currentWind=apparentWind;
  int diff = abs(appWindAvg-currentWind);
  if (diff>=180 && appWindAvg<currentWind){
    appWindAvg+=360;
  }
  else if (diff>=180 && appWindAvg>currentWind){
    currentWind+=360;
  }
  appWindAvg=k*(double)appWindAvg+(1-k)*(double)currentWind;
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

  rcSheetPercent = (sheet_fully_out - radio_in[sheet_output])/sheet_increment;
  sheet_percentage = pow(rcSheetPercent,0.625) * 5.62 ;
  rudderAngle=(radio_in[rudder_output]- rudder_centre)/rudder_increment;

  update_GPS();
  update_ApprentWind();
  printTelemetryData();       

}

void printTelemetryData(){
  char guiDataRC[200]; 
  char cogStr[10];
  char current_headingStr[10];
  char sogStr[10];
  char battery_voltage_string_1[10];
  char battery_voltage_string_2[10];


  int resetInstructions =(int) (data_input_switch>RESET_INSTRUCTIONS);
  if(millis() - update_timer >= 50) {

     /*
       The battery voltage is read on the arduino's analog pins, which have a 0-5 voltage range, which is read by the arduino as
       in the range of 0-1023 (ie. 5V will read as 1023)
       Since our battery voltage is actually above 5V, we use a voltage divider to divide the voltage into 2 before passing it to the arduino
     */
    int battery_voltage_reading_1 = analogRead(A0);
    int battery_voltage_reading_2 = analogRead(A2);
    battery_voltage_1 = 2*(battery_voltage_reading_1/1024.0)*5.0;
    battery_voltage_2 = 2*(battery_voltage_reading_2/1024.0)*5.0;
    
    update_timer = millis();
    dtostrf(SOG, 7, 2, sogStr);          
    dtostrf(COG, 7, 0,cogStr );     
    dtostrf(current_heading, 7, 1,current_headingStr );
    dtostrf(battery_voltage_1, 7, 3, battery_voltage_string_1);
    dtostrf(battery_voltage_2, 7, 3, battery_voltage_string_2);

    sprintf(guiDataRC,"%d, %11ld, %11ld, %8s, %8s, %8d, %8d, %8d, %8d, %8d, %8s, %8d, %d, %d, %d, %d", 
    mode, current_position -> longitude, current_position -> latitude,cogStr,current_headingStr,apparentWind, 
    (int)appWindAvg,sheet_percentage,g_gps -> hemisphereSatelites,g_gps->hdop, sogStr, rudderAngle,resetInstructions,battery_voltage_reading_1, battery_voltage_reading_2, radio_in[sheet_output]);  

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

  for (int count=0; sscanf(ptr, "%31[^,]%n", field, &n) == 1 && count<PARSED_DATA_COUNT ; count++)
  {
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
    boolean starboard = parsedData[1].toInt();
    gybe(starboard);
  }
}
//***********************************************************************************************************************************
void initPID() {

  rudder.SetMode(AUTOMATIC);

  //**TODO** test these constants
  int  rudderLimit=20;
  double kp = 1.2;
  double ki = 0;
  double kd = 0;
  double pidInterval = 200;
  rudder.SetOutputLimits(-rudderLimit,rudderLimit);
  rudder.SetTunings(kp, ki, kd);                     //set PID Constants    
  rudder.SetSampleTime(pidInterval);   

}


void adjust_sheets(int sheet_percent) {
  int altRcPercent;
  altRcPercent = pow( sheet_percent,1.74) * 0.033 ;
  APM_RC.OutputCh(sheet_output, sheet_fully_out - altRcPercent*sheet_increment ); 
}

//***********************************************************************************************************************************

void steer() {
  calculate_PID_input(sailByCourse);
  rudder.Compute();          //PID calculates rudder Output correction
  rudderAngle=Output;
  APM_RC.OutputCh(rudder_output, Output*rudder_increment + rudder_centre);
}


//***********************************************************************************************************************************

void calculate_PID_input(int sailByCourse) {

  int difference=0; 
  Input = 0;

  switch(sailByCourse) {

  case compassMethod: 
    difference = course - current_heading;
    break;

  case cogMethod: 
    difference = course - COG;
    break;

  case apparentWindMethod: 
    difference = appWindAvg - course;
    break;    
  }    


  if (difference > 180 || difference < -180) {
    if (difference < 0) {
      difference += 360;
    }
    else {
      difference -= 360;
    }
    Input -= difference;    
  }
  else {
    Input -= difference;        
  }        
}



//***********************************************************************************************************************************
// TACKING FUNCTIONS
//***********************************************************************************************************************************
void tack(short weather, boolean starboard) { //based off longDistanceRaceTack from 2012
  int tack_rudder_percent=80; //average value from last year
  int H = ((double)tack_rudder_percent/100)*45;   
  int baseRudderTime;
  int preTackRudderAngle=5;

  switch(weather) {
  case 0: 
    baseRudderTime = 1500;
    break;  
  case 1: 
    baseRudderTime = 1300;
    break;          
  case 2: 
    baseRudderTime = 1200;
    break;
  case 3: 
    baseRudderTime = 1100;
    break;             
  } 

  if (!starboard){
    H = -H;
  }
  else {   
    preTackRudderAngle =  -preTackRudderAngle;        
  }    
  sheet_percentage=95;
  adjust_sheets(sheet_percentage); //power up for the tack

  rudderAngle = preTackRudderAngle;
  APM_RC.OutputCh(rudder_output,(preTackRudderAngle*rudder_increment + rudder_centre));    
  waitForSpecifiedDuration(2000);

  rudderAngle=0.5*H;
  APM_RC.OutputCh(rudder_output,(rudderAngle*rudder_increment + rudder_centre));  
  waitForSpecifiedDuration(baseRudderTime);

  rudderAngle =1.2*H;
  APM_RC.OutputCh(rudder_output, (rudderAngle*rudder_increment + rudder_centre)); 
  waitForSpecifiedDuration(baseRudderTime*2);

  updateAverageApparentWindAfterTack();
}

void updateAverageApparentWindAfterTack(){ 
  //this function artificially updates the average wind quickly to make up for the apparent wind change after the maneuver
  double TACKING_WIND_AVERAGE_CONSTANT =.99;
  long apparentTimer = millis();

  for(int i=0;i<10;i++) {

    if(millis() - apparentTimer >= 50) {
      update_ApprentWind(); 
      apparentTimer = millis();
    }
    averageApprentWind(TACKING_WIND_AVERAGE_CONSTANT);
  }



}


void gybe(boolean starboard) {  //based off station_keeping_gybe from 2012
  rudderAngle = 45; //maximum degrees
  if (starboard) rudderAngle = -rudderAngle;
  sheet_percentage=25;
  adjust_sheets(sheet_percentage);
  APM_RC.OutputCh(rudder_output, rudderAngle*rudder_increment + rudder_centre); 
  waitForSpecifiedDuration(4000);
  updateAverageApparentWindAfterTack();
}

void waitForSpecifiedDuration(int duration){
  long timer = 0;
  long startTimer = millis();
  while(timer < duration){  
    timer = millis() - startTimer;
    checkForRCOverride();
    update_GPS();
    update_ApprentWind();  
    printTelemetryData();
  }
}
void checkForRCOverride(){
  read_radio();
  if(pilot_switch < RC_sail) {
    emergencySail();
  }
}


