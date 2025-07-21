//Required libraries
#include <Wire.h> //library for I2C usage
#include <Adafruit_MCP23X17.h> //library for GPIO expander
#include <Adafruit_ADS1X15.h> //library for ADS1115 usage
#include <Adafruit_NeoPixel.h> //library for WS2812b RGB LED strips
#include <LiquidCrystal_I2C.h> //library for I2C controlled LCD display
#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidrectional communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address
//#include <Preferences.h>

//Pin definitions (ESP32)
#define pwm1 32 //PWM of Motor 1
#define pwm2 33 //PWM of Motor 2
#define pwm3 25 //PWM of Motor 3
#define pwm4 26 //PWM of Motor 4
#define dcdcbat 16 //power switch for the DCDC converter that charges the battery from the external charge port
#define dcdcfc 17 //power switch for the DCDC converter that powers the FC controller from the battery
#define pwroff 19 //Relay for power off
#define pwrbtn 23 //NC output of power button
#define rgb 27 //RGB LED data pin

//Pin definitions (MCP23017)
#define int1 3 //interface pin to Arduino Nano #1 (MCP23017 A3)
#define int2 2 //Interface pin to Arduino Nano #2 (MCP23017 A2)
#define int3 1 //Interface pin to Arduino Nano #3 (MCP23017 A1)
#define int4 0 //Interface pin to Arduino Nano #4 (MCP23017 A0)
#define dir1 8 //Dir of Motor 1 (MCP23017 B0)
#define dir2 9 //Dir of Motor 2 (MCP23017 B1)
#define dir3 10 //Dir of Motor 3 (MCP23017 B2)
#define dir4 11 //Dir of Motor 4 (MCP23017 B3)
#define fcbat 13 //FC charge: relais for connecting FC to battery
#define fcsw 14 //FC controller: power button for fuel cell (connected to relais)

//Setup of PWM channels
const int freq       = 20000; //PWM frequency: 20 kHz
const int resolution = 8; //PWM with 8 bit (0 to 255)
const int Motor1     = 0; //PWM channel #0 of the ESP32 (you can have different channels with different frequencies at the same time)
const int Motor2     = 1;
const int Motor3     = 2; 
const int Motor4     = 3;

//Add peripherals
Adafruit_MCP23X17 mcp; //specification of the name of the GPIO expander in the sketch
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_ADS1115 adc1; //define first ADC in sketch
Adafruit_ADS1115 adc2; //define second ADC in sketch
int RGBcount = 48; //number of RGB LEDs of the strip -> 24 are in the bottom part of the chassis, plus 2x12 in the top part
Adafruit_NeoPixel strip(RGBcount, rgb, NEO_GRB + NEO_KHZ800); //Neopixel strip

//Input channels of the two ADS1115
#define Batcurrent 0 //channel 0 of adc1
#define Batvoltage 1 //channel 1 of adc1
#define Batcharge 2 //channel 2 of adc1
#define FCvoltage 3 //channel 3 of adc1
#define FCcurrent 4 //channel 2 of adc2
#define H2pressure 5 //channel 3 of adc2

//Global variables
const int intpins[] = {int1, int2, int3, int4}; //Simplify reading & using the interface with the Arduino Nano by creating vectors
int limits[] = {0, 0, 0, 0};
int rctimer = 0;
int analogcounter = 0;
int lcdtimer = 0;
double batvolt = 0;
double batcurr = 0;
double FCvolt = 0;
double FCcurr = 0;
double H2press = 0;
int rotate = 0;
int power = 0;
int motoroff = 0;
int motad = 0;
int motbc = 0;
double angle = 0;
double pi = 3.141592654;
double corr = 0;
double corrad = 0;
double corrbc = 0;
int ledstatus = 0;
int maxp = 178; //this is the maximum power for the motors; can be adjusted to change the robot's max speed
int batcounter0 = 0;
int batcounter1 = 0;
int fcstatus = 0;
int fccounter = 0;
int offdelay = 0;
int fcvoltcounter = 0;
int fcpresscounter = 0;
int lowbat = 0;
int lcdcounter = 0;
double Chargevolt = 0;
bool charging = false;
int pwrofftimer = 0;
int pwroffcounter = 0;
int signalofftimer = 0;
int motorcounter = 0;
bool manual = false; //change FC control mode: false = auto; true = manual

          //variables for ADC
          int val0 = 0;
          int val1 = 0;
          float v0 = 0;
          float v1 = 0;
          float v0_final = 0;
          float v1_final = 0;

          //variables for motors
          int directions = 0;
          int mot1 = 0;
          int mot2 = 0;
          int mot3 = 0;
          int mot4 = 0;
          int mot1old = 0;
          int mot2old = 0;
          int mot3old = 0;
          int mot4old = 0;

//ESPNOW setup is based on the project by Rand Nerd Tutorials (with some modifications as needed for this project).
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
//MAC addresses: Enter the specific MAC addresses of your ESPs to establish the communication
uint8_t RobotMAC[] = {0xAE, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address as the address of Hydrobot
uint8_t broadcastAddress[] = {0xA0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};    //MAC address of remote control
int signalquality = 10;
// Define a data structure for sending data to the remote control
typedef struct send_message {
  int voltageFC;
  int voltageBat;
  int currentFC;
  int currentBat;
  int H2p;
  int fcstat;
  int voltageCharge;
} send_message;
// Create a structured object
send_message outgoing;
// Define a data structure for receiving information from the remote control
typedef struct receive_message {
  int pwr;
  double angle;
  int rot;
  int fc;
  int lswitch;
  bool manual;  
} receive_message;
// Create a structured object
receive_message incoming;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS&&signalquality<19){ //signal quality is measured by changing the value of "signalquality" according to how many data packages were not received
    signalquality = signalquality +1;
  }else if(signalquality>0){
    signalquality = signalquality -1;
  }
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-Now lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

void setup() {
  //initialize serial monitor - can be used for code debugging
  Serial.begin(115200);

  //Initialize I2C peripherals
  lcd.init(); //LCD display start
  lcd.backlight(); //switch the backlight of the display on
  lcd.setCursor(0,0);
  lcd.print("HydrOBot startup");
  lcd.setCursor(0,1);

  mcp.begin_I2C(0x20); //default I2C address of MCP23017 (all ADDR pins at GND)
  adc1.begin(0x49); //default I2C address of ADS1115 (ADDR pin unconnected/at GND)
  adc2.begin(0x48); //adjusted I2C address of ADS1115 (ADDR pin to VDD)
  adc1.setDataRate(RATE_ADS1115_860SPS);
  adc2.setDataRate(RATE_ADS1115_860SPS);
  Wire.setClock(400000);

  //RGB LED strip start
  strip.begin();
  strip.clear();
  strip.show(); //initialize all pixels to 'off'
  for (int i=0; i<RGBcount; i++){ //testing the strip: Run through all pixels with R, then G, then B
    strip.setPixelColor(i,127,0,0);
    if (i>14){
      strip.setPixelColor(i-15,0,127,0);
    }
    if (i>30){
      strip.setPixelColor(i-31,0,0,127);
    }
    strip.show();
    if (i==0||i==5||i==10||i==15||i==20||i==25||i==30||i==35||i==40||i==45){
      lcd.print(".");
    }
    delay(50);
  }
  for (int i=31; i<RGBcount; i++){ //testing the strip: Run through all pixels with R, then G, then B
    strip.setPixelColor(i,0,127,0);
    strip.setPixelColor(i-15,0,0,127);
    strip.show();
    if (i==33||i==38||i==43){
      lcd.print(".");
    }    
    delay(50);
  }
  for (int i=31; i<RGBcount; i++){ //testing the strip: Run through all pixels with R, then G, then B
    strip.setPixelColor(i,0,0,127);
    strip.show();
    if (i==31||i==36||i==41){
      lcd.print(".");
    }      
    delay(50);
  }

  //initialization of GPIO pins
  mcp.pinMode(int1, INPUT); //do not activate the internal pullup for the interface pins: they are hardwired with pulldown resistors!
  mcp.pinMode(int2, INPUT);
  mcp.pinMode(int3, INPUT);
  mcp.pinMode(int3, INPUT);
  mcp.pinMode(dir1, OUTPUT);
  mcp.pinMode(dir2, OUTPUT);  
  mcp.pinMode(dir3, OUTPUT);
  mcp.pinMode(dir4, OUTPUT);
  mcp.digitalWrite(dir1, LOW);
  mcp.digitalWrite(dir2, LOW);
  mcp.digitalWrite(dir3, LOW);
  mcp.digitalWrite(dir4, LOW);
  mcp.pinMode(fcbat, OUTPUT);
  mcp.pinMode(fcsw, OUTPUT);
  mcp.digitalWrite(fcbat, LOW);
  mcp.digitalWrite(fcsw, LOW);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT); 
  pinMode(dcdcbat, OUTPUT);
  pinMode(dcdcfc, OUTPUT);
  pinMode(pwroff, OUTPUT);
  pinMode(pwrbtn, INPUT_PULLUP);
  digitalWrite(pwm1, LOW);
  digitalWrite(pwm2, LOW);
  digitalWrite(pwm3, LOW);
  digitalWrite(pwm4, LOW);
  digitalWrite(dcdcbat, LOW);
  digitalWrite(dcdcfc, LOW);
  digitalWrite(pwroff, LOW);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, RobotMAC); //overwrite the MAC address with a custom one
  //Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //this is the setup for the PWM channels for Motor 1-4
  ledcSetup(Motor1, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(pwm1, Motor1); //Pin attachment: Pin, PWM channel
  ledcWrite(Motor1, 0); //Pin control: PWM channel, duty cycle

  ledcSetup(Motor2, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(pwm2, Motor2); //Pin attachment: Pin, PWM channel
  ledcWrite(Motor2, 0); //Pin control: PWM channel, duty cycle
  
  ledcSetup(Motor3, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(pwm3, Motor3); //Pin attachment: Pin, PWM channel
  ledcWrite(Motor3, 0); //Pin control: PWM channel, duty cycle

  ledcSetup(Motor4, freq, resolution); //setup of PWM control: PWM channel, frequency, resolution
  ledcAttachPin(pwm4, Motor4); //Pin attachment: Pin, PWM channel
  ledcWrite(Motor4, 0); //Pin control: PWM channel, duty cycle

  rctimer = millis();
  lcdtimer = millis();
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("System");
  lcd.setCursor(4,1);
  lcd.print("online!");
  delay(1000);
  lcd.clear();
}

void loop() {

  manual = incoming.manual;
  if(manual){
    digitalWrite(dcdcfc, HIGH);
  }

  if(Chargevolt>10&&Chargevolt<32&&batvolt<13.5){
    digitalWrite(dcdcbat,HIGH);
    charging = true;
  }else if(Chargevolt<10||Chargevolt>32||batvolt>13.9){
    digitalWrite(dcdcbat,LOW);
    charging = false;
  }
  
  if(digitalRead(pwrbtn)){
    if (millis()>pwrofftimer+10){
      pwroffcounter = pwroffcounter+1;
      pwrofftimer = millis();
    }
  }else{
    pwroffcounter = 0;
  }

  if (fcstatus<1&&Chargevolt<8&&(pwroffcounter>99||signalofftimer>119)){
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("Power off");
    lcd.setCursor(6,1);
    lcd.print("in 3");
    delay(1000);
    lcd.setCursor(6,1);
    lcd.print("in 2");
    delay(1000);  
    lcd.setCursor(6,1);
    lcd.print("in 1");
    delay(1000); 
    lcd.setCursor(6,1);
    lcd.print("in 0");  
    delay(200);
    digitalWrite(pwroff, HIGH);
    delay(5000);  
  }

  for (int i = 0; i <= 3; i++) { //check the interface with the Arduino: which of the distance sensors is triggered?
    limits[i] = mcp.digitalRead(intpins[i]);
  }

  motorcounter = motorcounter+1;
  if(motorcounter>2){ //add a delay to get a smoother motor feedback
    MotorControl(); //function for motor control
    motorcounter = 0;
  }

  if (manual){
    FCmanual();
  }else{
    FCControl(); //function for fuel cell control
  }

  switch (analogcounter){ //only one analog channel is read per loop iteration to reduce the time required per loop run (reading the ADS1115 is comparably slow)
    case 0:
      outgoing.voltageFC = readAnalog(FCvoltage);
      FCvolt = double(outgoing.voltageFC)*0.0001875*(100+10)/10; //conversion of analog value to voltage, already accounting for the voltage divider
      analogcounter = 1;
      break;
    case 1:
      outgoing.H2p = readAnalog(H2pressure);
      H2press = double(outgoing.H2p)*0.0001875*(22+33)/33; //conversion of analog value to voltage, already accounting for the voltage divider
      H2press = H2press/5; //conversion of voltage to the measured relative pressure signal of the pressure sensor
      //Pressure sensor: WIKA A-10-6-BG310-NHZZ-GA-AKB-ZW (PE 81.60), which measures 0...1 bar relative pressure and provides 0...5 V as an analog output
      analogcounter = 2;
      break;
    case 2:  
      outgoing.currentFC = readAnalog(FCcurrent);
      FCcurr = double(outgoing.currentFC)*0.0001875*(22+33)/33; //conversion of analog value to voltage, already accounting for the voltage divider
      FCcurr = (FCcurr-2.47)/.185; //conversion of voltage to the measured current signal of the ACS712 (5 A version -> signal scales with 0.185 V per A) 
      if (FCcurr<0.1){
        FCcurr=0.0;
      } 
      analogcounter = 3;
      break;
    case 3:  
      outgoing.voltageBat = readAnalog(Batvoltage);
      batvolt = double(outgoing.voltageBat)*0.0001875*(100+22)/22; //conversion of analog value to voltage, already accounting for the voltage divider
      if (batvolt<12.8&&batcounter0<20){ //limit for "low battery" is 12.8 V
        batcounter0=batcounter0+1;
      }else if (batvolt>13&&batcounter0>0){
        batcounter0=batcounter0-1;
      }      
      if (batvolt<12.5&&batcounter1<20){ //limit for "drained battery" is 12.5 V
        batcounter1=batcounter1+1;
      }else if (batvolt>12.8&&batcounter1>0){
        batcounter1=batcounter1-1;
      }
      if (batcounter1>19){
        lowbat=2;
      }else if (batcounter0>19){
        lowbat=1;
      }else if(batcounter0==0){
        lowbat=0;
      }
      analogcounter = 4;
      break;
    case 4:
      outgoing.currentBat = readAnalog(Batcurrent);
      batcurr = double(outgoing.currentBat)*0.0001875*(22+33)/33; //conversion of analog value to voltage, already accounting for the voltage divider
      batcurr = (batcurr-2.48)/.100; //conversion of voltage to the measured current signal of the ACS712 (20 A version -> signal scales with 0.1 V per A)
      analogcounter = 5;
      break;
    case 5:
      outgoing.voltageCharge = readAnalog(Batcharge);  
      Chargevolt = double(outgoing.voltageCharge)*0.0001875*(100+10)/10;
      analogcounter = 0;
      break;
  }  //IMPORTANT: The voltage dividers as well as the baseline of the current sensors have been individually single-point calibrated based on readings by a multimeter or an external PSU

  if (millis()>lcdtimer+250){ //LCD update frequency: 0.25 s
    LCDupdate();
    //SerialOutput(); //the serial monitor can be used for debugging
    lcdtimer = millis();
    ledstatus=ledstatus+1; //LED blinking frequency: 0.5 s
    if (ledstatus>3){
      ledstatus=0;
    }
    if(signalquality==0){
      signalofftimer=signalofftimer+1;
    }else{
      signalofftimer=0;
    }
  }

  if (millis()>rctimer+25){ //data packages are sent 40x per second
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing)); //sending data package 20x per second
    LEDs(); //RGB LEDs are updated at the same frequency
    rctimer = millis();
  }
}

void FCmanual(){
  
  if(signalquality==0){ //force FC shutdown in case of a signal loss
    incoming.lswitch=0;
  }

  if (incoming.lswitch==0){ //left switch control whether the FC can charge the battery or not
    mcp.digitalWrite(fcbat, LOW);
  }else{
    mcp.digitalWrite(fcbat, HIGH);
  }

  if (incoming.fc==1){
    mcp.digitalWrite(fcsw, HIGH); //press FC "on" button
    fccounter = millis();
    fcstatus=1;
  }else if (millis()>fccounter+2500){
    mcp.digitalWrite(fcsw, LOW); //release FC "on" button after 2.5 s (2 s required according to manual)
    fcstatus=0;
    fccounter=0;
  }

  outgoing.fcstat = fcstatus; //report the current fuel cell status to the remote control

}

void FCControl(){
  //fcstatus==0 -> FC off, supposed to be off
  //fcstatus==1 -> left switch up, FC controller is powered on
  //fcstatus==2 -> both buttons pressed, fuel cell shall be started if the left switch is up
  //fcstatus==3 -> FC has been started
  //fcstatus==4 -> FC charges the battery
  //fcstatus==5 -> controlled FC shutdown shall be done (either both buttons pressed or left switch down)
  //fcstatus==6 -> FC shutdown due to battery fully charged
  //fcstatus==7 -> FC hydrogen supply outage
  //fcstatus==8 -> FC voltage too low

  if(signalquality==0){ //force FC shutdown in case of a signal loss
    incoming.lswitch=0;
  }

  if (fcstatus==0){
    offdelay=0;
    if(incoming.lswitch==0){
      digitalWrite(dcdcfc, LOW); //default status FC controller: switched off
    }else{
      digitalWrite(dcdcfc, HIGH); //activate FC controller with left switch
      fcstatus=1;
    }
  }

  if (fcstatus==1){
    if(incoming.lswitch==0){ //switch off FC controller with left switch
      if (offdelay>0){ //if the FC was running prior to switching off the cell, there is a 15 s delay between the command and the execution of switching off the controller
        if(millis()>offdelay+15000){
          digitalWrite(dcdcfc, LOW);
          fcstatus=0;
        }
      }else if(offdelay==0){
        digitalWrite(dcdcfc, LOW);
        fcstatus=0;
      }
    }else if(incoming.fc==1){ //start the FC by pressing both buttons
      if (H2press>0.4&&batvolt<13.5){ //start is prevented if the hydrogen pressure is low or if the battery is full
        fcstatus=2;
      }
    }
  }

  if (fcstatus==2){ 
    if (fccounter==0){
      mcp.digitalWrite(fcsw, HIGH); //press FC "on" button
      fccounter = millis();
    }else if (millis()>fccounter+2500){
      mcp.digitalWrite(fcsw, LOW); //release FC "on" button after 2.5 s (2 s required according to manual)
      fcstatus=3;
      fccounter=0;
    }
  }

  //IMPORTANT: the manufacturer recommends to run the cell for 10 min prior to adding load if the SCU is switched off - we'll try to skip that
  if (fcstatus==3){ //once the FC is started, wait for 20 s and check FC voltage
    if (fccounter==0){
      fccounter=millis();
    }else if (millis()>fccounter+20000){
      fccounter=0;
      if (FCvolt>16){ 
        fcstatus=4; 
      }else{
        fcstatus=8; //if the FC voltage is below 16 V, something is wrong (OCV should be at around 18-20 V); FC controller runs into safety shutdown at OCV<14.4V
      }
    }
  }

  if (fcstatus==4){ //charge the battery if the fuel cell runs as expected
    mcp.digitalWrite(fcbat, HIGH);
    if (H2press<0.4&&fcpresscounter<100){ //if the H2 pressure drops below 0.4 bar during load for a longer time, the FC is shut down
      fcpresscounter=fcpresscounter+1;
    }else if (H2press>0.4&&fcpresscounter>0){
      fcpresscounter=fcpresscounter-1;
    }

    if (incoming.lswitch==0||incoming.fc==0){ //switch off FC with left switch or both buttons
      fcstatus=5;
    }
    if(batvolt>13.9){ //the fuel cell is supposed to be shut down once the battery is full
      fcstatus=6; 
    }  
    if (fcpresscounter>99){ 
      fcstatus=7; 
    }
      
  }else{
    mcp.digitalWrite(fcbat, LOW); //disable the DCDC converter to the battery in all cases except "FC running as expected"
  }

  if (fcstatus>4){ //FC shutdown procedure: either by user input (5), battery full (6), or when an error occurs (7,8)
    if (fccounter==0){
      mcp.digitalWrite(fcsw, HIGH); //press fuel cell "off" button
      fccounter=millis();
    }else if (millis()>fccounter+2500){ //release fuel cell "off" button after 2.5 s (2 s needed according to manufacturer)
      mcp.digitalWrite(fcsw, LOW);
      fcstatus=1; //system back to standby (FC controller remains powered on)
      fccounter=0;
      offdelay=millis(); //delayed switch off of the FC controller when the FC has been operated
    }
  }

  outgoing.fcstat = fcstatus; //report the current fuel cell status to the remote control

}

void MotorControl(){
  motoroff = 0;
  mot1old=mot1;
  mot2old=mot2;
  mot3old=mot3;
  mot4old=mot4;    

  power = map(incoming.pwr,0,255,0,maxp); //limited to 60% of max (153/255); maxp = 153 has to be defined globally!
  angle = incoming.angle;
  rotate = map(incoming.rot,-255,255,-76,76); //limited to 20% of max (51/255)

  if (signalquality<7||lowbat==2){ //stop motors if the RC connection is lost or if the battery voltage is low
    incoming.pwr=0;
    incoming.rot=0;
    power = 0;
    rotate = 0;
  }

  if (power>0){ //stop the robot if it drives toward an obstacle; only applies to transversal movements - not to pure rotational movements
    if (limits[0]==1&&angle>0.1*pi&&angle<0.9*pi){ //front
      power = 0;
      motoroff = 1;
      rotate = 0;
    }else if (limits[1]==1&&angle>0.6*pi&&angle<1.4*pi){ //left
      power = 0;
      motoroff = 1;
      rotate = 0;
    }else if (limits[2]==1&&(angle>1.6*pi||angle<0.4*pi)){ //right
      power = 0;
      motoroff = 1;
      rotate = 0;
    }else if (limits[3]==1&&angle>1.1*pi&&angle<1.9*pi){ //rear
      power = 0;
      motoroff = 1;
      rotate = 0;
    }
  }

  //motor control is adapted from https://compendium.readthedocs.io/en/latest/tasks/drivetrains/mecanum.html
  if(angle<0.5*pi){ //translate input power & angle into motor speeds
    motad=power*sqrt(2)*0.5*(sin(angle)+cos(angle)-0.4*sin(angle*2));
    motbc=power*sqrt(2)*0.5*(sin(angle)-cos(angle));
  }else if (angle>=0.5*pi&&angle<pi){
    motad=power*sqrt(2)*0.5*(sin(angle)+cos(angle));
    motbc=power*sqrt(2)*0.5*(sin(angle)-cos(angle)+0.4*sin(angle*2));
  }else if (angle>=pi&&angle<1.5*pi){
    motad=power*sqrt(2)*0.5*(sin(angle)+cos(angle)+0.4*sin(angle*2));
    motbc=power*sqrt(2)*0.5*(sin(angle)-cos(angle));    
  }else{
    motad=power*sqrt(2)*0.5*(sin(angle)+cos(angle));
    motbc=power*sqrt(2)*0.5*(sin(angle)-cos(angle)-0.4*sin(angle*2));   
  }

  if (abs(motad)+abs(rotate)>double(0.7071*maxp)){ //correct the motor speed for the requested rotation speed
    corrad=double(abs(motad)+abs(rotate))/double(0.7071*maxp); //(0.7071 times the maximum specified power is the peak output of the sin/cos drive functions)
  }else{
    corrad=double(1);
  } 
  if (abs(motbc)+abs(rotate)>double(0.7071*maxp)){
    corrbc=double(abs(motbc)+abs(rotate))/double(0.7071*maxp);
  }else{
    corrbc=double(1);
  }
  if (corrad>corrbc){
    corr=corrad;
  }else{
    corr=corrbc;
  }

  mot1=(double(motbc)+double(rotate))/corr; //these are the final motor speeds (between +-0.7071*maxp)
  mot2=(double(motad)-double(rotate))/corr;
  mot3=(double(motbc)-double(rotate))/corr;
  mot4=(double(motad)+double(rotate))/corr;

  mot1=speedinterpolate(mot1,mot1old); //the interpolation code prevents abrupt speed changes
  mot2=speedinterpolate(mot2,mot2old);
  mot3=speedinterpolate(mot3,mot3old);
  mot4=speedinterpolate(mot4,mot4old);

  if (mot1<0){ //specify motor drive directions
    mcp.digitalWrite(dir1, HIGH);
  }else{
    mcp.digitalWrite(dir1, LOW);
  }
  if (mot2<0){
    mcp.digitalWrite(dir2, LOW);
  }else{
    mcp.digitalWrite(dir2, HIGH);
  }
  if (mot3<0){
    mcp.digitalWrite(dir3, HIGH);
  }else{
    mcp.digitalWrite(dir3, LOW);
  }
  if (mot4<0){
    mcp.digitalWrite(dir4, HIGH);
  }else{
    mcp.digitalWrite(dir4, LOW);
  }

  ledcWrite(Motor1,abs(mot1)); //power the motors according to the determined speed and angle
  ledcWrite(Motor2,abs(mot2));
  ledcWrite(Motor3,abs(mot3));
  ledcWrite(Motor4,abs(mot4));

}

int speedinterpolate(int newval, int oldval){ //the interpolation algorithm limits the speed change rate to reduce the maximum acceleration/torque acting on the mechanics
  if((newval-oldval)>12){
    newval=oldval+5;
  }else if((newval-oldval)>5){
    newval=oldval+2;
  }else if ((newval-oldval)>0){
    newval=oldval+1;
  }
  if((newval-oldval)<-12){
    newval=oldval-5;
  }else if((newval-oldval)<-5){
    newval=oldval-2;
  }else if ((newval-oldval)<0){
    newval=oldval-1;
  }
  return(newval);
}

void LEDs(){

  //bottom LED control (LEDs 0-23): drive segments
  if (signalquality>6&&lowbat<2){ //default status: connection to RC ready
    //default color: green
    for (int i=0; i<RGBcount/2; i++){
      strip.setPixelColor(i,0,127,0);
    }

    //obstacle detected: turn segment yellow
    if (limits[0]==1){ //front
      for (int i=12; i<=17; i++){ //segments 5+6
        strip.setPixelColor(i,127,127,0);
      }
    }
    if (limits[1]==1){ //left
      for (int i=18; i<=23; i++){ //segments 7+8
        strip.setPixelColor(i,127,127,0);
      }
    }
    if (limits[2]==1){ //right
      for (int i=6; i<=11; i++){ //segments 3+4
        strip.setPixelColor(i,127,127,0);
      }
    }
    if (limits[3]==1){ //rear
      for (int i=0; i<=5; i++){ //segments 1+2
        strip.setPixelColor(i,127,127,0);
      }
    }

    //LED control when driving
    if (power>0||motoroff==1){  //manual as needed
      if (angle>=0&&angle<0.375*pi||angle>=1.875*pi){ //LED segment 4 (right-front)
        for (int i=9; i<=11; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127); //indicate active drive direction with white light
          }else{
            strip.setPixelColor(i,127,0,0); //indicate detected obstacle in drive direction with red light
          }
        }
      }
      if (angle>=0.125*pi&&angle<0.625*pi){ //LED segment 5 (front-right)
        for (int i=12; i<=14; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      }
      if (angle>=0.375*pi&&angle<0.875*pi){ //LED segment 6 (front-left)
        for (int i=15; i<=17; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      }  
      if (angle>=0.625*pi&&angle<1.125*pi){ //LED segment 7 (left-front)
        for (int i=18; i<=20; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      }  
      if (angle>=0.875*pi&&angle<1.375*pi){ //LED segment 8 (left-rear)
        for (int i=21; i<=23; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      } 
      if (angle>=1.125*pi&&angle<1.625*pi){ //LED segment 1 (rear-left)
        for (int i=0; i<=2; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      }  
      if (angle>=1.375*pi&&angle<1.875*pi){ //LED segment 2 (rear-right)
        for (int i=3; i<=5; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      } 
      if (angle>=1.625*pi&&angle<=2*pi||angle<0.125*pi){ //LED segment 3 (right-rear)
        for (int i=6; i<=8; i++){
          if (motoroff==0){
            strip.setPixelColor(i,127,127,127);
          }else{
            strip.setPixelColor(i,127,0,0);
          }
        }
      } 
    }
  }else if(lowbat<2){ //blink bottom segments in red if signal is lost
    if (ledstatus<2){
      for (int i=0; i<RGBcount/2; i++){ 
        strip.setPixelColor(i,127,0,0);       
      }   
    }else{
      for (int i=0; i<RGBcount/2; i++){ 
        strip.setPixelColor(i,0,0,0);  
      }   
    }
  }else{
    for (int i=0; i<RGBcount/2; i++){ 
      strip.setPixelColor(i,0,0,0);  
    }     
  }
    
  //top LED segments (LEDs 24-47): battery & fuel cell status
  if (lowbat==2){
    if (ledstatus<2){ //blinking red: motors disabled due to battery low
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,127,0,0);
      }   
    }else{
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,0,0,0);
      }   
    } 
  }else if(fcstatus>6){ 
    if (ledstatus<2){ //blinking purple: battery ok, but FC malfunction
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,127,0,127);
      }   
    }else{
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,0,0,0);
      }   
    }     
  }else if(fcstatus==0){
    if (lowbat==0&&!charging){ //green: running on battery, battery high
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        if (manual){
          if (incoming.lswitch==0){
            strip.setPixelColor(i,0,127,0);
          }else{
            strip.setPixelColor(i,0,0,127);
          }
        }else{
          strip.setPixelColor(i,0,127,0);
        }
      }   
    }else if(charging){ //blue: charging from external DC connector
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,0,0,127);
      }        
    }else{ //red: running on battery, battery low
      for (int i=RGBcount/2; i<RGBcount; i++){ 
        strip.setPixelColor(i,127,0,0);
      }       
    }        
  }else if(fcstatus==1){ //cyan: FC controller online, ready to start the cell
    for (int i=RGBcount/2; i<RGBcount; i++){ 
      strip.setPixelColor(i,0,127,127);
    }       
  }else if(fcstatus==4){  //blue: charging battery with FC
    for (int i=RGBcount/2; i<RGBcount; i++){ 
      strip.setPixelColor(i,0,0,127);
    }       
  }else{ //yellow: FC status change (fcstatus 2, 3, 5, 6)
    for (int i=RGBcount/2; i<RGBcount; i++){ 
      strip.setPixelColor(i,127,127,0);
    }     
  }

  strip.show();

}

void LCDupdate(){
  if (lcdcounter<80){
    lcdcounter = lcdcounter + 1;
  }else{
    lcdcounter = 0;
  }
  if(lcdcounter==0||lcdcounter==20||lcdcounter==40||lcdcounter==60){
    lcd.clear();
  }
  lcd.setCursor(0,0);
  if (lcdcounter<20){
    lcd.print("Battery");
    if (lowbat==2){
      lcd.print(" low!");
    }else{
      lcd.print("     ");
    }
    lcd.setCursor(0,1);
    lcd.print(batvolt,1);
    lcd.print("V, ");
    lcd.print(batcurr,1);
    lcd.print("A    ");
  }else if(lcdcounter<40){
    if(manual){
      lcd.print("Fuel cell (man.)");
      lcd.setCursor(0,1);
      if(incoming.lswitch==0){
        lcd.print("relay off  ");
      }else{
        lcd.print("relay on   ");
      }
    }else{
      lcd.print("Fuel cell (auto)");
      lcd.setCursor(0,1);      
      if(fcstatus==0){
        lcd.print("offline    ");
      }else if(fcstatus==1){
        lcd.print("standby    ");
      }else if(fcstatus==2){
        lcd.print("start      ");
      }else if(fcstatus==3){
        lcd.print("start      ");
      }else if(fcstatus==4){
        lcd.print("running    ");
      }else if(fcstatus==5){
        lcd.print("shutdown   ");
      }else if(fcstatus==6){
        lcd.print("shutdown   ");
      }else if(fcstatus==7){
        lcd.print("H2 is low  ");
      }else if(fcstatus==8){
        lcd.print("OCV too low");
      }
    }
  }else if(lcdcounter<60){
    lcd.print("FC: ");
    lcd.print(FCvolt,1);
    lcd.print("V, ");
    lcd.print(FCcurr,1);
    lcd.print("A ");
    lcd.setCursor(0,1);
    lcd.print("H2: ");
    lcd.print(H2press,2);
    lcd.print("bar");
  }else if(lcdcounter<80){
    lcd.print("Charge port ");
    lcd.print(Chargevolt,0);
    lcd.print("V ");
    lcd.setCursor(2,1);
    if (charging){
      lcd.print("  charging  ");
    }else{
      lcd.print("not charging");
    }
  }
}

int readAnalog(int InputChannel){ //this code unifies the readout of the analog channels of the two ADS1115
  int val = 0;
  if (InputChannel<4){
    val = adc1.readADC_SingleEnded(InputChannel);
  }else{
    val = adc2.readADC_SingleEnded(InputChannel-2); //InputChannel4&InputChannel5 belong to inputs 2 and 3 of the 2nd ADS1115
  }
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}

//below this line is optional code used for debugging

void SerialOutput(){
  Serial.print("Echo 1 (front) limit: ");
  Serial.println(limits[0]);
  Serial.print("Echo 2 (left) limit: ");
  Serial.println(limits[1]);
  Serial.print("Echo 3 (right) limit: ");
  Serial.println(limits[2]);
  Serial.print("Echo 4 (rear) limit: ");
  Serial.println(limits[3]);
  Serial.print("FC current: ");
  Serial.println(FCcurr);

  if(limits[0]==1&&limits[1]==1&&limits[2]==1&&limits[3]==1){ //all sides blocked
    Serial.println("all sides blocked");
  }else if(limits[0]==0&&limits[1]==1&&limits[2]==1&&limits[3]==1){ //only front free, rest blocked
    Serial.println("only front free, rest blocked");
  }else if(limits[0]==1&&limits[1]==0&&limits[2]==1&&limits[3]==1){ //only left free, rest blocked
    Serial.println("only left free, rest blocked");
  }else if(limits[0]==1&&limits[1]==1&&limits[2]==0&&limits[3]==1){ //only right free, rest blocked
    Serial.println("only right free, rest blocked");
  }else if(limits[0]==1&&limits[1]==1&&limits[2]==1&&limits[3]==0){ //only rear free, rest blocked
    Serial.println("only rear free, rest blocked");
  }else if(limits[0]==0&&limits[1]==0&&limits[2]==1&&limits[3]==1){ //front, left free, rest blocked
    Serial.println("front and left free, rest blocked");
  }else if(limits[0]==1&&limits[1]==0&&limits[2]==0&&limits[3]==1){ //left, right free, rest blocked
    Serial.println("left and right free, rest blocked");
  }else if(limits[0]==1&&limits[1]==1&&limits[2]==0&&limits[3]==0){ //right, rear free, rest blocked
    Serial.println("right and rear free, rest blocked");
  }else if(limits[0]==0&&limits[1]==1&&limits[2]==0&&limits[3]==1){ //front, right free, rest blocked
    Serial.println("front and right free, rest blocked");
  }else if(limits[0]==1&&limits[1]==0&&limits[2]==1&&limits[3]==0){ //left, rear free, rest blocked
    Serial.println("left and rear free, rest blocked");
  }else if(limits[0]==0&&limits[1]==1&&limits[2]==1&&limits[3]==0){ //front, rear free, rest blocked
    Serial.println("front and rear free, rest blocked");
  }else if(limits[0]==1&&limits[1]==0&&limits[2]==0&&limits[3]==0){ //only front blocked
    Serial.println("only front blocked");
  }else if(limits[0]==0&&limits[1]==1&&limits[2]==0&&limits[3]==0){ //only left blocked
    Serial.println("only left blocked");
  }else if(limits[0]==0&&limits[1]==0&&limits[2]==1&&limits[3]==0){ //only right blocked
    Serial.println("only right blocked");
  }else if(limits[0]==0&&limits[1]==0&&limits[2]==0&&limits[3]==1){ //only rear blocked
    Serial.println("only rear blocked");
  }else{ //no side blocked
    Serial.println("all free - no sides blocked");
  }

  Serial.println("----------");
}