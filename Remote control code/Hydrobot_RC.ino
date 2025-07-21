//include libraries for esp wireless connection, external analog/digital converter and LCD
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_ADS1X15.h> //library for ADS1115 usage
#include <LiquidCrystal_I2C.h>
#include <esp_wifi.h> //needed to adjust the MAC address

//pin assignment
#define btnl 25
#define btnr 26
#define lsw 23
#define rsw 32

//create global variables
int btnldebounce0 = 0;
int btnldebounce1 = 0;
int btnlv = 1;
int btnrdebounce0 = 0;
int btnrdebounce1 = 0;
int btnrv = 1;
int joyrxv = 0;
int joyryv = 0;
int joylv = 0;
double batvolt = 0;
int lcdtimer = 0;
int rctimer = 0;
double robotbat = 0;
double current = 0;
double angle = 0;
double radius = 0;
double rmax = 0;
double pi = 3.141592654;
double FCvolt = 0;
double FCcurr = 0;
double H2press = 0;
int counter = 0;
int fcstatus = 0;
int fcstatus_old = 0;
int screen = 0;
int screen_old = 0;
int batlow = 0;
int changetimer = 10;
bool manual = false; //change FC control mode: false = auto; true = manual

//Add I2Cperipherals
Adafruit_ADS1115 adc1;
int joyrx = 3; //Right joystick up/down
int joyry = 2; //Right joystick left/right
int joyl = 1; //Left joystick left/right
int batt = 0; //Battery voltage

LiquidCrystal_I2C lcd(0x27, 20, 4); //the display is a 4-line 20-character LCD display

//ESPNOW setup is based on the project by Random Nerd Tutorials (with some modifications as needed for this project).
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
//MAC addresses: The communication is established between two ESP32 based on their MAC addresses
uint8_t RCMAC[] = {0xA0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address as the address of the remote control
uint8_t broadcastAddress[] = {0xAE, 0xA0, 0xBE, 0xEF, 0xFE, 0xED}; //this is the MAC address of Hydrobot
int signalquality = 0;

//Definition of the data package that is sent to the other ESP32 and creation of such a structure
struct send_message {
  int pwr;
  double angle;
  int rot;
  int fc;
  int lswitch;
  bool manual;  
};
struct send_message outgoing;
//Definition of the data package that is received from the other ESP32 and creation of such a structure
struct receive_message {
  int voltageFC;
  int voltageBat;
  int currentFC;
  int currentBat;
  int H2p;
  int fcstat;
  int voltageCharge;
};
struct receive_message incoming;
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
 
//Interrupt functions for reading buttons
void IRAM_ATTR btnlint(){
  btnldebounce1 = millis();
  if (btnldebounce1>btnldebounce0+500){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btnlv = 0;    
  }
  btnldebounce0 = btnldebounce1;
}

void IRAM_ATTR btnrint(){
  btnrdebounce1 = millis();
  if (btnrdebounce1>btnrdebounce0+500){
    btnrv = 0;    
  }
  btnrdebounce0 = btnrdebounce1;
}

void setup() {

  Serial.begin(115200);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, RCMAC); //overwrite the MAC address with a custom one
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

  //specify input/output pins
  pinMode(lsw, INPUT_PULLUP);
  pinMode(rsw, INPUT_PULLUP);
  pinMode(btnl, INPUT_PULLUP);
  pinMode(btnr, INPUT_PULLUP);
  attachInterrupt(btnl,btnlint,FALLING);
  attachInterrupt(btnr,btnrint,FALLING);

  //start the LCD display
  lcd.init();  // initialize the lcd   
  lcd.backlight();
  adc1.begin();
  adc1.setDataRate(RATE_ADS1115_860SPS);
  Wire.setClock(400000);

  lcd.setCursor(0,0); //Defining positon to write from first row, first column .
  lcd.print("********************");
  lcd.setCursor(5,1); //Second row, first column
  lcd.print("HydrObot"); 
  lcd.setCursor(0,2); //Third row, first column
  lcd.print("powered by hydrogen"); 
  lcd.setCursor(0,3); //Fourth row, first column
  lcd.print("********************");
  lcdtimer = millis();
  while(millis()<lcdtimer+2000){
    if(digitalRead(btnr)<1){
      manual=true;
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("FC mode: manual");
      delay(1000);
    }
  }
  outgoing.manual=manual;
  lcd.clear(); //clear the whole LCD

  lcdtimer = millis();
  rctimer  = lcdtimer;

  if (digitalRead(lsw)==1||digitalRead(rsw)==1){ //the robot does only start when the switch position does not result in a powering of the FC controller
    lcd.setCursor(0,0);
    lcd.print("Move switches down");
    lcd.setCursor(0,1);
    lcd.print("to proceed!");
    while(digitalRead(lsw)==1||digitalRead(rsw)==1){
      lcd.setCursor(counter,3);
      lcd.print(".");
      if (counter>9){
        lcd.setCursor(counter-10,3);
        lcd.print(" ");
      }else{
        lcd.setCursor(counter+10,3);
        lcd.print(" ");
      }
      if (counter<19){     
        counter = counter+1;
      }else{
        counter=0;
      }
      delay(250);
    }
    delay(250);
    lcd.clear();
  }

}
 
void loop() {

  readJoysticks();
  if(manual){
    switchmanual();
  }else{
    readSwitches();
  }

  //read analog sensors, convert counts and inputs from the robot into the correct units
  batvolt = readAnalog(batt); //battery voltage of the remote control
  batvolt = double(batvolt)*0.0001875*(47+22)/22; //voltage calculation: 1 count equals 0.1875 mV with default settings of ADS1115; voltage divider with 47 and 22 kOhm
  if (batvolt<6.9){ //a voltage less than 1.15 V / cell is considered as fully discharged -> threshold is indicated in the LCD screen
    batlow = 1;
  }
  robotbat = incoming.voltageBat;
  robotbat = double(robotbat)*0.0001875*(100+22)/22;
  current = double(incoming.currentBat)*0.0001875*(22+33)/33; //current draw of the car: Analog value of the ACS712 sensor, corrected for its voltage divider with 22 and 33 kOhm
  current = (current-2.48)/.100; //conversion of voltage to the measured current signal of the ACS712 (20 A version -> signal scales with 0.1 V per A)
  FCvolt = double(incoming.voltageFC)*0.0001875*(100+10)/10; //conversion of analog value to voltage, already accounting for the voltage divider
  H2press = double(incoming.H2p)*0.0001875*(22+33)/33; //conversion of analog value to voltage, already accounting for the voltage divider
  H2press = H2press/5; //conversion of voltage to the measured relative pressure signal of the pressure sensor
  FCcurr = double(incoming.currentFC)*0.0001875*(22+33)/33; //conversion of analog value to voltage, already accounting for the voltage divider
  FCcurr = (FCcurr-2.47)/.185; //conversion of voltage to the measured current signal of the ACS712 (5 A version -> signal scales with 0.185 V per A)
  if(FCcurr<0.1){
    FCcurr=0.0;
  }   
  //IMPORTANT: The voltage dividers as well as the baseline of the current sensors have been individually single-point calibrated based on readings by a multimeter or an external PSU

  if (millis()>lcdtimer+200){ //the LCD display is updated 10x per second
    LCDselect();
    //updateSerial(); //serial output for debugging
    lcdtimer = millis();
  }

  if (millis()>rctimer+25){ //wireless communication sends packages every 25 ms (40x per second)
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing)); //sending data package 20x per second
    rctimer = millis();
  }

}

void LCDselect(){

  if(changetimer<10){
    changetimer=changetimer+1;
    if(changetimer==10){
      lcd.clear();
    }
  }

  fcstatus_old = fcstatus;
  fcstatus = incoming.fcstat;
  if(fcstatus_old==fcstatus&&changetimer>9){
    if (screen!=screen_old){
      lcd.clear(); //whenever the page of the LCD screen is flipped, the screen is cleared once
    }
    if (screen==0){
      LCDupdate();
    }else{
      LCD1update();
    }
    screen_old=screen;
  }else if(fcstatus_old!=fcstatus){
    changetimer=0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("FC status:        ");
    lcd.setCursor(0,1);
    switch (fcstatus){
      case 0:
        if(manual){
          lcd.setCursor(0,1);
          lcd.print("Releasing ON button");
        }else{
          lcd.print("Offline            ");
          lcd.setCursor(0,2);
          lcd.print("(FC controller is  ");
          lcd.setCursor(0,3);
          lcd.print("powered off)       ");
        }       
        break;
      case 1:
        if(manual){
          lcd.setCursor(0,1);
          lcd.print("Pressing ON button ");
        }else{
          lcd.print("Standby            ");
          lcd.setCursor(0,2);
          lcd.print("(FC controller is  ");
          lcd.setCursor(0,3);
          lcd.print("powered on)        ");
        }
        break;
      case 2:
        lcd.print("Requesting start...");
        lcd.setCursor(0,2);
        lcd.print("(Telling FC Con to ");
        lcd.setCursor(0,3);
        lcd.print("start the stack)   ");        
        break;
      case 3:
        lcd.print("Starting cell...   ");
        lcd.setCursor(0,2);
        lcd.print("(Waiting for FC to ");
        lcd.setCursor(0,3);
        lcd.print("reach power)       ");             
        break;
      case 4:
        lcd.print("Online             ");
        lcd.setCursor(0,2);
        lcd.print("(Charging the      ");
        lcd.setCursor(0,3);
        lcd.print("battery)           ");           
        break;
      case 5:
        lcd.print("Shutting down...   ");
        lcd.setCursor(0,2);
        lcd.print("(Telling FC Con to ");
        lcd.setCursor(0,3);
        lcd.print("stop the stack)    ");         
        break;
      case 6:
        lcd.print("Battery full       ");  
        lcd.setCursor(0,2);
        lcd.print("(Auto-shutdown of  ");
        lcd.setCursor(0,3);
        lcd.print("stack initiated)   ");          
        break;
      case 7:
        lcd.print("H2 pressure low    ");
        lcd.setCursor(0,2);
        lcd.print("(FC must not be    ");
        lcd.setCursor(0,3);
        lcd.print("further operated)  ");         
        break;
      case 8:
        lcd.print("Voltage low        ");
        lcd.setCursor(0,2);
        lcd.print("(FC does not reach ");
        lcd.setCursor(0,3);
        lcd.print("required voltage)  ");         
        break;  
    }
  }
}

void switchmanual(){
  if (btnlv==0||btnrv==0){ //only if both buttons are pressed at the same time, the FC starts/stops
    delay(250);
    if (btnlv==0&&btnrv==0){
      outgoing.fc=1;
    }
    btnlv=1;
    btnrv=1;
  }

  if (incoming.fcstat==1){ //reset output variable
    outgoing.fc=0;
  }

  outgoing.lswitch=digitalRead(lsw); //the left switch defines whether the FC charges the battery or not

  screen=digitalRead(rsw); //the right switch changes the page of the LCD screen  
}

void readSwitches(){ //function that reads switches and buttons
  if (btnlv==0||btnrv==0){ //only if both buttons are pressed at the same time, the FC starts/stops
    delay(250);
    if (btnlv==0&&btnrv==0){
      if (incoming.fcstat==1){
        if(H2press<0.4){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("H2 pressure low!   ");
          lcd.setCursor(0,2);
          lcd.print("FC start denied.    ");
          delay(2000);
          lcd.clear();   
        }else if(robotbat>13.4){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Battery full!       ");
          lcd.setCursor(0,2);
          lcd.print("FC start denied.    ");
          delay(2000);
          lcd.clear();  
        }else{
          outgoing.fc=1;
        }
      }else{
        outgoing.fc=0;
      }
    }
    btnlv=1;
    btnrv=1;
  }

  if(incoming.fcstat>5){ //avoids auto-restarting the fuel cell if an error occurs
    outgoing.fc=0;
  }

  outgoing.lswitch=digitalRead(lsw); //the left switch defines whether the FC controller is powered on or not
  if (outgoing.lswitch==0){ //the FC is switched off if the FC controller switch is moved to "off" during operation
    outgoing.fc=0;
  }

  screen=digitalRead(rsw); //the right switch changes the page of the LCD screen

}

void readJoysticks(){ //read joystick inputs, convert input into polar coordinates, and prepare outputs for robot
  joyrxv = readAnalog(joyrx); //forward & reverse
  joyrxv = map(joyrxv, 0, 17500, -255, 255);
   if (abs(joyrxv)<10){
     joyrxv=0;
   }
  joyryv = readAnalog(joyry); // left & right
  joyryv = map(joyryv, 0, 17500, -255, 255);
   if (abs(joyryv)<10){
     joyryv=0;
   }
  joylv = readAnalog(joyl); //rotation
  joylv = map(joylv, 0, 17500, -255, 255);
  if (abs(joylv)<10){
    joylv=0;
  }

  if (joyryv!=0){ //get the angle based on the quadrant of the right joystick: https://www.mathsisfun.com/polar-cartesian-coordinates.html#summary
    if (joyryv>0&&joyrxv>0){
      angle = atan(double(joyrxv)/double(joyryv));
    }else if (joyryv<0){
      angle = atan(double(joyrxv)/double(joyryv))+pi;
    }else{
      angle = atan(double(joyrxv)/double(joyryv))+2*pi;
    }
  }else{
    if(joyrxv>0){
      angle = 0.5*pi;
    }else{
      angle = 1.5*pi;
    }
  }

  radius = sqrt(double(joyrxv)*double(joyrxv)+double(joyryv)*double(joyryv)); //get the maximum possible radius of the current angle: https://www.mathsisfun.com/polar-cartesian-coordinates.html#summary
  if (angle<0.25*pi||angle>1.75*pi){
    rmax = 255/cos(angle); 
  }else if (angle>=0.25*pi&&angle<=0.75*pi){
    rmax = 255/sin(angle);
  }else if (angle>0.75*pi&&angle<1.25*pi){
    rmax = -255/cos(angle);
  }else if (angle>=1.25*pi&&angle<=1.75*pi){
    rmax = -255/sin(angle);
  }

  outgoing.pwr = int((radius/rmax)*255); //radius relative to maximum possible radius
  outgoing.angle = angle;
  outgoing.rot = joylv;

}

void LCD1update(){ //LCD screen page 2: FC status
  lcd.setCursor(0,0);

  if(manual){
    lcd.print("FC status (manual):");
    lcd.setCursor(0,1);
    if(outgoing.lswitch==0){
      lcd.print("Relay off");
    }else{
      lcd.print("Relay on ");
    }
  }else{
    lcd.print("FC status (auto):");
    lcd.setCursor(0,1);
    switch (fcstatus){
      case 0:
        lcd.print("Offline            ");
        break;
      case 1:
        lcd.print("Standby            ");
        break;
      case 2:
        lcd.print("Requesting start...");
        break;
      case 3:
        lcd.print("Starting cell...   ");
        break;
      case 4:
        lcd.print("Online             ");
        break;
      case 5:
        lcd.print("Shutting down...   ");
        break;
      case 6:
        lcd.print("Battery full       ");  
        break;
      case 7:
        lcd.print("H2 pressure low    ");
        break;
      case 8:
        lcd.print("OCV is too low     ");
        break;  
    }
  }


  if (signalquality>6){
    lcd.setCursor(0,2); 
    if (FCvolt>10){
      lcd.print(FCvolt,1);
      lcd.print("V, ");   
    }else if (FCvolt>0){
      lcd.print(FCvolt,1);
      lcd.print("V,  ");
    }else{
      lcd.print("0.0V,  ");
    }
    lcd.print(FCcurr,1);
    lcd.print("A, ");
    if (FCvolt*FCcurr>10){
      lcd.print(FCvolt*FCcurr,1);
      lcd.print ("W");  
    }else if (FCvolt*FCcurr>0){
      lcd.print(FCvolt*FCcurr,1);
      lcd.print ("W ");      
    }else{
      lcd.print ("0.0W ");     
    }

    lcd.setCursor(0,3);
    lcd.print("H2: ");
    if (H2press>0){
      lcd.print(H2press,2);
      lcd.print("bar");
    }else{
      lcd.print("0.00bar");
    }
  }else{
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("  (no connection!)  ");
  }

}

void LCDupdate(){ //LCD screen page 1: Robot status
  lcd.setCursor(0,0);
  lcd.print("TX ");
  lcd.print(batvolt,1);
  lcd.print("V");
  lcd.setCursor(10,0);
  lcd.print("RX ");
  if (signalquality>6){
    lcd.print(robotbat,1);
    lcd.print("V ");
    lcd.setCursor(10,2);
    lcd.print("FC ");
    if (FCvolt*FCcurr>10){
      lcd.print(FCvolt*FCcurr,1);
      lcd.print ("W");  
    }else if (FCvolt*FCcurr>0){
      lcd.print(FCvolt*FCcurr,1);
      lcd.print ("W ");      
    }else{
      lcd.print ("0.0W ");     
    }
  }else{
    lcd.print("no cxn");
    lcd.setCursor(10,2);
    lcd.print("        ");
  }
  lcd.setCursor(0,1);
  if (batlow==1){
    lcd.print("battery");
  }else{
    lcd.print("Spd ");
    lcd.print(outgoing.pwr);
    lcd.print("  ");
  }
  lcd.setCursor(13,1);
  if (signalquality>6){
    lcd.print(current,1);
    lcd.print("A  ");
    if(current>0){
      lcd.print(" ");
    }
    lcd.setCursor(10,3);
    lcd.print("H2 ");
    if (H2press>0){
      lcd.print(H2press,2);
      lcd.print("bar");
    }else{
      lcd.print("0.00bar");
    }
  }else{
    lcd.print("       ");
    lcd.setCursor(10,3);
    lcd.print("          ");
  }
  lcd.setCursor(0,2);
  if (batlow==1){
    lcd.print("low!   ");
    lcd.setCursor(0,3);
    lcd.print("        ");
  }else{
    lcd.print("Ang ");
    if (outgoing.pwr>0){
      lcd.print(angle*180/pi,0);
    }else{
      lcd.print("0");
    }
    lcd.print("  ");
    lcd.setCursor(0,3);
    lcd.print("Rot ");
    lcd.print(outgoing.rot);
    if (abs(outgoing.rot)<100){
      lcd.print("   ");
    }else{
      lcd.print("  ");
    }
  }


}

 void updateSerial(){
  // Serial.print("Right joystick X: ");
  // Serial.println(joyrxv);
  // Serial.print("Right joystick Y: ");
  // Serial.println(joyryv);
  // Serial.print("Left joystick: ");
  // Serial.println(joylv);
  // Serial.print("Angle in radians: ");
  // Serial.println(angle);
  // Serial.print("Angle in degree: ");
  // Serial.println(angle*180/pi);
  // Serial.print("Relative Magnitude: ");
  // Serial.println((radius/rmax)*255,0);
  // Serial.print("Sine of 180°: ");
  // Serial.println(sin(0.5*pi));
  Serial.print("Left button: ");
  Serial.println(btnlv);
  Serial.print("Right button: ");
  Serial.println(btnrv);
  Serial.print("outgoing.fc: ");
  Serial.println(outgoing.fc);
  Serial.println("----------");
 }

int readAnalog(int InputChannel){
  int val = 0;
  val = adc1.readADC_SingleEnded(InputChannel);
  if (val>17500){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17500;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}
