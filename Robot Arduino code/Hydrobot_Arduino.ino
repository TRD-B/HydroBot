// pins for trigger
int trig1 = 4; //front
int trig2 = 2; //left
int trig3 = 8; //right
int trig4 = 6; //rear
const int trigpins[] = {trig1, trig2, trig3, trig4};

// pins for echo
int echo1 = 5;
int echo2 = 3;
int echo3 = 9;
int echo4 = 7;
const int echopins[] = {echo1, echo2, echo3, echo4};

// pins for interface

int int1 = A0;
int int2 = A1;
int int3 = A2;
int int4 = A3;
const int intpins[] = {int1, int2, int3, int4};

// Variable für die Speicherung der Entfernung
unsigned long durations[] = {0, 0, 0, 0};
int distances[] = {0, 0, 0, 0};
int mindist[] = {25, 35, 35, 25};   //in cm; left & right are larger because of the geometry (sensors behind mecanum wheels)


void setup() 
{
  for (int i = 0; i <= 3; i++) {
    pinMode(trigpins[i], OUTPUT);
    digitalWrite(trigpins[i], LOW);
    pinMode(intpins[i], OUTPUT);
    digitalWrite(intpins[i], LOW);
    pinMode(echopins[i], INPUT);
  }

  // Seriellen Monitor starten
  Serial.begin(115200);
}

void loop() 
{
  for (int i = 0; i <= 3; i++) {
    digitalWrite(trigpins[i], LOW);
    delay(2); //short switch-off of sender to avoid measurement artifacts
    digitalWrite(trigpins[i], HIGH);
    delayMicroseconds(10); //short high-pulse to send a "ping"
    digitalWrite(trigpins[i], LOW);

    durations[i] = pulseIn(echopins[i], HIGH, 10000); //measuring time to signal, with a timeout of 10 ms (10.000 microseconds), around 1.7 m
    distances[i] = (durations[i]/2) * 0.03432; //conversion to cm

    Serial.print("Distance #");
    Serial.print(i+1);
    if (durations[i] > 0) 
    {
      Serial.print(" in cm: ");
      Serial.println(distances[i]);
      if(distances[i]>0&&distances[i]<mindist[i]){
        digitalWrite(intpins[i], HIGH);
      }else{
        digitalWrite(intpins[i], LOW);
      }
    }else{
      Serial.println(": too large/error");
      digitalWrite(intpins[i], LOW);
    }
  }

  Serial.println("----------");
  //delay(500);

}

