#include <Wire.h>
#include <Time.h>
#include <PID_v1.h>

//these lines are for setting ramp. period is the time between increases in temperature.
unsigned long periodt = 600000; //calculated as ms, so 1000ms = 1s
unsigned long perioda = 300000; //calculated as ms, so 1000ms = 1s
unsigned long co2_time = 120000; //waiting time between start of the experiment and the first temp increase.

#define histeresis 0.1 // this defines how close it gets to setpoint.

boolean change;
boolean ramp;
boolean heating;
boolean counter;

//Set pins from monster
//direction of one side
#define CWA  7
#define CCWA 8
#define PWMA 5
//direction of the other one
#define CWB  4
#define CCWB 9
#define PWMB 6

//enable pins
#define EN_PIN_A A0
#define EN_PIN_B A1

//setting of thermistor
#define SERIESRESISTOR 10000
#define NOMINAL_RESISTANCE 10000
#define NOMINAL_TEMPERATURE 25
#define BCOEFFICIENT 3950


double setpoint1, setpoint2, Average, Input, OutputE, OutputC, OutputD, OutputF, temp1, temp2;
unsigned long init_time, interval, initial_time, currentMillis, previousMillis;
int EN_PIN_1, EN_PIN_2;
PID myPID1(&temp1, &OutputC, &setpoint1,10,40,5, DIRECT);
PID myPID2(&temp1, &OutputE, &setpoint1,10,40,5, REVERSE);
PID myPID3(&temp2, &OutputD, &setpoint2,10,40,5, DIRECT);
PID myPID4(&temp2, &OutputF, &setpoint2,10,40,5, REVERSE);


void setup(){
  pinMode(EN_PIN_A, OUTPUT);
  pinMode(EN_PIN_B, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(CWA, OUTPUT);
  pinMode(CWB, OUTPUT);
  pinMode(CCWA, OUTPUT);
  pinMode(CCWB, OUTPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(10,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);

  digitalWrite(EN_PIN_A, HIGH);
  digitalWrite(EN_PIN_B, HIGH);
  digitalWrite(10,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(2,LOW);
  myPID1.SetOutputLimits(0,200);
  myPID1.SetMode(MANUAL);
  myPID2.SetOutputLimits(0,200); 
  myPID2.SetMode(MANUAL);
  myPID3.SetOutputLimits(0,200);
  myPID3.SetMode(MANUAL);
  myPID4.SetOutputLimits(0,200); 
  myPID4.SetMode(MANUAL);
  setpoint1=23;
  setpoint2=23;
  ramp = false;
  change = false;
  heating = false;
  interval = 1;
  Serial.begin(9600);
  Serial.print("Setpoint1: ");
  Serial.print(setpoint1);
  Serial.print("Setpoint2: ");
  Serial.println(setpoint2);
  
}

void loop(){
  currentMillis = millis();
  initial_time = currentMillis;
  previousMillis = currentMillis;
  return_interval(initial_time);
  Serial.print("Secs since start: ");
  int secs = initial_time/1000;
  Serial.println(secs);
  Serial.print("interval: ");
  Serial.println(interval);
  if (interval % 2 == 0){
    heating = true;
  }
  else{
    heating = false;
    }
  if (!heating){
    currentMillis = millis(); 
    init_time = millis();  
    while ((currentMillis - init_time) <  perioda){      
      if ((currentMillis - init_time) <  co2_time){
        digitalWrite(3,LOW);
        digitalWrite(2,LOW);
        }
      else{
        digitalWrite(3,LOW);
        digitalWrite(2,LOW);
      }
      setpoint1 = 23.0;
      setpoint2 = 23.0;
      temp1=Thermister1();
      temp2=Thermister2();
      controlTemp1();
      controlTemp2();
      currentMillis = millis();
      Serial.print("Temp1: ");
      Serial.print(temp1);
      Serial.print("  Temp2: ");
      Serial.println(temp2);
      Serial.print("  Time: ");
      Serial.print(currentMillis/1000);
      Serial.println("");
        }
  init_time = millis();
  }
  else{
    while ((currentMillis - init_time) <  periodt){ 
      if ((currentMillis - init_time) <  co2_time){
        digitalWrite(3,HIGH);
        digitalWrite(2,HIGH);
        }
      else{
        digitalWrite(3,LOW);
        digitalWrite(2,LOW);
      }
      setpoint2 = calc_setpoint(interval);
      setpoint1 = 23.0;
      temp1=Thermister1();
      temp2=Thermister2();
      controlTemp1();
      controlTemp2();
      currentMillis = millis();
      Serial.print("Temp1: ");
      Serial.print(temp1);
      Serial.print("  Temp2: ");
      Serial.println(temp2);
      Serial.print("Setpoint1: ");
      Serial.print(setpoint1);
      Serial.print("  Setpoint2: ");
      Serial.println(setpoint2);
      Serial.print("  Time: ");
      Serial.print(currentMillis/1000);
      Serial.println("");
    }  
  }
}

int calc_setpoint (int interval){
  int setpoint2;
  setpoint2 = 23.0;
  if (interval == 2){
    setpoint2 = 31.0;
  }
  else if (interval == 4){
    setpoint2 = 37.0;
  }
  else if (interval == 6){
    setpoint2 = 42.0;
  }
  else if (interval == 8){
    setpoint2 = 47.0;
  }
  else if (interval == 10){
    setpoint2 = 52;
  }
  return setpoint2;
}

void return_interval(int initial_time){
  currentMillis = millis();
  unsigned long periodt = 600000; //calculated as ms, so 1000ms = 1s
  unsigned long perioda = 300000; //calculated as ms, so 1000ms = 1s
  counter = false;
   if (interval % 2 == 0){
    counter  = true;
  }
  else{
    counter = false;
    }
   if (!counter){
    if ((currentMillis - initial_time) >=  perioda){
    interval +=1;
  }
   }
  else{
    if ((currentMillis - initial_time) >=  periodt){
    interval +=1;
  }
   } 
}


double Thermister1() {  //Function to perform the fancy math of the Steinhart-Hart equation
  byte NTCPin = A4;
  float ADCvalue;
  float Resistance;
  ADCvalue = sample(NTCPin);
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = SERIESRESISTOR * Resistance;
  float steinhart;
  steinhart = Resistance / NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  return steinhart;
}
//
double Thermister2(){  //Function to perform the fancy math of the Steinhart-Hart equation
  byte NTCPin = A5;
  float ADCvalue;
  float Resistance;
  ADCvalue = sample(NTCPin);
  Resistance = (1023 / ADCvalue) - 1;
  Resistance = SERIESRESISTOR * Resistance;
  float steinhart;
  steinhart = Resistance / NOMINAL_RESISTANCE; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= BCOEFFICIENT; // 1/B * ln(R/Ro)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  return steinhart;
}


void controlTemp1() {
  if (temp1<setpoint1-histeresis){
     myPID1.SetMode(AUTOMATIC);
     myPID1.Compute();
     myPID2.SetMode(MANUAL);
     OutputE=0;
     analogWrite(PWMA, OutputC);
     digitalWrite(CWA,HIGH);
     digitalWrite(CCWA,LOW);  
     }
  if ((temp1>setpoint1-histeresis) && (temp1<setpoint1+histeresis)){
     myPID1.SetMode(MANUAL);
     OutputC=0;
     myPID2.SetMode(MANUAL);
     OutputE=0;
     analogWrite(PWMA, 0);
     digitalWrite(CWA,LOW);
     digitalWrite(CCWA,LOW);  
     }
   if (temp1>setpoint1+histeresis){
     myPID2.SetMode(AUTOMATIC);
     myPID2.Compute();
     myPID1.SetMode(MANUAL);
     OutputC=0;
     analogWrite(PWMA, OutputE);
     digitalWrite(CWA,LOW);
     digitalWrite(CCWA,HIGH);  
     }
     delay (100);  
}

void controlTemp2() {
  if (temp2<setpoint2-histeresis){
     myPID3.SetMode(AUTOMATIC);
     myPID3.Compute();
     myPID4.SetMode(MANUAL);
     OutputF=0;
     analogWrite(PWMB, OutputD);
     digitalWrite(CWB,HIGH);
     digitalWrite(CCWB,LOW);
     }
  if ((temp2>setpoint2-histeresis) && (temp2<setpoint2+histeresis)){
     myPID3.SetMode(MANUAL);
     OutputD=0;
     myPID4.SetMode(MANUAL);
     OutputF=0;
     analogWrite(PWMB, 0);
     digitalWrite(CWB,LOW);
     digitalWrite(CCWB,LOW);
     }
   if (temp2>setpoint2+histeresis){
     myPID4.SetMode(AUTOMATIC);
     myPID4.Compute();  
     myPID3.SetMode(MANUAL);
     OutputD=0;
     analogWrite(PWMB, OutputF);
     digitalWrite(CWB,LOW);
     digitalWrite(CCWB,HIGH);
     }
     delay (100);  
}


float sample(byte z)
/* This function will read the Pin 'z' 5 times and take an average.
 */
{
  byte i;
  float sval = 0;
  for (i = 0; i < 200; i++)
  {
  sval = sval + analogRead(z);// sensor on analog pin 'z'
  }
  sval = sval / 200.0;    // average
  return sval;
}
