#include <Wire.h>
#include <MechaQMC5883.h>
#include <Math.h>

MechaQMC5883 qmc;

#define ENC_COUNT_REV 12

#define ENC_IN  3
#define ENC_IN2 4

#define PWM  10
#define PWM2 9

#define M1DIR1  12
#define M1DIR2  5

#define M2DIR1 8
#define M2DIR2 11

volatile long encoderValue = 0;
volatile long encoderValue2 = 0;

int interval = 10;

long previousMillis = 0;
long currentMillis = 0;

double rpm = 0;
double u = 0, u_1 = 0, e_2 = 0, e = 0, e_1 = 0, ref = 0;

double rpm2 = 0;
double u2 = 0, u2_1 = 0, e2_2 = 0, e2 = 0, e2_1 = 0, ref2 = 0;


int motorPWM = 0;
int motorPWM2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENC_IN, INPUT_PULLUP);
  pinMode(ENC_IN2, INPUT_PULLUP);

  pinMode(PWM,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(M1DIR1,OUTPUT);
  pinMode(M1DIR2,OUTPUT);
  pinMode(M2DIR1,OUTPUT);
  pinMode(M2DIR2,OUTPUT);
  
  digitalWrite(M1DIR1,HIGH);
  digitalWrite(M1DIR2,LOW);
  digitalWrite(M2DIR1,HIGH);
  digitalWrite(M2DIR2,LOW);
  
  attachInterrupt(digitalPinToInterrupt(ENC_IN),updateEncoder,RISING);
//  attachInterrupt(digitalPinToInterrupt(ENC_IN2),updateEncoder2,RISING);

  previousMillis = millis();
  Wire.begin();
  qmc.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  //motorPwm = 255;
  analogWrite(PWM, motorPWM);
  analogWrite(PWM2, motorPWM2);
  if(detectaFlanco(ENC_IN2)==1)
  {
    encoderValue2++;
  }
  currentMillis = millis();
  // Control Motor 1
  ref = 150; 
  ref2 = 150; 
  if(currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
    rpm = (float)(encoderValue*(60.0*(1000.0/10.0))/(ENC_COUNT_REV*34.014));
    e = ref - rpm;
    u = u_1 + (12.07/200.0)*e - (4.61/200.0)*e_1;
    if(u > 12)
    {
      u = 12;
    }
    if(u < 0)
    {
      u = 0;
    }
    motorPWM = (int)u*255.0/12.0;
    if(motorPWM > 255)
    {
      motorPWM = 255;
    }
    if(motorPWM < 0)
    {
      motorPWM = 0;
    }
    u_1 = u;
    e_2 = e_1;
    e_1 = e;
    encoderValue = 0;
    

    rpm2 = (float)(encoderValue2*(60.0*(1000.0/10.0))/(ENC_COUNT_REV*34.014));
    e2 = ref2 - rpm2;
    u2 = u2_1 + (12.07/200.0)*e2 - (4.61/200.0)*e2_1;
    if(u2 > 12)
    {
      u2 = 12;
    }
    if(u2 < 0)
    {
      u2 = 0;
    }
    motorPWM2 = (int)u2*255.0/12.0;
    if(motorPWM2 > 255)
    {
      motorPWM2 = 255;
    }
    if(motorPWM2 < 0)
    {
      motorPWM2 = 0;
    }
    u2_1 = u2;
    e2_2 = e2_1;
    e2_1 = e2;
    encoderValue2 = 0;
    Serial.println(rpm2);
  }
  int x,y,z,acimut;
  qmc.read(&x,&y,&z,&acimut);
  if(acimut >= 20 && acimut < 180)
  {
      digitalWrite(M1DIR1,LOW);
      digitalWrite(M1DIR2,HIGH);

      digitalWrite(M2DIR1,HIGH);
      digitalWrite(M2DIR2,LOW);
    
  }
  else if(acimut >= 190 && acimut < 340)
  {
      digitalWrite(M1DIR1,HIGH);
      digitalWrite(M1DIR2,LOW);

      digitalWrite(M2DIR1,LOW);
      digitalWrite(M2DIR2,HIGH);
    
  }
  else if(acimut >= 350 || acimut < 10)
  {
      digitalWrite(M1DIR1,HIGH);
      digitalWrite(M1DIR2,LOW);
      
      digitalWrite(M2DIR1,HIGH);
      digitalWrite(M2DIR2,LOW);
  }
 
}
void updateEncoder(){
  encoderValue++;
}
void updateEncoder2(){
  encoderValue2++;
}

int detectaFlanco(int pin) {
  //Devuelve 1 flanco ascendente, -1 flanco descendente y 0 si no hay nada
  static boolean anterior_estado = digitalRead(pin);
  boolean estado = digitalRead(pin);


  if (anterior_estado != estado) {
    if (estado == HIGH) {
   anterior_estado = estado;
      return 1;
    } 
    else {
      anterior_estado = estado;
      return -1;
    }
  }
  else {
    return 0;
  }
}
// -12.019500657621025, -77.04692980809826
double angleFromCoordinate(double lat1, double long1, double lat2,
        double long2) {

    double dLon = (long2 - long1);

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)
            * cos(lat2) * cos(dLon);

    double brng = atan2(y, x);

    return brng;
}
