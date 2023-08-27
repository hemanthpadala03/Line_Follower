

//#include <SparkFun_TB6612.h>
//
//#define AIN1 4
//#define BIN1 6
//#define AIN2 3
//#define BIN2 7
//#define PWMA 9
//#define PWMB 10
//#define STBY 5
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
//const int offsetA = 1;
//const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

//Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
//Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;


int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
}


void loop()
{
//  while (digitalRead(11)) {}
//  delay(1000);
//  calibrate();
//  while (digitalRead(12)) {}
//  delay(1000);

//  while (1)
//96
//  {
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);
    if(analogRead(A0)>800 && analogRead(A1)>800 && analogRead(A2)>800 && analogRead(A3)>800 && analogRead(A4)>800){
        analogWrite(in3,0);
        analogWrite(in4,0);
        analogWrite(in1,0);
        analogWrite(in2,0); 
    }
    if(analogRead(A0)<100 && analogRead(A1)<100 && analogRead(A2)<100 && analogRead(A3)<100 && analogRead(A4)<100){
        analogWrite(in3,0);
        analogWrite(in4,0);
        analogWrite(in1,0);
        analogWrite(in2,0); 
    }
    if (analogRead(A0) > 500 && analogRead(A4) < 500 )
    {
      lsp = 0; rsp = lfspeed;
//      motor1.drive(0);
//      motor2.drive(lfspeed);
        Serial.println(lfspeed);
        Serial.println("Right");
        analogWrite(in1,lfspeed);
        analogWrite(in2,0);
        analogWrite(in3,0);
        analogWrite(in4,0);
    }

    else if (analogRead(A4) > 500 && analogRead(A0) < 500)
    { lsp = lfspeed; rsp = 0;
//      motor1.drive(lfspeed);
//      motor2.drive(0);
        Serial.println(lfspeed);
        Serial.println("Left");
        analogWrite(in3,lfspeed);
        analogWrite(in4,0);
        analogWrite(in1,0);
        analogWrite(in2,0);
    }
    else if (analogRead(A2) > 500)
    {
      Kp = 0.0006 * (1000 - analogRead(A2));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
//  }
}

void linefollow()
{
  int error = (analogRead(A1) - analogRead(A3));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
//  motor1.drive(lsp);
//  motor2.drive(rsp);
    Serial.println("PID");
    Serial.println(lsp);
    Serial.println(rsp);
    analogWrite(in1,lsp);
    analogWrite(in2,0);
    analogWrite(in3,rsp);
    analogWrite(in4,0);
}

//void calibrate()
//{
//  for ( int i = 1; i < 6; i++)
//  {
//    minValues[i] = analogRead(i);
//    maxValues[i] = analogRead(i);
//  }
//  
//  for (int i = 0; i < 3000; i++)
