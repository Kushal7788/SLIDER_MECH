#include "AutoPID.h"
// pins defined
//defined pin for encoder
#define encoderlfPin  2
#define encoderlbPin  3
//#define encoderrfPin  3
//#define encoderrbPin  3
//defined pins for motors
#define lfleg1 3
#define lfleg2 5
#define rfleg1 6
#define rfleg2 9
#define lbleg1 10
#define lbleg2 11
#define rbleg1 12
#define rbleg2 13

//PID variables declared here
int bklcorr = 0, bkrcorr = 0;
double synlkp = 0, synlki = 0.000, synlkd = 0, synrkp = 0, synrki = 0.00, synrkd = 00;    //Kp,Ki,Kd for AutoPID Lib
uint8_t synmax = 100, synmin = 0;   //Variables for min and max adjust pwm
//global variables defined
int pwmsl = 70;
int endvalue=0;
float angle;
volatile unsigned int encoderlfPos = 0;
volatile unsigned int encoderlbPos = 0;
volatile unsigned int encoderrfPos = 0;
volatile unsigned int encoderrbPos = 0;

AutoPID syncl(&encoderlbPos, encoderlfPos, &bklcorr, synmin, synmax, synlkp, synlki, synlkd);
AutoPID syncr(&encoderrbPos, encoderrfPos, &bkrcorr, synmin, synmax, synrkp, synrki, synrkd);
void setup() {

  Serial.begin (9600);
  // turn on pull-up resistor
  pinMode(encoderlfPin, INPUT_PULLUP);
  pinMode(encoderlbPin, INPUT_PULLUP);
  // pinMode(encoderrfPin, INPUT_PULLUP);
  //pinMode(encoderrbPin, INPUT_PULLUP);

  pinMode(lfleg1, OUTPUT);
  pinMode(lfleg2, OUTPUT);
  pinMode(rfleg1, OUTPUT);
  pinMode(rfleg2, OUTPUT);
  pinMode(lbleg1, OUTPUT);
  pinMode(lbleg2, OUTPUT);
  pinMode(rbleg1, OUTPUT);
  pinMode(rbleg2, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(encoderlfPin), doEncoderlf, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderlbPin), doEncoderlb, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderrfPin), doEncoderrf, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoderrbPin), doEncoderrb, CHANGE);
  syncl.setTimeStep(5);     //AutoPID Lib Functon
}
void doEncoderlf() {
  encoderlfPos++;
  encoderlfPos %= 280;
  //angle=encoder0Pos*1.285714;
}
void doEncoderlb() {
  encoderlbPos++;
  encoderlbPos %= 280;
  //angle=encoder1Pos*1.285714;
}
void doEncoderrf() {
  encoderrfPos++;
  encoderrfPos %= 280;
  //angle=encoder2Pos*1.285714;
}
void doEncoderrb() {
  encoderrbPos++;
  encoderrbPos %= 280;
  //angle=encoder3Pos*1.285714;
}
void drive(int st, int en)
{
  int diff = abs(st - en);
  analogWrite(lfleg1, pwmsl);
  analogWrite(lfleg2, 0);
  while (diff >= 0)
  {
    diff--;
    analogWrite(lbleg1, pwmsl + bklcorr);
    analogWrite(lbleg2, 0);
    syncl.run();
  }
}

void loop() {
  endvalue=40;
drive(encoderlfPos,encoderlfPos+endvalue)
}



