 /* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors
   to the A & B channel outputs

*/

#define encoder0PinA  2
#define encoder0PinB  3
#define pin1 10
#define pin2 11
float angle;
volatile unsigned int encoder0Pos = 0;

void setup() {
  pinMode(encoder0PinA, INPUT_PULLUP);
       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT_PULLUP);
       // turn on pull-up resistor
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), doEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), doEncoder, CHANGE);// encoder pin on interrupt 0 - pin 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

void loop() {
    Serial.println (angle);
  // do some stuff here - the joy of interrupts is that they take care of themselves
while(angle<270)
{
  analogWrite(pin1,70);
  analogWrite(pin2,0);
   Serial.println(angle);
}
Serial.println(angle);
}

void doEncoder() {
    encoder0Pos++;
    angle=encoder0Pos*1.285714;
}

/* See this expanded function to get a better understanding of the
   meanings of the four possible (pinA, pinB) value pairs:
*/
void doEncoder_Expanded() {
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }

  }
  Serial.println (encoder0Pos, DEC);          // debug - remember to comment out
  // before final program run
  // you don't want serial slowing down your program if not needed
}

/*  to read the other two transitions - just use another attachInterrupt()
  in the setup and duplicate the doEncoder function into say,
  doEncoderA and doEncoderB.
  You also need to move the other encoder wire over to pin 3 (interrupt 1).
*/

