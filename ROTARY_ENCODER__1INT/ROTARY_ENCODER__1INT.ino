 /* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors
   to the A & B channel outputs

*/

#define encoder0PinA  2
float angle;

volatile unsigned int encoder0Pos = 0;

void setup() {
  pinMode(encoder0PinA, INPUT_PULLUP);
       // turn on pull-up resistor
  
  attachInterrupt(digitalPinToInterrupt(2), doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

void loop() {
  Serial.println(angle);
  delay(250);
  }

void doEncoder() {
    encoder0Pos++;
    angle=encoder0Pos*360/70;
  }

/* See this expanded function to get a better understanding of the
   meanings of the four possible (pinA, pinB) value pairs:
*/
//void doEncoder_Expanded() {
//  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
//    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
//      // encoder is turning
//      encoder0Pos = encoder0Pos - 1;         // CCW
//    }
//    else {
//      encoder0Pos = encoder0Pos + 1;         // CW
//    }
//  }
//  else                                        // found a high-to-low on channel A
//  {
//    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
//      // encoder is turning
//      encoder0Pos = encoder0Pos + 1;          // CW
//    }
//    else {
//      encoder0Pos = encoder0Pos - 1;          // CCW
//    }
//
//  }
//  Serial.println (encoder0Pos, DEC);          // debug - remember to comment out
//  // before final program run
//  // you don't want serial slowing down your program if not needed
//}
//
///*  to read the other two transitions - just use another attachInterrupt()
//  in the setup and duplicate the doEncoder function into say,
//  doEncoderA and doEncoderB.
//  You also need to move the other encoder wire over to pin 3 (interrupt 1).
//*/

