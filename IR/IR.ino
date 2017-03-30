/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B  

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors 
   to the A & B channel outputs 

*/ 

#define encoder0PinA  2
#define encoder0PinB  4
#define encoder1PinA  3
#define encoder1PinB  5

volatile unsigned int encoder0Pos = 0;
volatile unsigned int encoder1Pos = 0;

void doEncoder0();
void doEncoder1();


void setup() { 

  //setup encoder 0
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(0, doEncoder0, CHANGE);  // encoder pin on interrupt 0 - pin 2

  //setup encoder 1
  
  pinMode(encoder1PinA, INPUT); 
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT); 
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(1, doEncoder1, CHANGE);  // encoder pin on interrupt 0 - pin 2

  
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk

} 

int counter = 0;
bool A0_read = LOW;
bool A1_read = LOW;
bool last_B0_read = LOW;
bool last_B1_read = LOW;

void loop(){
// do some stuff here - the joy of interrupts is that they take care of themselves
}


/* See this expanded function to get a better understanding of the
 * meanings of the four possible (pinA, pinB) value pairs:
 */
 //right wheel
void doEncoder0(){
    A0_read = digitalRead(encoder0PinA);
    bool current_B = digitalRead(encoder0PinB);
    if (current_B != last_B0_read) {
        if(A0_read == LOW && current_B == LOW){
          Serial.println ("CW");
          encoder0Pos = encoder0Pos - 1;
        } else if (A0_read == LOW && current_B == HIGH){
          Serial.println ("CCW");
          encoder0Pos = encoder0Pos + 1;
        } else if (A0_read == HIGH && current_B == LOW){
          Serial.println ("CCW");
          encoder0Pos = encoder0Pos + 1;
        } else {
          Serial.println ("CW");
          encoder0Pos = encoder0Pos - 1;
        }
          
        last_B0_read = current_B;
    }
    Serial.println (encoder0Pos, DEC);
}

//left wheel
void doEncoder1(){
    A1_read = digitalRead(encoder1PinA);
    bool current_B = digitalRead(encoder1PinB);
    if (current_B != last_B1_read) {
      if(A1_read == LOW && current_B == LOW){
          Serial.println ("CCW");
          encoder1Pos = encoder1Pos - 1;
        } else if (A1_read == LOW && current_B == HIGH){
          Serial.println ("CW");
          encoder1Pos = encoder1Pos + 1;
        } else if (A1_read == HIGH && current_B == LOW){
          Serial.println ("CW");
          encoder1Pos = encoder1Pos + 1;
        } else {
          Serial.println ("CCW");
          encoder1Pos = encoder1Pos - 1;
        }
        
        last_B1_read = current_B;
    }
    Serial.println (encoder1Pos, DEC);
}

