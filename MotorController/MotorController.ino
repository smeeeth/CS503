#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "DualMC33926MotorShield.h"

MPU6050 mpu;
DualMC33926MotorShield md;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define encoder0PinA  2
#define encoder0PinB  5
#define encoder1PinA  3
#define encoder1PinB  6

volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;

bool blinkState = false;
void doEncoder0();
void doEncoder1();

// ================================================================
// ===                 FAULT DETECTION ROUTINE                  ===
// ================================================================

void stopIfFault()
{
    if (md.getFault())
    {
        Serial.println("fault");
        while (1);
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)---->Two Wire Bit Rate Generator
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize(); //MPU6050.cpp
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    //encoder setup
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
    attachInterrupt(1, doEncoder1, CHANGE);  // encoder pin on interrupt 1 - pin 3

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //motor setup
    md.init();

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

//BALANCER VALUES
int pwm_r = 0;
int pwm_l = 0;
int PWMMAX = 400;
double theta_ref = 0.0; //0.035
double gyro_off = 0.44;
double Kangle = 2.7; //0.7
double Bangle = 0.23; //0.2
double PWMout = 0.0;
int torque_stall_r = 37;  //get rid of this perhaps?
int torque_stall_l = 36;  //get rid of this perhaps?
bool use_torque = true;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//IR VALUES
bool A0_read = LOW;
bool A1_read = LOW;
bool last_B0_read = LOW;
bool last_B1_read = LOW;

//ENCODER VALUES
double theta = 0;
double xLocation = 0;
double yLocation = 0;
const int ENCODERCOUNTS = 32;
const double CIRCUMFERENCE = 10; //PLACEHOLDER: circumference of one wheel
const double WHEELBASE = 20; //PLACEHOLDER: distance between wheels and center (must be in same units as circumference, and input for challenge)
const double MULTIPLER = 5; //multiplier to amplify distance from point into PWM

//CLOCK VALUES
long previousMillis = 0;        // will store last time LED was updated
//long interval = 200;           // interval at which to blink (milliseconds)
long interval = 50;



//testing vals
int error_right = 0;
int error_left = 0;
double velocity = 0;
const double THETA_MAX_FORWARD = 0.13;
const double THETA_MAX_BACKWARD = -0.13;

bool stopping = false;
double distLeft;
double distRight;
const int encoderCounts = 32;


//motor 1 is the right wheel
//motor 2 is the left wheel
int forward = 0;
double angle = 0.0;
double pastVel = 0.0;
double accel = 0.0;


double Kvelocity = 0.3;
double Bvelocity = 0.1;

double PWMvel = 0.0;
double deltaVelPWM = 0;
double past_loc = 0;
double x_diff = 0.0;
double x_rate = 0.0;
double distToGo = 0.0;
double v_diff = 0.0;
const double TRANSLATE_MAX = 50;
double theta_change = 0.0;
double Ktheta = 0.0055;
double Btheta = 0.0366;
bool accelerate = false;

double x_goal = 100.0;
double L_stick = 4.0;
bool do_balance = true;
bool stopped = false;
int counter = 0;
double dist_to_goal = 0.0;
//double accel = 0.0;
double past_vel = 0.0;
int right = 0, left = 0;
bool use_t = true;

void loop() {
  
   unsigned long currentMillis = millis();
   long diff = currentMillis - previousMillis;

    if (diff > interval){
        previousMillis = currentMillis;
        dist_two();

        accelerate = true;

//        if (abs(theta) > 0.015){
//            left = theta*1000;
//            right = -theta*1000;
//        }

//        Serial.print("Theta_ref: ");
//        printDouble(theta_ref, 4);
//        Serial.print(", Location: ");
//        Serial.print(xLocation);
//        Serial.print(", dist_to_goal: ");
//        Serial.print(x_goal-xLocation);
//        Serial.print(", Velocity: ");
//        Serial.println(velocity);

        if (accelerate){

          accel = velocity - past_vel;
          past_vel = velocity;

          dist_to_goal = x_goal-xLocation;

          if (dist_to_goal > L_stick){
              dist_to_goal = L_stick;
          } else if (dist_to_goal < 0 && abs(dist_to_goal) > L_stick){
              dist_to_goal = -L_stick;
          }

          if (dist_to_goal < 2.0){
              counter++;
          }
          if (counter < 10){
              theta_ref += 0.01;
          } else {
              theta_ref = 0.0;
              //x_goal += 10.0;
              counter = 0;
          }

//          Serial.print("Dist: ");
//          Serial.println(dist_to_goal);
          
          theta_ref = Ktheta*(dist_to_goal) + -Btheta*velocity; //goal - xlocation
            
        }

//        if (theta_ref >= THETA_MAX_FORWARD){
//            theta_ref = THETA_MAX_FORWARD;
//        } else if (theta_ref <= THETA_MAX_BACKWARD){
//            theta_ref = THETA_MAX_BACKWARD;
//        }
    }

    if (do_balance){
        balance(left, right, use_t);
    } else {
        left_turn();
    }

    
}


/* See this expanded function to get a better understanding of the
 * meanings of the four possible (pinA, pinB) value pairs:
 */
//right wheel
void doEncoder0() {
    A0_read = digitalRead(encoder0PinA);
    bool current_B = digitalRead(encoder0PinB);
    if (current_B != last_B0_read) {
        if (A0_read == LOW && current_B == LOW) {
            encoder0Pos = encoder0Pos - 1;
            dist_Change_Minus_left();
        } else if (A0_read == LOW && current_B == HIGH) {
            encoder0Pos = encoder0Pos + 1;
            dist_Change_Plus_left();
        } else if (A0_read == HIGH && current_B == LOW) {
            encoder0Pos = encoder0Pos + 1;
            dist_Change_Plus_left();
        } else {
            encoder0Pos = encoder0Pos - 1;
            dist_Change_Minus_left();
        }

        last_B0_read = current_B;
    }
}

//left wheel
void doEncoder1() {
    A1_read = digitalRead(encoder1PinA);
    bool current_B = digitalRead(encoder1PinB);
    if (current_B != last_B1_read) {
        if (A1_read == LOW && current_B == LOW) {
            encoder1Pos = encoder1Pos - 1;
            dist_Change_Minus_right();
        } else if (A1_read == LOW && current_B == HIGH) {
            encoder1Pos = encoder1Pos + 1;
            dist_Change_Plus_right();
        } else if (A1_read == HIGH && current_B == LOW) {
            encoder1Pos = encoder1Pos + 1;
            dist_Change_Plus_right();
        } else {
            encoder1Pos = encoder1Pos - 1;
            dist_Change_Minus_right();
        }

        last_B1_read = current_B;
    }
}

void balance(int left, int right, bool use_t) {
    use_torque = use_t;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double raw_angle = atan2(ay,az);

    double angle_err = raw_angle + theta_ref;
    double angular_rate = ((double)gx)/131.0 - gyro_off;
    double deltaPWM = -Kangle*angle_err - Bangle*angular_rate;
    PWMout += deltaPWM;
    

    int PWMint = (int)PWMout;

    if (use_torque){
      if (PWMint > 0){
        pwm_l = -PWMint - torque_stall_l;
        pwm_r = PWMint + torque_stall_r;
      } else {
        pwm_l = -PWMint + torque_stall_l;
        pwm_r = PWMint - torque_stall_r;
      }
    } else {
      pwm_l = -PWMint;
      pwm_r = PWMint;
    }

    //for balance
    if (PWMint >= PWMMAX){
      PWMint = PWMMAX;
    }
    else if (PWMint <= -PWMMAX){
      PWMint = -PWMMAX;
    }

    md.setM1Speed(pwm_r + right);
    md.setM2Speed(pwm_l + left); 
}


void dist_Change_Plus_left(){
    distLeft += 2 * 1 * 1 / (encoderCounts/CIRCUMFERENCE);
  }

void dist_Change_Minus_left(){
    distLeft +=- 2 * 1 * 1 / (encoderCounts/CIRCUMFERENCE);
}

void dist_Change_Plus_right(){
  distRight += 2 * 1 * 1 / (encoderCounts/CIRCUMFERENCE);
}

void dist_Change_Minus_right(){
  distRight +=- 2 * 1 * 1 / (encoderCounts/CIRCUMFERENCE);
}

void dist_two(){
    //add new values to global variable

    double distAverage = (distLeft + distRight) / 2;
    double thetaChange = (distLeft - distAverage)/ WHEELBASE;
    theta = theta + thetaChange;
    xLocation = xLocation + (distAverage * cos(theta));
    velocity = distAverage;
    distLeft = 0;
    distRight = 0;

}
void left_turn(){
    use_torque = false;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double raw_angle = atan2(ay, az);

    double angle_err = raw_angle + theta_ref;
    double angular_rate = ((double)gx) / 131.0 - gyro_off;
    double deltaPWM = -Kangle * angle_err - Bangle * angular_rate;
    PWMout += deltaPWM;// + deltaVelPWM;

    int PWMint = (int)PWMout;

    if (use_torque) {
        if (PWMint > 0) {
            pwm_l = -PWMint - torque_stall_l;
            pwm_r = PWMint + torque_stall_r;
        } else {
            pwm_l = -PWMint + torque_stall_l;
            pwm_r = PWMint - torque_stall_r;
        }
    } else {
        pwm_l = -PWMint;
        pwm_r = PWMint;
    }

    if (PWMint >= PWMMAX) {
        PWMint = PWMMAX;
    }
    else if (PWMint <= -PWMMAX) {
        PWMint = -PWMMAX;
    }

    md.setM1Speed(pwm_r+100);
    md.setM2Speed(pwm_l+100);
  
}

void right_turn(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double raw_angle = atan2(ay, az);

    double angle_err = raw_angle + theta_ref;
    double angular_rate = ((double)gx) / 131.0 - gyro_off;
    double deltaPWM = -Kangle * angle_err - Bangle * angular_rate;
    PWMout += deltaPWM;// + deltaVelPWM;

    int PWMint = (int)PWMout;

    if (use_torque) {
        if (PWMint > 0) {
            pwm_l = -PWMint - torque_stall_l;
            pwm_r = PWMint + torque_stall_r;
        } else {
            pwm_l = -PWMint + torque_stall_l;
            pwm_r = PWMint - torque_stall_r;
        }
    } else {
        pwm_l = -PWMint;
        pwm_r = PWMint;
    }

    if (PWMint >= PWMMAX) {
        PWMint = PWMMAX;
    }
    else if (PWMint <= -PWMMAX) {
        PWMint = -PWMMAX;
    }

    md.setM1Speed(pwm_r/3-100);
    md.setM2Speed(pwm_l/3-100);
}






void move_forward(int next){

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double raw_angle = atan2(ay, az);

    double angle_err = raw_angle + theta_ref;
    double angular_rate = ((double)gx) / 131.0 - gyro_off;
    double deltaPWM = -Kangle * angle_err - Bangle * angular_rate;
    PWMout += deltaPWM;// + deltaVelPWM;

    int PWMint = (int)PWMout;

    if (use_torque) {
        if (PWMint > 0) {
            pwm_l = -PWMint - torque_stall_l;
            pwm_r = PWMint + torque_stall_r;
        } else {
            pwm_l = -PWMint + torque_stall_l;
            pwm_r = PWMint - torque_stall_r;
        }
    } else {
        pwm_l = -PWMint;
        pwm_r = PWMint;
    }

    if (PWMint >= PWMMAX) {
        PWMint = PWMMAX;
    }
    else if (PWMint <= -PWMMAX) {
        PWMint = -PWMMAX;
    }

    Serial.println(next);

    md.setM1Speed(pwm_r + next);
    md.setM2Speed(pwm_l - next);
}


void printDouble( double val, byte precision){
 // prints val with number of decimal places determine by precision
 // precision is a number from 0 to 6 indicating the desired decimial places
 // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)
 if (val < 0)
  Serial.print("-");
 Serial.print (int(val));  //prints the int part
 if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
      mult *=10;
      
   if(val >= 0)
     frac = (val - int(val)) * mult;
   else
     frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
     padding--;
   while(  padding--)
     Serial.print("0");
   Serial.print(frac,DEC) ;
 }
 Serial.println("");
}

//void accel_forward(){
//    theta_ref = 0.06;
//
//    if (theta_ref > THETA_MAX){
//      theta_ref = THETA_MAX;
//    }
//    balance();
//}
//
//void accel_backward(){
//    theta_ref += -0.06;
//
//    if (theta_ref < -THETA_MAX){
//      theta_ref = -THETA_MAX;
//    }
//    
//    balance();
//}
//
//void no_accel(){
//    theta_ref = -0.005;
//    balance();
//}


