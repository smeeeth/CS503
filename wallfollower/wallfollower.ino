#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include "DualMC33926MotorShield.h"

MPU6050 mpu;
DualMC33926MotorShield md;

#define encoder0PinA  2
#define encoder0PinB  5
#define encoder1PinA  3
#define encoder1PinB  6
#define PING_LEFT  13
#define PING_FORWARD  11
#define LED_STATE_F A3
#define LED_STATE_L A2

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
//    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_STATE_F, OUTPUT);
    pinMode(LED_STATE_L, OUTPUT);

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
double theta_ref = 0; //0.035
double gyro_off = 0.44;
double Kangle = 2.7; //0.7
double Bangle = 0.23; //0.2
double PWMout = 0.0;
int torque_stall_r = 24;  //get rid of this perhaps?
int torque_stall_l = 22;  //get rid of this perhaps?
bool use_torque = true;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//IR VALUES
bool A0_read = LOW;
bool A1_read = LOW;
bool last_B0_read = LOW;
bool last_B1_read = LOW;

//ENCODER VALUES
double theta = 0.0;
double xLocation = 0.0;
double yLocation = 0.0;
const int ENCODERCOUNTS = 32;
const double CIRCUMFERENCE = 10; //PLACEHOLDER: circumference of one wheel
const double WHEELBASE = 20; //PLACEHOLDER: distance between wheels and center (must be in same units as circumference, and input for challenge)
const double MULTIPLER = 5; //multiplier to amplify distance from point into PWM

//CLOCK VALUES
long previousMillis = 0;        // will store last time LED was updated
//long interval = 200;           // interval at which to blink (milliseconds)
long interval = 30;



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
//double Ktheta = 0.009;//0.011
double Ktheta = 0.0020;//0.058
//double Btheta = 0.064;//0.065 go over
double Btheta = 0.000;//0.068 go over
bool accelerate = true;

double x_goal = -190.0;//190
double y_goal = 0;
double L_stick = 6.0;
bool do_balance = true;
bool stopped = false;
int counter = 0;
double dist_to_goal = 0.0;
//double accel = 0.0;
double past_vel = 0.0;
int right = 0;
int left = 0;
bool use_t = true;
double last_dist =100;
bool start = false;
bool overshot = false;
enum states { BEGIN, NOTHING, ROTATE_RIGHT, MOVE_F, WAIT, ROTATE_20, STOP, ROTATE_270, MOVE_F2, ROTATE_270_2, MOVE_F3, MOVE_F4, ROTATE_L, MOVE_F5, MOVE_F6};
states robot_state = BEGIN;
enum wall_states { START };
wall_states maze_state = START;
double change_heading = 0.0;
bool use_heading = false;
bool corrected = false;
double x_circle = 10.0;

//PING variables
long Lduration = 0, Linches = 0, Lcm = 0;
long Fduration = 0, Finches = 0, Fcm = 0;
double last_loc = 0;
bool check_forward = true;
int turn_count = 0;
double min_dist = 1000;

void loop() {
  
   unsigned long currentMillis = millis();
   long diff = currentMillis - previousMillis;

    if (diff > interval){
        previousMillis = currentMillis;
        dist_two();
        counter++;

        if (counter > 1000){
            //robot_state = MOVE_F;
            counter = 0;
        }
        if (check_forward){
            check_forward = false;
            ping_start(PING_FORWARD);
            pinMode(PING_FORWARD, INPUT);
            Fduration = pulseIn(PING_FORWARD, HIGH, 3500);
            Fcm = Fduration / 29 / 2;
        } else if (!check_forward){ //counter % 2 == 0
            check_forward = true;
            ping_start(PING_LEFT);
            pinMode(PING_LEFT, INPUT);
            Lduration = pulseIn(PING_LEFT, HIGH, 3500);
            Lcm = Lduration / 29 / 2;
        }

        
        if (Fcm > 0 && Fcm <= 30){
            digitalWrite(LED_STATE_F, HIGH);
            if(robot_state == MOVE_F){
                robot_state = NOTHING;
            }
            last_loc = xLocation;
        } else if (Fcm <= 0 || Fcm > 40){
            digitalWrite(LED_STATE_F, LOW);
            if(robot_state == NOTHING || robot_state == BEGIN){
                robot_state = MOVE_F;
            }
        }

        if (Lcm > 0 && Lcm <= 40){
            digitalWrite(LED_STATE_L, HIGH);
            if(Lcm <= min_dist){
                min_dist = Lcm;
                Serial.println(Lcm);
            } else {
                if(robot_state == ROTATE_RIGHT){
                  robot_state = STOP;
                  Serial.println("STOP");
                }
            }
            //last_loc = xLocation;
        } else {
            digitalWrite(LED_STATE_L, LOW);
            //robot_state = MOVE_F;
        }

        

        
        switch(robot_state){

            case BEGIN:
                break;

            case NOTHING:
                Ktheta = 0.0040;
                Btheta = 0.068;
                torque_stall_r = 36;
                torque_stall_l = 36;
                dist_to_goal = last_loc-xLocation;
                theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                turn_count++;

                if( turn_count > 10){
                    robot_state = ROTATE_RIGHT;
                    turn_count = 0;
                }
                
                break;

            case ROTATE_RIGHT:
                left = 80;
                right = -80;
                use_torque = false;
                use_heading = false;
                break;

            case STOP:
                left = 0;
                right = 0;
                use_torque = true;
                robot_state = MOVE_F;
                break;
    
            case MOVE_F:
                Ktheta = 0.0037;
                Btheta = 0.068;
                use_torque = true;
                use_heading = true;
                change_heading = theta - atan2((y_goal - yLocation),(x_goal - xLocation));
                dist_to_goal = x_goal-xLocation;
                
                if(abs(dist_to_goal) < 2 && abs(velocity) < 0.1 && start){
                    use_torque = true;
                    theta_ref = 0;
                  }
    
                if(abs(xLocation - x_circle) < 0.5){
                    theta -= 0.011;
                    x_circle = xLocation + 10;
                  }
        
                if (accelerate){
                    start = true;
                    
                    accel = velocity - past_vel;
                    past_vel = velocity;
          
                    if (dist_to_goal > L_stick){
                        dist_to_goal = L_stick;
                    } else if (dist_to_goal < 0 && abs(dist_to_goal) > L_stick){
                        dist_to_goal = -L_stick;
                    }
    
                    theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                    if( abs(dist_to_goal) < 5){
                          theta_ref = 0;
                          use_heading = false;
                          use_torque = true;
                          robot_state = ROTATE_20;
                          xLocation = 0;
                          yLocation = 0;
                          x_goal = 70;
                          y_goal = 0;
                          theta = 0;
                          x_circle = 8;
                          use_heading = true;
                          start = false;
                    }else{
                          if(!stopped){
                            theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                          }
                    }
          
                }
                break;
    
            case ROTATE_20:
                  use_torque = false;
                  change_heading = theta - atan2((y_goal - yLocation),(x_goal - xLocation));
                  dist_to_goal = x_goal-xLocation;
                  
                  if(abs(dist_to_goal) < 2 && abs(velocity) < 0.1 && start){
                      use_torque = true;
                      theta_ref = 0;
                    }
    
                  if(abs(xLocation - x_circle) < 0.5){
                    theta -= 0.08;
                    x_circle = xLocation + 8;
                  }
    
                  if (accelerate){
                      start = true;
                      
                      accel = velocity - past_vel;
                      past_vel = velocity;
            
                      if (dist_to_goal > L_stick){
                          dist_to_goal = L_stick;
                      } else if (dist_to_goal < 0 && abs(dist_to_goal) > L_stick){
                          dist_to_goal = -L_stick;
                          //overshot = true;
                      }
      
                      theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                      if( abs(dist_to_goal) < 5){
                            theta_ref = 0;
                            use_heading = false;
                            robot_state = STOP;
                      }else{
                            if(!stopped){
                              theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                            }
                      }
            
                  }
                  break;
    
              case MOVE_F2:
    
                  use_torque = false;
                  change_heading = theta - atan2((y_goal - yLocation),(x_goal - xLocation));
                  dist_to_goal = x_goal-xLocation;
                  
                  if(abs(dist_to_goal) < 2 && abs(velocity) < 0.1 && start){
                      use_torque = true;
                      theta_ref = 0;
                    }
    
                  if(abs(xLocation - x_circle) < 0.5){
                    theta -= 0.043;
                    x_circle = xLocation + 10;
                  }
     
          
                  if (accelerate){
                      start = true;
                      
                      accel = velocity - past_vel;
                      past_vel = velocity;
            
                      if (dist_to_goal > L_stick){
                          dist_to_goal = L_stick;
                      } else if (dist_to_goal < 0 && abs(dist_to_goal) > L_stick){
                          dist_to_goal = -L_stick;
                      }
      
                      theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                      if( abs(dist_to_goal) < 5){
                            theta_ref = 0;
                            use_heading = false;
                            robot_state = ROTATE_270;
    
                            xLocation = 0;
                            yLocation = 0;
                            x_goal = 200;
                            y_goal = 0;
                            theta = 0;
                            x_circle = 6;
                            use_heading = true;
                            start = false;
                            Ktheta = 0.0070;
                            Btheta = 0.030;
                            }else{
                            if(!stopped){
                              theta_ref = Ktheta*(dist_to_goal) - Btheta * velocity; //goal - xlocation
                            }
                      }
            
                  }
                  break;
          }
    }

    if (do_balance){
        if(use_heading){
          left = change_heading*2;
          right = -change_heading*2;
        }
        balance(left, right, use_t);
    } else {
        //left_turn();
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
        } else if (A0_read == LOW && current_B == HIGH) {
            encoder0Pos = encoder0Pos + 1;
        } else if (A0_read == HIGH && current_B == LOW) {
            encoder0Pos = encoder0Pos + 1;
        } else {
            encoder0Pos = encoder0Pos - 1;
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
        } else if (A1_read == LOW && current_B == HIGH) {
            encoder1Pos = encoder1Pos + 1;
        } else if (A1_read == HIGH && current_B == LOW) {
            encoder1Pos = encoder1Pos + 1;
        } else {
            encoder1Pos = encoder1Pos - 1;
        }

        last_B1_read = current_B;
    }
}

void balance(int left, int right, bool use_t) {
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
    md.setM2Speed(pwm_l - left ); 
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

    distRight += 2 * encoder0Pos * 1 / (encoderCounts/CIRCUMFERENCE);
    distLeft += 2 * encoder1Pos * 1 / (encoderCounts/CIRCUMFERENCE);
    double distAverage = (distLeft + distRight) / 2;
    double thetaChange = (distLeft - distAverage)/ WHEELBASE;
    theta = theta + thetaChange;
    xLocation = xLocation + (distAverage * cos(theta));
    yLocation = yLocation + (distAverage * sin(theta));
    velocity = distAverage;
//    Serial.print("Right: ");
//    Serial.print(distRight);
//    Serial.print(", Left: ");
//    Serial.println(distLeft);
    
    distLeft = 0;
    distRight = 0;
    encoder0Pos = 0;
    encoder1Pos = 0;

    

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

void ping_start(int pin){

    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
}

