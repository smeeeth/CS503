#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "DualMC33926MotorShield.h"

MPU6050 mpu;

DualMC33926MotorShield md;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// ================================================================
// ===                 FAULT DETECTION ROUTINE                  ===
// ================================================================

void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
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
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //motor setup
    Serial.println("Dual MC33926 Motor Shield");
    md.init();

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

//BALANCER VALUES
int pwm_r = 0;
int pwm_l = 0;
int PWMMAX = 300;
double theta_ref = 0.035; //0.036
double gyro_off = 0.44;
double Kangle = 1.3; //55
double Bangle = 0.7; //50
double PWMout = 0.0;
int torque_stall = 38;  //get rid of this perhaps?
bool use_torque = true;
int16_t ax, ay, az;
int16_t gx, gy, gz;

//IR VALUES

//ENCODER VALUES
double theta = 0;
double xLocation = 0;
double yLocation = 0;
const int encoderCounts = 32;
const double CIRCUMFERENCE = 10; //PLACEHOLDER: circumference of one wheel
const double WHEELBASE = 20; //PLACEHOLDER: distance between wheels and center (must be in same units as circumference, and input for challenge)
const double MULTIPLER = 5; //multiplier to amplify distance from point into PWM
void loop() {
  
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double raw_angle = atan2(ay,az);

    double angle_err = raw_angle + theta_ref;
    double angular_rate = ((double)gx)/131.0 - gyro_off;

    Serial.print(angle_err);
    Serial.print("  ");
    Serial.println(angular_rate);
    
    double deltaPWM = -Kangle*angle_err - Bangle*angular_rate;
    PWMout += deltaPWM;

    if (PWMout >= PWMMAX){
      PWMout = PWMMAX;
    }
    else if (PWMout <= -PWMMAX){
      PWMout = -PWMMAX;
    }

    if (use_torque){
      if (PWMout > 0){
        pwm_r = -(int)PWMout - torque_stall;
        pwm_l = (int)PWMout + torque_stall;
      } else {
        pwm_r = -(int)PWMout + torque_stall;
        pwm_l = (int)PWMout - torque_stall;
      }
    } else {
      pwm_r = -(int)PWMout;
      pwm_l = (int)PWMout;
    }

    md.setM1Speed(pwm_l);
    md.setM2Speed(pwm_r); 

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    //encoder stuff (LOTS OF PLACEHOLDERS)
    int leftEncoderCounts = 0; //PLACEHOLDER get encoder counts on left wheel
    int rightEncoderCounts = 0; //PLACEHOLDER get encoder counts on right wheel
    double distLeft = 2 * PI * leftEncoderCounts / (encoderCounts/CIRCUMFERENCE);
    double distRight = 2 * PI * rightEncoderCounts / (encoderCounts/CIRCUMFERENCE);

    double distAverage = (distLeft + distRight) / 2; //calcualte delta distance (for both wheels)
    double thetaChange = (distLeft - distAverage) / WHEELBASE; //calculate delta theta 

    //add new values to global variable
    theta = theta + thetaChange;
    xLocation = xLocation + (distAverage * cos(theta));
    yLocation = yLocation + (distAverage * sin(theta));

    Serial.print("[x = ");
    Serial.print(xLocation);
    Serial.print(", y = ");
    Serial.print(yLocation);
    Serial.print(", theta = ");
    Serial.print(theta);
    Serial.println("]");

    //adjust PWM for Y direction (forward backwards)
    //PWM -= yLocation * MULTIPLIER //this is easy, just accelerate forward or back
   
    
    
}
