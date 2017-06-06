/*
 *  Arduino_move - Robot Motor Control System
 *    by Tyler Reese 
 *    
 *  references: https://www.robotshop.com/en/3wd-compact-omni-directional-arduino-compatible-mobile-robot.html  
 *              https://wiki.ros.org/rosserial_arduino/Tutorials
 */

#include <ros.h>
#include <Wheel.h>  // This library is a scaled down version of the original omni-wheeled robot libraries
#include <Wire.h>
#include <PIDT.h>  

// Robot state definitions 
#define STOP 0
#define ADVANCE 1
#define ROTATE 2
#define ROT_PWM 60

// Serial message definitions
#define MSG_KEY1 123
#define MSG_KEY2 132
#define MSG_KEY3 231
#define LIN_MSG 17
#define ROT_MSG 18
#define STOP_MSG 19
#define HDG_MSG 20
#define END_MSG 4

// PID Variable Declarations
volatile float lin_PWM = 0.0;
volatile double pps1_feedbk = 0.0;
volatile double pps2_feedbk = 0.0;
volatile double pps3_feedbk = 0.0;
volatile double heading_feedbk = 0.0;
double w1_output = 0.0;
double w2_output = 0.0;
double w3_output = 0.0;
int wheel_rot_dir = 0;                            // Wheel rotation direction (0:clockwise | 1:cntr-clockwise) 
double linear_output = 0.0;             
double Setpoint_PPS = 0.0;
double Setpoint_Heading = 0.0;
// PID Tuning Parameters
double Kp_Rot1=0.26, Ki_Rot1=0.06, Kd_Rot1=0.0;   // Wheel 1 rotational PID tunings(orig:0.26,0.02,0.0) 
double Kp_Rot2=0.26, Ki_Rot2=0.06, Kd_Rot2=0.0;   // Wheel 2 rotational PID tunings
double Kp_Rot3=0.26, Ki_Rot3=0.06, Kd_Rot3=0.0;   // Wheel 3 rotational PID tunings
double Kp_Lin=250.0, Ki_Lin=0.0, Kd_Lin=0;       // Linear PID tunings

union Data1{
  byte as_byte[4];
  double dbl;
};

union Data2{
  byte as_byte[2];
  int intgr;
};

// Serial Communication Variable Declarations
int current_Action = STOP;
byte event[11] = {0};


// Initialize the Rotational and Linear PIDs  
PID rotate_W1(&pps1_feedbk,&w1_output,&Setpoint_PPS,Kp_Rot1,Ki_Rot1,Kd_Rot1,DIRECT);
PID rotate_W2(&pps2_feedbk,&w2_output,&Setpoint_PPS,Kp_Rot2,Ki_Rot2,Kd_Rot2,DIRECT);
PID rotate_W3(&pps3_feedbk,&w3_output,&Setpoint_PPS,Kp_Rot3,Ki_Rot3,Kd_Rot3,DIRECT);
PID linear_CTRL(&heading_feedbk,&linear_output,&Setpoint_Heading,Kp_Lin,Ki_Lin,Kd_Lin,DIRECT);

void isr1();                                // ISR init for optical encoder on wheel #1       
  Wheel wheel1(9,8,6,7,&isr1);              // Wheel object #1 initialization
  irqISR(wheel1,isr1);                      // Attach Wheel1 to isr1
void isr2();                                // ISR init for optical encoder on wheel #2
  Wheel wheel2(10,11,14,15,&isr2);          // Wheel object #2 init
  irqISR(wheel2,isr2);                      // Attach Wheel2 to isr2
void isr3();                                // ISR init for optical encoder on wheel #3
  Wheel wheel3(3,2,4,5,&isr3);              // Wheel object #3 init 
  irqISR(wheel3,isr3);                      // Attach Wheel3 to isr3

// serialEvent() - This routine runs between loop() runs. If there is an event it reads the rPi instruction and acts accordingly. 
void serialEvent() {
  while (Serial.available() < 4);
  byte isStart = Serial.read();
  if (isStart == MSG_KEY1) {
    isStart = Serial.read();
    if (isStart == MSG_KEY2) {
      isStart = Serial.read();
      if (isStart == MSG_KEY3) {
        isStart = Serial.read();
        
        if (isStart == LIN_MSG) {
          union Data1 data1;
          current_Action = ADVANCE;
          while (Serial.available() < 9);  
          for (int i = 0; i < 4; i++) {
            data1.as_byte[i] = Serial.read();
          }
          heading_feedbk = data1.dbl;  
          for (int i = 0; i < 4; i++) {
            data1.as_byte[i] = Serial.read();
          }
          Setpoint_Heading = data1.dbl;
          lin_PWM = float(Serial.read());
        } 
         
        else if (isStart == ROT_MSG) {
          union Data1 data1;
          union Data2 data2;
          current_Action = ROTATE;
          while (Serial.available() < 6);  
          for (int i = 0; i < 4; i++) {
            data1.as_byte[i] = Serial.read();
          }
          heading_feedbk = data1.dbl;  
          data2.as_byte[0] = Serial.read();
          data2.as_byte[1] = Serial.read();
          Setpoint_PPS = double(data2.intgr);           
          if (Setpoint_PPS < 0.0) {
            wheel_rot_dir = 0;
            rotate_W1.SetControllerDirection(REVERSE);   
            rotate_W2.SetControllerDirection(REVERSE);
            rotate_W3.SetControllerDirection(REVERSE);       
          }else {
            wheel_rot_dir = 1;
            rotate_W1.SetControllerDirection(DIRECT);   
            rotate_W2.SetControllerDirection(DIRECT);
            rotate_W3.SetControllerDirection(DIRECT);
          }  
        }
        
        else if (isStart == STOP_MSG) {
          current_Action = STOP;
        }
        
        else if (isStart == HDG_MSG) {
          union Data1 data1;
          while (Serial.available() < 4);  
          for (int i = 0; i < 4; i++) {
            data1.as_byte[i] = Serial.read();
          }
          double new_head = data1.dbl;
          if (new_head < PI || new_head > -PI) {
            if (!(new_head > -0.0000005 && new_head < 0.0000005)) {        
              heading_feedbk = new_head;
               // Serial.println(heading_feedbk,5);                                  
            }          
          }
        } 
      }
    }     
  }
}

// motorsUpdate() - updates the motors with their new state
void motorsUpdate() {
  
  if (current_Action == ADVANCE){
    float whl2 = (lin_PWM+linear_output);
    float whl3 = (lin_PWM-linear_output); 
    if (whl2 < 0) whl2 = 0;
    else if (whl2 > 255) whl2 = 255;
    if (whl3 < 0) whl3 = 0;
    else if (whl3 > 255) whl3 = 255;
    
    // Send the new PWM values to the motors
    wheel1.runPWM(0,0); 
    wheel2.runPWM(uint8_t(whl2),0);
    wheel3.runPWM(uint8_t(whl3),1);
  }
  
  else if (current_Action == ROTATE) {    
    float whl1 = (w1_output);
    float whl2 = (w2_output);
    float whl3 = (w3_output);
    
    // Send the new PWM values to the motors
    wheel1.runPWM(uint8_t(whl1),wheel_rot_dir); 
    wheel2.runPWM(uint8_t(whl2),wheel_rot_dir);
    wheel3.runPWM(uint8_t(whl3),wheel_rot_dir);
  }
}

//---------------SETUP-----------------
void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;        // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;        // Pin3,Pin11 PWM 31250Hz
  Wire.begin(8);                  // Join I2C bus as slave - address #8
  Wire.onReceive(receiveEvent);   // Function "receiveEvent" triggers when data is received from encoders
  Serial.begin(115200);           // Start serial for output

  // Set the PIDs to automatically maintain their parameters
  // i.e. stopping integral windup, etc.
  rotate_W1.SetMode(AUTOMATIC);   
  rotate_W2.SetMode(AUTOMATIC);
  rotate_W3.SetMode(AUTOMATIC);
  rotate_W1.SetSampleTime(10);   
  rotate_W2.SetSampleTime(10);
  rotate_W3.SetSampleTime(10);
  linear_CTRL.SetMode(AUTOMATIC);
  linear_CTRL.SetControllerDirection(DIRECT);
  linear_CTRL.SetOutputLimits(-255.0,255.0);
}  

//----------------LOOP-----------------
void loop() { 
  if (current_Action == ROTATE) {             // If currently rotating, compute the new PID values and update motors
    rotate_W1.Compute();
    rotate_W2.Compute();
    rotate_W3.Compute();
    motorsUpdate();
  }
  
  else if (current_Action == ADVANCE) {       // If currently advancing, compute the new PID values and update motors
    linear_CTRL.Compute();
    motorsUpdate();   
  }  
  
  else if (current_Action == STOP) {
    // Send the new PWM values to the motors
    wheel1.runPWM(0,0); 
    wheel2.runPWM(0,0);
    wheel3.runPWM(0,0);
  }
}

void receiveEvent(int bytes) {
  byte pps1_high = Wire.read();
  byte pps1_low = Wire.read();
  pps1_feedbk = double((pps1_high << 8) | pps1_low);
  byte pps2_high = Wire.read();
  byte pps2_low = Wire.read();
  pps2_feedbk = double((pps2_high << 8) | pps2_low);
  byte pps3_high = Wire.read();
  byte pps3_low = Wire.read();
  pps3_feedbk = double((pps3_high << 8) | pps3_low);
}
  
     
