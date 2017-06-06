/*
 *  Arduino_encoder - Encoder data retrieval and forwarding
 *    by Tyler Reese  
 *    
 */
 
#include <Wire.h>

#define ENCODER1A   18          // Supports interrupts (Interrupt 5)
#define ENCODER1B   22
#define ENCODER2A   19          // Supports interrupts (Interrupt 4)
#define ENCODER2B   24
#define ENCODER3A   2           // Supports interrupts (Interrupt 0)
#define ENCODER3B   26
#define MICROS_PER_SEC 1000000  // Microseconds in 1 second
#define PPS_ZERO_OUT 20000      // Duration in microseconds before an unchanged pps is set to zero

// Pulse and PPS calculation variables
volatile uint32_t prevmicr1 = 0;    
volatile uint32_t prevmicr2 = 0;
volatile uint32_t prevmicr3 = 0;
volatile uint32_t micr1 = 0;
volatile uint32_t micr2 = 0;
volatile uint32_t micr3 = 0;
volatile int16_t pps1 = 0;
volatile int16_t pps2 = 0;
volatile int16_t pps3 = 0;
volatile bool strt_pls_1 = true;
volatile bool strt_pls_2 = true;
volatile bool strt_pls_3 = true;
volatile long encoder1 = 0;
volatile long encoder2 = 0;
volatile long encoder3 = 0;
long prevenc1 = 0;
long prevenc2 = 0;
long prevenc3 = 0;
long W1 = 0;
long W2 = 0;
long W3 = 0;
int check_sum = 0;
int send_count = 0;


//---------------SETUP-----------------
void setup() {
  Wire.begin();                            // Join I2C bus as Master
  Serial.begin(115200);                    // Initialize serial with high baudrate
  
  // Input setup to receive the encoder pulses
  pinMode(ENCODER1A, INPUT);
  pinMode(ENCODER1B, INPUT);
  pinMode(ENCODER2A, INPUT);
  pinMode(ENCODER2B, INPUT);
  pinMode(ENCODER3A, INPUT);
  pinMode(ENCODER3B, INPUT);

  // Attach interrupts to each encoder 
  attachInterrupt(5, enc1_isr, RISING);
  attachInterrupt(4, enc2_isr, RISING);
  attachInterrupt(0, enc3_isr, RISING); 

  // Timer interrupt that overflows every millisecond
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}  

//----------------------LOOP------------------------
void loop() {
  send_pps();
  delay(0.001);
}
//--------------------------------------------------

// Interrupt for encoder Motor 1
void enc1_isr() { 
  //Read Inputs
  int a = digitalRead(ENCODER1A);
  int b = digitalRead(ENCODER1B);
  micr1 = micros();
  if (strt_pls_1 == false && micr1 > prevmicr1) {
    pps1 = MICROS_PER_SEC/(micr1 - prevmicr1);
    if (a == b){
      pps1 *= -1;
    }
  }else strt_pls_1 = false;
  prevmicr1 = micr1;
  if (a == b) {                     // B is leading A(counter-clockwise)
    encoder1--;
  } 
  else {                            // A is leading B(clockwise)
    encoder1++;
  } 
}

// Interrupt for encoder Motor 2
void enc2_isr() {
  //Read Inputs
  int a = digitalRead(ENCODER2A);
  int b = digitalRead(ENCODER2B);
  micr2 = micros();
  if (strt_pls_2 == false && micr2 > prevmicr2) {
    pps2 = MICROS_PER_SEC/(micr2 - prevmicr2);   
    if (a == b){
      pps2 *= -1;
    }
  }else strt_pls_2 = false;
  prevmicr2 = micr2;
  if (a == b) {                     // B is leading A(counter-clockwise)
    encoder2--;
  } 
  else {                            // A is leading B(clockwise)
    encoder2++;
  }
}

// Interrupt for encoder Motor 3
void enc3_isr() {
  //Read Inputs
  int a = digitalRead(ENCODER3A);
  int b = digitalRead(ENCODER3B);
  micr3 = micros();
  if (strt_pls_3 == false && micr3 > prevmicr3) {
    pps3 = MICROS_PER_SEC/(micr3 - prevmicr3);
    if (a == b){                    // B is leading A(counter-clockwise)
      pps3 *= -1;
    }
  }else strt_pls_3 = false;
  prevmicr3 = micr3;
  if (a == b) {                     // B is leading A(counter-clockwise)
    encoder3--;
  } 
  else {                            // A is leading B(clockwise)
    encoder3++;
  }
}

// Interrupt called every 1ms:
//    - sends encoder counts to ROS network (sends every ~10ms)
//    - sets pps of wheels that are not moving to 0 (checks every ~20ms)
SIGNAL(TIMER0_COMPA_vect) {
  if (send_count == 10){
    W1 = encoder1;
    W2 = encoder2;
    W3 = encoder3;
    
    // Basic check to ensure encoder values haven't jumped, unrealistically, from previous values, skewing position data 
    if (((abs(W1-prevenc1) < 100) && (abs(W2-prevenc2) < 100)) && (abs(W3-prevenc3) < 100)) {
      check_sum = (abs(W1 + W2 + W3) % 1000);
      Serial.print(String(W1) + "," + String(W2) + "," + String(W3) + "," + String(check_sum) + "\n");
      prevenc1 = W1;
      prevenc2 = W2;
      prevenc3 = W3;
      send_count = 0;
    } 
  }else {
    send_count++;
    prevenc1 = W1;
    prevenc2 = W2;
    prevenc3 = W3;
  }
  // Set wheel 1,2,3 pps to 0 if pulse count is unchanged after PPS_ZERO_OUT microseconds
  if (micros() - prevmicr1 > PPS_ZERO_OUT) pps1 = 0;
  if (micros() - prevmicr2 > PPS_ZERO_OUT) pps2 = 0;
  if (micros() - prevmicr3 > PPS_ZERO_OUT) pps3 = 0;
}

// Sends pps data every loop over I2C to Motor Arduino
void send_pps() {
  byte byte_array[6];
  byte_array[0] = (pps1 >> 8) & 0xff;     // Bit shift 8 right to 
  byte_array[1] = pps1 & 0xff;
  byte_array[2] = (pps2 >> 8) & 0xff;
  byte_array[3] = pps2 & 0xff;
  byte_array[4] = (pps3 >> 8) & 0xff;
  byte_array[5] = pps3 & 0xff;
  
  Wire.beginTransmission(8);              // Transmit to device #8
  Wire.write(byte_array, 6);              // Send bytes
  Wire.endTransmission();                 // Stop transmitting
}





  


     
