#include <Wheel.h>

#define PWM_VALS 3    // Number of chars needed for PWM values 
#define TIME_CHARS 2  // Number of chars needed to represent execution time 
#define MBUFSIZE 8   // Set size for the motor instruction buffer
#define STRT_MSG 17   // Start of message character
#define END_MSG 19    // End of message character

// ------ Motor instruction typedef ------
typedef struct motors_data {  
   uint8_t pwm_values[PWM_VALS]; 
   bool pwm_dirs[PWM_VALS];
   uint32_t time_secs;
   uint32_t time_nsecs;
  }motors_data;

volatile motors_data motors_Buff[MBUFSIZE]; // The motor instruction buffer
volatile uint8_t mtBufFront = 0;            // Index for the next executed instruction in the buffer 
volatile uint8_t mtBufOpen = 0;             // The next free index in the buffer
volatile uint8_t mtBufCnt = 0;              // The number of instructions currently in the buffer
uint8_t event_Capture[12];                  // Temporary holding for incoming instructions
volatile bool Instruction_ready = false;    // 'True' if an instruction is queued in the buffer

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
// Side note: 'for' loops are usually bad news in an interrupt, however, these three run only 3, 3, and 4 loops respectively.
void serialEvent() {
  uint8_t isStart = Serial.read();
  if (isStart == STRT_MSG) {
    Serial.readBytesUntil(END_MSG, event_Capture, 13);
    uint8_t i = 0; uint8_t j = 0;
      for (; j<PWM_VALS; i++,j++) {
        motors_Buff[mtBufOpen].pwm_values[j] = uint8_t(event_Capture[i]); // Add new motor PWM values into the motor instruction buffer
      }
      for (j=0; j<PWM_VALS; i++,j++) {
        motors_Buff[mtBufOpen].pwm_dirs[j] = bool(event_Capture[i]);      // Add new direction values into motor buffer
      } 
      motors_Buff[mtBufOpen].time_secs = event_Capture[i++];              // Set seconds for instruction execution
      motors_Buff[mtBufOpen].time_nsecs = event_Capture[i];               // Set nanoseconds for instruction execution
      
     mtBufOpen++;                     // Increment to the next open index in the motor buffer 
     mtBufOpen = mtBufOpen%MBUFSIZE;  // Modulo the open index with buffer size to wrap index back to front if it is at the last index
     mtBufCnt++;                      // Increment motor buffer instruction counter
     Instruction_ready = true;        // An instruction is ready to be executed
  }  
}

//motorsUpdate() - updates the motors with their new state received from the rPi
void motorsUpdate() {
  wheel1.runPWM(motors_Buff[mtBufFront].pwm_values[0],motors_Buff[mtBufFront].pwm_dirs[0]); 
  wheel2.runPWM(motors_Buff[mtBufFront].pwm_values[1],motors_Buff[mtBufFront].pwm_dirs[1]);
  wheel3.runPWM(motors_Buff[mtBufFront].pwm_values[2],motors_Buff[mtBufFront].pwm_dirs[2]);  
  mtBufFront++;                       // Increment the front index to next instruction of the motor buffer 
  mtBufFront = mtBufFront%MBUFSIZE;   // Modulo index with buffer size to wrap index back to front
  mtBufCnt--;                         // Decrement motor buffer instruction counter
  if (mtBufCnt == 0)                  // If the motor buffer is empty 
    Instruction_ready = false;
}

//---------------SETUP-----------------
void setup() {
  TCCR1B=TCCR1B&0xf8|0x01;            // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;            // Pin3,Pin11 PWM 31250Hz
  Serial.begin(115200);               // High baudrate for quick serial messaging
  wheel1.resetCurrPulse();
  wheel2.resetCurrPulse();
  wheel3.resetCurrPulse();
}  

//----------------LOOP-----------------
void loop() {
  if (Instruction_ready == true) { // If there is an instruction to execute in the buffer
      motorsUpdate();
  }
}

  


     
