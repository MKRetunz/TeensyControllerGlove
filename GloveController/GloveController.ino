/* 
 * Connection of MPU9150 to Unity,
 * We use the DMP funrctions to obtain the rotation of the MPU9150.
 * 
 * Adapted by Gidi van Liempd (gidi@geedesign.com) July 2016
 * Gidi: This is based on example MPU6050_DMP6 (can be found in https://github.com/sparkfun/MPU-9150_Breakout/tree/master/firmware )
 * combined with (my) sketch "testCOnnection", which uses code for a direct connection to Unity, 
 * 
 * Note: since we use the serial connection to Unity, most serial.print function calls in the loop 
 * from the original example must be commented out!!
*/
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_QUATERNION

#define LED_PIN 13 // (Arduino is 13, Teensy2.0 is 11, Teensy2.0++ is 6)
bool blinkState = false;

#define INTERRUPT_PIN  2  // Gidi: added for use with Teensy 3.1, where any pin can be an interrupt
                          // In this case, we make pin 2 the interrupt pin

#define GIVE_INPUT     'a'  // the byte signalling that the computer wants input
#define ISSUE_COMMAND  'b'  // the byte signalling that the computer issues a command
#define SET_LED_ON     'x'
#define SET_LED_OFF    'y'
#define DEAD_X 5
#define DEAD_Y 5
#define MAX_MOVE_X 60
#define MAX_MOVE_Y 40
#define ALPHA 0.5

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

const int LMOUSE_PIN = 22;  // Pin for the left mouse button
const int ESC_PIN = 21;     // Pin for the escape button

int prev_delta_x, prev_delta_y;
int LmouseState = 0;
int EscState = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    pinMode(INTERRUPT_PIN, INPUT); // Gidi: sets the digital pin as input,  necessary for the interrupt
    pinMode(LMOUSE_PIN, INPUT); 
    pinMode(ESC_PIN, INPUT);
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(9600);
    while (!Serial); 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    delay(50);
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  int diff_x, diff_y;     // difference between new and recorded position
  float delta_x, delta_y; // difference used for smoothing 
  
    if (!dmpReady) 
      while (!mpuInterrupt && fifoCount < packetSize) {
        if(Serial.available()) {
          char inByte = Serial.read();
        }    
      }

     LmouseState = digitalRead(LMOUSE_PIN);
     EscState = digitalRead(ESC_PIN);
    if (LmouseState == LOW) {      
      Mouse.click();
      Mouse.set_buttons(1, 0, 0);
    } 
    else {
      Mouse.set_buttons(0, 0, 0);
    }
    if (EscState == LOW) {
      Keyboard.set_key1(KEY_ESC);
      Keyboard.send_now();
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);

        if (abs(diff_x) < DEAD_X) diff_x = 0;
        if (abs(diff_y) < DEAD_Y) diff_y = 0;
        /*Serial.print("quat\t");
        Serial.print(q.w);         
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);*
    }
    delay(50);
}
