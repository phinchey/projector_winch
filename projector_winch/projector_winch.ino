#include <Adafruit_BNO055.h>
#include <Adafruit_VL6180X.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#define CEILING_DISTANCE 100 //distance to ceiling in mm
int P = 30; //Proportional gain of motor speed differential per degree of tilt


// Motor controller pins
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

#include "SPI.h"
#include "NRFLite.h"


// NRF24L01 setup
const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t PIN_RADIO_CE = 3;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t command;
//    uint32_t OnTimeMillis;
//    uint32_t FailedTxCount;
};

NRFLite _radio;
RadioPacket _radioData;


// Motor controller setup
// Used to configure the motor direction
const int offsetA = 1;
const int offsetB = -1;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Rangefinder setup
Adafruit_VL6180X vl = Adafruit_VL6180X();

// IMU Setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
float tilt = 0;
float accel = 0;

//EEPROM setup
float offset;
int eeprom_address = 0;

//State machine
enum state_machine {
  idle,
  reelup,
  reeldown,
  adjusting_cw,
  adjusting_ccw,
  idle_after_adjust,
  idle_before_adjust,
  dance
};

state_machine state = idle;


void setup() {

  Serial.begin(9600);
  Serial.println("Serial initialised");
  delay(100);

  // Rangefinder setup
  if (! vl.begin()) {
    Serial.println("Failed to find rangefinder");
    delay(100);
  }

  // Radio setup
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, 0))
    {
        Serial.println("Cannot communicate with radio");
    }

  //IMU setup
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  Serial.println("Setup complete");
  delay(100);

  //load offset from EEPROM
  byte byte1 = EEPROM.read(eeprom_address);
  byte byte2 = EEPROM.read(eeprom_address + 1);
  int int_offset = (byte1 << 8) + byte2;
  offset = float(int_offset)/100;
  Serial.println("Offset:" + String(offset));
}

void loop() {
  static unsigned long radio_heartbeat; //only used in active modes (adjust)
  static unsigned long adjust_timer;

  // Radio stuff
  if (_radio.hasData())
    {
        radio_heartbeat = millis();
        _radio.readData(&_radioData); // Note how '&' must be placed in front of the variable name.
        uint8_t msg = _radioData.command;


        switch (msg) {
          case 1:
            if(state == idle || state == reelup){
              state = reeldown;
            }
            else if(state == adjusting_ccw || state == idle_after_adjust){
              state = adjusting_cw;
              adjust_timer = millis();
            }
            break;
          case 2:
            if(state == idle || state == reeldown){
              state = reelup;
            }
            else if(state == adjusting_cw || state == idle_after_adjust){
              state = adjusting_ccw;
              adjust_timer = millis();
            }
            break;
          case 3:
            if(state != idle_before_adjust && state != dance){
              state = idle_before_adjust;
              adjust_timer = millis();
            }
            else if(state == idle_before_adjust){
              if(millis() - adjust_timer > 3e3){
                state = dance;
              }
            }
            break;
          case 0:
            if(state == reelup || state == reeldown){
//              state = idle;
            }
            else if(state == adjusting_cw || state == adjusting_ccw){
              state = idle_after_adjust;
              adjust_timer = millis();
            }
            else if(state == idle_before_adjust){
              state = idle;
            }
            break;
        }
        print_state();
    }

// Motor stuff

  static unsigned long last_motor_loop;
  if(millis() - last_motor_loop > 100){
//    print_state();
    static const int REEL_SPD = 200;
    
    if(state == reelup || state == reeldown){
      int motor_dir = 1; //down
      if(state == reelup) motor_dir = -1; //up

      if(state == reeldown and accel < -1.0){
//        state = idle;
      }
      else{
  
        int motor_spd = REEL_SPD * motor_dir;
        int motor_l = motor_spd + ((int)tilt - offset) * P;
        int motor_r = motor_spd - ((int)tilt - offset) * P;
  
        //constrain motor values to < REEL_SPD
        if(motor_l*motor_dir > REEL_SPD){
          motor_r = 2 * motor_dir * REEL_SPD - motor_l;
          motor_l = REEL_SPD * motor_dir;
        }
        if(motor_r*motor_dir > REEL_SPD){
          motor_l = 2 * motor_dir * REEL_SPD - motor_r;
          motor_r = REEL_SPD * motor_dir;
        }
//        if(motor_l < REEL_SPD *-1 && motor_dir == -1){
//          motor_r = -2 * REEL_SPD - motor_l;
//          motor_l = REEL_SPD *-1;
//        }
//        if(motor_r < REEL_SPD *-1 && motor_dir == -1){
//          motor_l = -2 * REEL_SPD - motor_r;
//          motor_r = REEL_SPD *-1;
//        }


        // don't let a motor spin backwards
        if(motor_dir == 1){
          if(motor_r < 0){
            motor_r = 0;
          }
          if(motor_l < 0){
            motor_l = 0;
          }
        }
        else{
          if(motor_r > 0){
            motor_r = 0;
          }
          if(motor_l > 0){
            motor_l = 0;
          }
        }
        
        motor1.drive(motor_l);
        motor2.drive(motor_r);
      }

    }
    
    if(state == idle or state == idle_after_adjust or state == idle_before_adjust){
      motor1.standby();
      motor2.standby();
    }


    if(state == adjusting_cw || state == adjusting_ccw){

      if(millis() - radio_heartbeat > 1e3){
        state = idle_after_adjust;
        adjust_timer = millis();
      }
      
      int motor_spd = 112;

      if(state == adjusting_cw){
        motor1.drive(motor_spd);
        motor2.drive(-1 * motor_spd);
      }
      else{
        motor1.drive(-1 * motor_spd);
        motor2.drive(motor_spd);
      }
    }


    if(state == idle_after_adjust){

      if(millis() - adjust_timer > 5e3 && state == idle_after_adjust){

        int number = int(tilt*100);
        byte byte1 = number >> 8;
        byte byte2 = number & 0xFF;
        EEPROM.update(eeprom_address, byte1);
        EEPROM.update(eeprom_address + 1, byte2);
        Serial.println("Offset:" + String(number));

        state = idle;
        adjust_timer = millis();
      }
    }


    if(state == dance){ //do a little dance to show we're in adjust mode
      int motor_spd = -255;
      int motor_l = motor_spd;
      int motor_r = motor_spd;
      
      motor1.drive(motor_l);
      motor2.drive(motor_r);

      delay(2000);

      motor_spd = 255;
      motor_l = motor_spd;
      motor_r = motor_spd;
      
      motor1.drive(motor_l);
      motor2.drive(motor_r);

      delay(1000);

      motor1.brake();
      motor2.brake();

      state = idle_after_adjust;
      adjust_timer = millis();
      

      while(_radio.hasData()){
        RadioPacket garbage;
        _radio.readData(&garbage); //flush radio data that accumulated while delaying
      }
      
    }
    
    
    last_motor_loop = millis();
  }

  
  // Rangefinder stuff
  if(state == reelup || state == reeldown || state == adjusting_cw || state == adjusting_ccw || state == dance){
    static unsigned long last_vl_loop;
    static bool was_limited = true;
    
    if(millis() - last_vl_loop > 100){
      uint8_t range = vl.readRange();
      uint8_t rf_status = (vl.readRangeStatus());

      if(rf_status == 0){
        if(range < CEILING_DISTANCE){ 
          state = idle;
        }
      }

      last_vl_loop = millis();
      was_limited = rf_status;
    }
  }

  // IMU stuff
  if(state != idle){
    static unsigned long last_imu_loop;
    
    if(millis() - last_imu_loop > 100){
      sensors_event_t event;
      bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
      tilt = event.orientation.y;
      bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
      accel = event.acceleration.z;
      last_imu_loop = millis();
    }
  }
}

void print_state(){
  switch(state){
    case idle:
      Serial.print("idle");
      break;
    case reelup:
      Serial.print("reelup");
      break;
    case reeldown:
      Serial.print("reeldown");
      break;
    case adjusting_cw:
      Serial.print("adjusting_cw");
      break;
    case adjusting_ccw:
      Serial.print("adjusting_ccw");
      break;
    case idle_after_adjust:
      Serial.print("idle_after_adjust");
      break;
    case idle_before_adjust:
      Serial.print("idle_before_adjust");
      break;
    case dance:
      Serial.print("dance");
      break;
  }
  Serial.print('\t');
  Serial.println(tilt);
}
