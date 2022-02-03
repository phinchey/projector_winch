/*
Demonstrates simple RX and TX operation.
Any of the Basic_RX examples can be used as a receiver.
Please read through 'NRFLite.h' for a description of all the methods available in the library.
Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND
*/

#include "SPI.h"
#include "NRFLite.h"

const static uint8_t RADIO_ID = 1;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 0; // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 6;
const static uint8_t PIN_RADIO_CSN = 5;

const static uint8_t BUTTON_UP_GND = A0;
const static uint8_t BUTTON_UP_IN = A1;
const static uint8_t BUTTON_DN_GND = A2;
const static uint8_t BUTTON_DN_IN = A3;


struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t command;
//    uint32_t OnTimeMillis;
//    uint32_t FailedTxCount;
};

NRFLite _radio;
RadioPacket _radioData;


void setup()
{
    Serial.begin(9600);
    
    // By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
    // Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
    // You can run the 'ChannelScanner' example to help select the best channel for your environment.
    // You can assign a different bitrate and channel as shown below.
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE2MBPS, 100) // THE DEFAULT
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE1MBPS, 75)
    //   _radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, 0)

    pinMode(BUTTON_UP_GND, OUTPUT);
    digitalWrite(BUTTON_UP_GND, LOW);
    pinMode(BUTTON_UP_IN, INPUT_PULLUP);
    pinMode(BUTTON_DN_GND, OUTPUT);
    digitalWrite(BUTTON_DN_GND, LOW);
    pinMode(BUTTON_DN_IN, INPUT_PULLUP);
    
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE250KBPS, 0))
    {
        Serial.println("Cannot communicate with radio");
    }
}

void loop()
{
    _radioData.command = 0;

    bool button_up = !digitalRead(BUTTON_UP_IN);
    bool button_dn = !digitalRead(BUTTON_DN_IN);
    static int previous_state = 0;
    int state;

    if(button_up && !button_dn){
      state = 1;
    }
    else if(button_dn && !button_up){
      state = 2;
    }
    else if(button_up && button_dn){
      state = 3;
    }
    else{
      state = 0;
    }


    
    if(previous_state != 0 || state != 0){
      previous_state = state;
      _radioData.command = state;
      Serial.println(state);
      if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) // Note how '&' must be placed in front of the variable name.
      {
        Serial.println("success");
      }
      else
      {
        Serial.println("fail");
      }
      _radio.powerDown(); // put radio into standby after transmission to conserve power
    }

    previous_state = state;
    delay(100);

}
