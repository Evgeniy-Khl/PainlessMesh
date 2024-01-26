#ifndef main_h
#define main_h

#include "Arduino.h"
#include "BluetoothSerial.h"
#include "multiserial.h"

// some gpio pin that is connected to an LED...
// on my rig, this is 5, change to the right number of your LED.
// #ifdef LED_BUILTIN
//     #define LED LED_BUILTIN
// #else
//     #define LED 2
// #endif
// ------------------  Multiserial  ------------------------------------
// This is the name that this device will appear under during discovery
#define BT_NAME "esp32-bridge"
// At least this number of ms must elapse between each
// character of the escape sequence for it to be counted; this
// is done to prevent legitimate occurrences of the escape sequence
// occurring in binary causing us to enter the bridge mode.
#define BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY 250
// For communicating with the microcontroller, we will transmit and
// receive on the following pins.  Do not set these to match the pins
// used by defualt for the ESP32 unit's UART1 (IO35 and IO34).  I've also
// heard, but not confirmed, that the IO pin used for TX must be a higher
// IO number than the pin used for RX.  If you can confirm or deny this,
// I'd love to hear a definitive answer.
#define UC_TX 27
#define UC_RX 14
// This pin will be pulled HIGH (if defined) when the device is ready for connections
#define PIN_READY 2
#define PIN_MONITOR 5
#define PIN_MESH_START 18
// This pin will be pulled HIGH when a client is connected over bluetooth.
#define PIN_CONNECTED 4
// If your microcontroller pulls this pin HIGH, it can send commands directly to the ESP32 unit
#define BT_KEY 16
// Connect this to your microcontoller's reset line to allow you to reset your microcontoller at-will.
#define UC_NRST 17
#define MAX_SEND_WAIT 50
#define MAX_CMD_BUFFER 128
#define MAX_SEND_BUFFER 128
//------------------------------------------------------------------------

#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for

#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

// Prototypes
void sendMessage(); 

void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

union pvValue{
  uint8_t pvdata[32];
  struct rampv {
   uint16_t cellID;               // 2 байт ind=0         сетевой номер и тип прибора
    int16_t pvT[4];               // 8 байт ind=2-ind=9   значения [MAX_DEVICES] датчиков температуры
    int16_t pvRH;                 // 2 байт ind=10;ind=11 значение датчика относительной влажности
    int16_t pvCO2[3];             // 6 байт ind=12-ind=17 значения датчика CO2 ---------- ИТОГО 18 bytes ------------
    uint8_t pvTimer;              // 1 байт ind=18        значение таймера
    uint8_t pvTmrCount;           // 1 байт ind=19        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t pvFlap;               // 1 байт ind=20        положение заслонки 
    uint8_t power;                // 1 байт ind=21        мощность подаваемая на тены
    uint8_t fuses;                // 1 байт ind=22        короткие замыкания 
    uint8_t errors, warning;      // 2 байт ind=23;ind=24 ошибки и предупреждения
    uint8_t cost0, cost1;         // 2 байт ind=25;ind=26 затраты ресурсов
    uint8_t date, hours;          // 2 байт ind=27;ind=28 счетчики суток и часов
    uint8_t other0;               // 1 байт ind=29        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t other1;               // 1 байт ind=29        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t other2;               // 1 байт ind=29        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
  } pv;// ------------------ ИТОГО 30 bytes -------------------------------
} ;

extern MultiSerial CmdSerial;
extern HardwareSerial UCSerial;
extern BluetoothSerial SerialBT;

#endif /* _ESP32_CORE_MAIN_H_ */