//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 4. prints anything it receives to Serial.print
//
//
//************************************************************
#include "Arduino.h"
#include <painlessMesh.h>
#include <ArduinoJson.h>
#include "main.h"
#include "BluetoothSerial.h"

#include "esp_bt.h"

#include "multiserial.h"
#include "commands.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

void receivedCallback(uint32_t from, String & msg);
void reseiveUCSerial(void);
Scheduler     userScheduler;  // to control your personal task
painlessMesh  mesh;
JsonDocument  doc;            // Allocate the JSON document
String        sendMsg;

// ------------------  Multiserial  ------------------------------------
BluetoothSerial SerialBT;
HardwareSerial UCSerial(1);
MultiSerial CmdSerial;

char BT_CTRL_ESCAPE_SEQUENCE[] = {'\4', '\4', '\4', '!'};
uint8_t BT_CTRL_ESCAPE_SEQUENCE_LENGTH = sizeof(BT_CTRL_ESCAPE_SEQUENCE)/sizeof(BT_CTRL_ESCAPE_SEQUENCE[0]);

double dbTemp = 199.9;

unsigned long lastSend = 0;
unsigned long blinkLed = 0;
bool isConnected = false;
bool btKeyHigh = false;
bool wifiHigh = false;
bool btReady = false;
bool escIsEnabled = false;
String sendBuffer;
String commandBuffer;

int8_t escapeSequencePos = 0;
unsigned long lastEscapeSequenceChar = 0;

bool bridgeInit = false;
bool ucTx = false;
pvValue upv;
int indData = 0;
//----------------------------------------------------------------------

bool calc_delay = false;
SimpleList<uint32_t> nodes;

Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

void setup() {
  Serial.begin(115200);
  upv.pvdata[indData]=0;
  // pinMode(LED, OUTPUT);
  // ------------------  Multiserial  ------------------------------------

    pinMode(BT_KEY, INPUT_PULLDOWN);
    pinMode(PIN_WIFI, INPUT_PULLDOWN);
    pinMode(PIN_CONNECTED, OUTPUT);
    digitalWrite(PIN_CONNECTED, LOW);
    pinMode(UC_NRST, INPUT);

    
    UCSerial.begin(9600, SERIAL_8N1, UC_RX, UC_TX);
    UCSerial.setRxBufferSize(1024);

    CmdSerial.addInterface(&Serial);
    CmdSerial.addInterface(&UCSerial);

    sendBuffer.reserve(MAX_SEND_BUFFER);
    commandBuffer.reserve(MAX_CMD_BUFFER);

    setupCommands();

    Serial.println("Waiting for STM32 transmission.");
    //----- Wait BT_Name ----- 
    wifiHigh = true;
    while(indData<32)
    {
        if(UCSerial.available()) {
            reseiveUCSerial();
        }
    }
    sendMsg = "ISIDA-" + String(upv.pv.cellID);
    Serial.print("New name bluetooth:");
    Serial.println(sendMsg);

    SerialBT.begin(sendMsg);
    CmdSerial.addInterface(&SerialBT);

    while(CmdSerial.available()) {
        CmdSerial.read();
    }
    CmdSerial.disableInterface(&SerialBT);
    CmdSerial.disableInterface(&UCSerial);

    Serial.print("escapeIsEnabled(): ");
    Serial.println(escapeIsEnabled());

    Serial.print("monitorBridgeEnabled(): ");
    Serial.println(monitorBridgeEnabled());

    Serial.print("Serial Bridge Ready: ");
    Serial.println(BT_NAME);

    commandPrompt();

    digitalWrite(PIN_READY, HIGH);
    pinMode(PIN_READY, OUTPUT);

    digitalWrite(PIN_MONITOR, LOW);
    pinMode(PIN_MONITOR, OUTPUT);

    // Add values in the document
    doc["isida"] = upv.pv.cellID;
    // Add an array
    JsonArray data = doc["temper"].to<JsonArray>();
    for (int i=0; i<4; ++i) data.add(dbTemp);
    // Add values in the document
    doc["humid"] = upv.pv.pvRH;
    doc["minut"] = upv.pv.pvTimer;
    doc["seconds"] = upv.pv.pvTmrCount;
    doc["flap"] = upv.pv.pvFlap;
    doc["power"] = upv.pv.power;
    doc["fuses"] = upv.pv.fuses;
    doc["errors"] = upv.pv.errors;
    doc["warning"] = upv.pv.warning;
    doc["hours"] = upv.pv.hours;
    serializeJson(doc, Serial);
    Serial.println();
  //-----------------------------------------------------------------------  

  mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();

  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
      // If on, switch off, else switch on
      if (onFlag)
        onFlag = false;
      else
        onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);

      if (blinkNoNodes.isLastIteration()) {
        // Finished blinking. Reset task for next run 
        // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
        // Calculate delay based on current mesh time and BLINK_PERIOD
        // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD - 
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
      }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();

  randomSeed(analogRead(A0));
}

void loop() {
  mesh.update();
  // digitalWrite(LED, !onFlag);
  // ------------------  Multiserial  ------------------------------------
  commandLoop();

    bool _connected = SerialBT.hasClient();

    if(isConnected != _connected) {
        isConnected = _connected;

        if(isConnected) {
            Serial.println("<Client Connected>");
        } else {
            Serial.println("<Client Disconnected>");
            unescape();
        }
        digitalWrite(PIN_CONNECTED, isConnected);
    }
    if(isConnected){                    // подключен к BT горит постоянно
        digitalWrite(PIN_READY, HIGH);
    } else if(escIsEnabled){            // в режиме Esc последовательности медлено мигает
        if(millis() - blinkLed > MAX_SEND_WAIT*20){
            blinkLed = millis();
            if(btReady){
                digitalWrite(PIN_READY, HIGH);
                btReady = false;
            } else {
                digitalWrite(PIN_READY, LOW);
                btReady = true;
            }
        }
    } else if(millis() - blinkLed > MAX_SEND_WAIT*2){// в режиме моста быстро мигает
        blinkLed = millis();
        if(btReady){
            digitalWrite(PIN_READY, HIGH);
            btReady = false;
        } else {
            digitalWrite(PIN_READY, LOW);
            btReady = true;
        }
    }

    bool _btKeyHigh = digitalRead(BT_KEY) == HIGH;
    bool   wifiHigh = digitalRead(PIN_WIFI) == HIGH;
    if(btKeyHigh != _btKeyHigh) {
        btKeyHigh = _btKeyHigh;

        if(btKeyHigh) {
            Serial.println("<BtKey High>");
        } else {
            Serial.println("<BtKey Low>");
        }
    }

    if(UCSerial.available()) {
        reseiveUCSerial();
    } else if (millis() - lastSend > MAX_SEND_WAIT) {
        sendBufferNow();
    }

    if(!escapeIsEnabled()) {        // режим моста
        if(SerialBT.available()) {
            int read = SerialBT.read();

            if(read != -1) {
                if(monitorBridgeEnabled()) {
                    if(ucTx || bridgeInit == false) {
                        Serial.println();
                        Serial.print("BT> ");
                        ucTx = false;
                        bridgeInit = true;
                    }
                    Serial.print((char)read);
                }
                UCSerial.write((char)read);
                if(
                    read == BT_CTRL_ESCAPE_SEQUENCE[escapeSequencePos]
                    && (
                        millis() > (
                            lastEscapeSequenceChar + BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY
                        )
                    )
                ) {
                    lastEscapeSequenceChar = millis();
                    escapeSequencePos++;
                } else {
                    escapeSequencePos = 0;
                }
                if(escapeSequencePos == BT_CTRL_ESCAPE_SEQUENCE_LENGTH) {
                    enableEscape();
                }
            }
        }
    }
  //----------------------------------------------------------------------
}

// ------------------  Multiserial  ------------------------------------
void reseiveUCSerial(){
    int read = UCSerial.read();
    if(read != -1) {
        if(btKeyHigh) {
            // The uC is trying to send us a command; let's process
            // it as such.
            commandByte(read);
        } else if(!wifiHigh) {
            if(monitorBridgeEnabled()) {
                digitalWrite(PIN_MONITOR, HIGH);
                if(!ucTx || bridgeInit == false) {
                    Serial.println();
                    Serial.print("UC> ");
                    ucTx = true;
                    bridgeInit = true;
                }
                Serial.print((char)read);
            } else {
                digitalWrite(PIN_MONITOR, LOW);
            }

            sendBuffer += (char)read;
            if(
                ((char)read == '\n') 
                || sendBuffer.length() >= (MAX_SEND_BUFFER - 1)
            ) {
                sendBufferNow();
            }
        } else {    // передача масива для MESH
            upv.pvdata[indData] = read;
            ++indData;
            if(indData == 32) {
                indData = 0;
                doc["isida"] = upv.pv.cellID;
                for (int i=0; i<4; ++i){
                    dbTemp = (double)upv.pv.pvT[i]/10; //Присваиваем в dbTemp число и округляем его до десятых
                    doc["temper"][i] = dbTemp;
                }
                doc["humid"] = upv.pv.pvRH;
                doc["minut"] = upv.pv.pvTimer;
                doc["seconds"] = upv.pv.pvTmrCount;
                doc["flap"] = upv.pv.pvFlap;
                doc["power"] = upv.pv.power;
                doc["fuses"] = upv.pv.fuses;
                doc["errors"] = upv.pv.errors;
                doc["warning"] = upv.pv.warning;
                doc["hours"] = upv.pv.hours;
                serializeJson(doc, Serial);
                Serial.println();
            }
        }
    }
}
void sendBufferNow() {
    int sentBytes = 0;
    if(isConnected) {
        if(sendBuffer.length() > 0) {
            while(sentBytes < sendBuffer.length()) {
                sentBytes += SerialBT.write(
                    &(((const uint8_t*)sendBuffer.c_str())[sentBytes]),
                    sendBuffer.length() - sentBytes
                );
            }
        }
    }
    sendBuffer = "";
    lastSend = millis();
}
//-------------------------------------------------------------------------------

void sendMessage() {
  getReadings ();
  String msg;
  serializeJson(doc, sendMsg);
  msg = sendMsg;
  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
  mesh.sendBroadcast(msg);

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  Serial.printf("Sending message : %s\n", msg.c_str());
  
  taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
}

void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("Received message: %s from %u\n", msg.c_str(), from);
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}