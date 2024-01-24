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
upv.pvdata = {0};
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

  // pinMode(LED, OUTPUT);
  // ------------------  Multiserial  ------------------------------------

    pinMode(BT_KEY, INPUT_PULLDOWN);
    pinMode(PIN_WIFI, INPUT_PULLDOWN);
    pinMode(PIN_CONNECTED, OUTPUT);
    digitalWrite(PIN_CONNECTED, LOW);
    pinMode(UC_NRST, INPUT);

    SerialBT.begin(BT_NAME);
    UCSerial.begin(9600, SERIAL_8N1, UC_RX, UC_TX);
    UCSerial.setRxBufferSize(1024);

    CmdSerial.addInterface(&Serial);
    CmdSerial.addInterface(&SerialBT);
    CmdSerial.addInterface(&UCSerial);

    sendBuffer.reserve(MAX_SEND_BUFFER);
    commandBuffer.reserve(MAX_CMD_BUFFER);

    setupCommands();

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
}

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