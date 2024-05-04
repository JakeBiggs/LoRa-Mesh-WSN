/*
Basic LoRa Mesh Network - Jacob Biggs
Using unofficial Heltec LoRa Library found at: https://github.com/ropg/Heltec_ESP32_LoRa_v3
*/

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <map>
// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE               300

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY           866.3       // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH           250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference. 
#define SPREADING_FACTOR    9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER      0


struct Packet {
  uint64_t senderId;
  uint64_t recieverId;
  float data;
  int hopCount;

};

struct NodeInfo{
  uint64_t nodeId;
  float rssi;
  unsigned long lastBroadcastTime;
};

enum State{
  NORMAL,
  SELECTING_NODE,
  SLEEP
};

//Define routing table
std::map<uint64_t, NodeInfo> nodeList;
const int MAX_HOPS = 1;
unsigned long lastTimeoutCheck = 0;
unsigned long lastBroadcast = 0;

uint64_t chipId = 0;
String rxdata; //Recieved data buffer
size_t rxDataLength = 0;

volatile bool rxFlag = false;
float temp; 

uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

void setup() {
  // put your setup code here, to run once:

  //Same as heltec_setup()
  Serial.begin(115200);
  #ifndef HELTEC_NO_DISPLAY_INSTANCE
    heltec_display_power(true);
    display.init();
    display.setContrast(255);
    display.flipScreenVertically();
  #endif
  
  //Initialise Radio
  both.println("Radio Initialising...");
  RADIOLIB_OR_HALT(radio.begin());
  //Set the callback function for recieved packets
  radio.setDio1Action(rx);

  //Set radio parameters
  both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  //Initialise Temperature
  temp = heltec_temperature();
  both.printf("Initial Temperature: %.1f °C\n", temp);

  //Get chip ID
  chipId = getChipId();
  //Set up stuff
  unsigned long lastTimeoutCheck = millis();
  unsigned long lastBroadcast = millis();
  broadcastNode();

  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));

}

void rx(){
  rxDataLength = radio.getPacketLength();
  rxFlag = true;
}

Packet createPacket(uint64_t recvId, float data){
  Packet packet;
  packet.senderId = getChipId();
  packet.recieverId = recvId;
  packet.data = data;
  return packet;
}

String createPacketStr(uint64_t senderId, uint64_t recvId, float temperature, int hopCount){
  //char packetStr[256];
  String packet;
  if (senderId != chipId){
    packet = String(String((uint64_t)senderId)+","+String((uint64_t)recvId)+","+String(temperature,1)+","+String(hopCount));
  }else{
    String packet = String(ESP.getEfuseMac()) + "," + String((uint64_t)recvId) + "," + String(temperature,1) + "," + String(hopCount);
  }
  //sprintf(packetStr, "%llu,%llu,%.1f", senderId, recvId, temperature);
  return packet;
}

Packet decodePacket(String packetStr){
  Packet packet;
  char* token;

  //Convert packet string to char array
  char packetCharArray[255];
  packetStr.toCharArray(packetCharArray, 255);

  //Extract sender
  token = strtok(packetCharArray, ","); //Get first delimiter
  if (token == NULL) {
    both.printf("Error: Invalid packet format\n");
    return packet;
  }
  packet.senderId = strtoull(token, NULL, 10);

  //Extract reciever
  token = strtok(NULL, ","); //Split up to  second delimiter
  if (token == NULL) {
    both.printf("Error: Invalid packet format\n");
    return packet;
  }
  packet.recieverId = strtoull(token, NULL, 10);

  //Extract data
  token = strtok(NULL, ","); //Split up to final delimiter
  if (token == NULL) {
    both.printf("Error: Invalid packet format\n");
    return packet;
  }
  packet.data = atof(token);
  
  //Extract hop count
  token = strtok(NULL, ",");
  if (token == NULL){
    both.printf("Error: Invaild packet format\n");
    return packet;
  }
  packet.hopCount = atoi(token);

  return packet;
}

int broadcastNode()
{
  //Broadcasts a packet to all nodes
  //Returns 0 if successful, -1 if failed

  //Create packet
  //Receive ID of 0 its a broadcast
  String packetStr = createPacketStr(chipId, 0, 0.0, 0);

  //Transmit packet
  RADIOLIB(radio.transmit(String(packetStr).c_str()));
  both.printf("\n->Broadcasting To Network...\n");
  if (_radiolib_status == RADIOLIB_ERR_NONE)
  {
    both.printf("\n->Broadcast OK\n");
    return 0;
  }else{
    both.printf("\n->Broadcast Fail :(\n");
    return -1;
  
  }
}

int handlePacket(Packet packet){
  //Handle recieved packet
  //Returns 0 if successful, -1 if failed
  if(packet.recieverId == 0){
    updateNodeList(packet.senderId, radio.getRSSI());
  }
  else if(packet.recieverId == chipId){
    both.printf("RX From [%s]\n", String(packet.senderId).c_str());
    //both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
    //both.printf("  SNR: %.2f dB\n", radio.getSNR());

    both.printf("Temperature recieved: %.1f \n", packet.data);
  }
  else if(packet.recieverId != chipId && packet.recieverId != 0){
    both.printf(String("\nRecieved packet that wasnt meant for you").c_str());
    //TODO: handle mesh routing
  }
  else{
    both.printf("Error: Invalid packet\n");
    return -1;
  }
  return 0;
}

void updateNodeList(uint64_t nodeId, float rssi){
  //Updates the list of nodes in the network
  NodeInfo node;
  node.nodeId = nodeId;
  node.rssi = rssi;
  node.lastBroadcastTime = millis();

  //Add node to list or overwrite existing node info
  nodeList[nodeId] = node;
}

void removeTimeoutNodes(unsigned long timeout){
  //Removes nodes from the list that have not broadcasted in a while
  for (auto it = nodeList.begin(); it != nodeList.end();){
    if(millis() - it->second.lastBroadcastTime > timeout){
      it = nodeList.erase(it); //Erase node if timeout and increment iterator
    }else{
      it++; //Increment iterator if not erased
    }
  }
}

void routePacket(Packet packet){
   //Routes a packet to the correct node
  if (nodeList.find(packet.recieverId) != nodeList.end()){
    //Node exists in list
    if (packet.hopCount < MAX_HOPS){
      //Packet has not reached maximum hops
      packet.hopCount++;
      both.printf("Routing packet to [%s]\n", String(packet.recieverId).c_str());
      String packetStr = createPacketStr(packet.senderId, packet.recieverId, 0.0, packet.hopCount);
      RADIOLIB(radio.transmit(String(packetStr).c_str()));

    }else{
      both.printf("Packet has reached maximum hops, discarding\n");
    }
  }
  
}

String getChipIdStr(){
  uint64_t chipId = ESP.getEfuseMac(); //Unique chip id (essentially mac address)
  char chipIdStr[17]; // Buffer to hold the chip ID string and null terminator
  sprintf(chipIdStr, "%016llX", chipId); // Convert the chip ID to a string
                                        //% format specifier
                                        //016: minimum field width
                                        //ll: same as long long int (uint64_t)
                                        //x: formatted as unsigned hex int 
  return String(chipIdStr);
}

uint64_t getChipId(){
  uint64_t chipId = ESP.getEfuseMac();
  return chipId;
}



void loop() {
  //updates the state of the power button and implements long-press power off if used.
  //Functions same as heltec_loop()
  button.update();
  #ifdef HELTEC_POWER_BUTTON
    // Power off button checking
    if (button.pressedFor(1000)) {
      #ifndef HELTEC_NO_DISPLAY_INSTANCE
        // Visually confirm it's off so user releases button
        display.displayOff();
      #endif
      // Deep sleep (has wait for release so we don't wake up immediately)
      heltec_deep_sleep();
    }
  #endif

  //TODO: Add maximum duty cycle limits?
  if ((button.isSingleClick())){
    temp = heltec_temperature();
    both.printf("TX Temp: %.1f °C", temp);


    radio.clearDio1Action();
    heltec_led(50); //50% Brightness
    tx_time = millis();
    //Create packet
    String packetStr = createPacketStr(chipId, 0, temp, 0);
    //both.printf("\nPS->%s", packetStr.c_str());
    //both.printf("\n->%llu,%llu,%.1f", packet.senderId, packet.recieverId, packet.data);
    //delay(1000);
    RADIOLIB(radio.transmit(String(packetStr).c_str()));
    tx_time = millis() - tx_time;
    heltec_led(0); //Turn off LED
    if (_radiolib_status == RADIOLIB_ERR_NONE) {
      both.printf("\n->TX OK (%i ms)\n", tx_time);
    } else {
      both.printf("\n->TX fail (%i)\n", _radiolib_status);
    }

    last_tx = millis();
    radio.setDio1Action(rx);
  
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  }
  //On packet recieved
  if(rxFlag){
    rxFlag = false;
    rxDataLength = radio.getPacketLength();
    //both.printf("\nRX Data Length: %i\n", rxDataLength);
    
    both.printf("Reading data...\n");
    radio.readData(rxdata);
    
    //recvPacketCharArray[rxDataLength] = '\0'; //Null terminator
    if (_radiolib_status == RADIOLIB_ERR_NONE){
      //both.printf("RCV->%s\n", rxdata.c_str());
      both.printf("Decoding Packet...\n");
      Packet decodedPacket = decodePacket(rxdata);



      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    }
  }
  //Remove nodes that have not broadcasted 
  if (millis() - lastTimeoutCheck > 120000){ //Check every 2 minutes
    removeTimeoutNodes(900000); //Remove nodes that have not broadcasted in 15 minutes
    lastTimeoutCheck = millis();
  }
  
  //Broadcast node every 5 minutes
  if (millis() - lastBroadcast > 300000){
    broadcastNode();
    lastBroadcast = millis();
  }
}

