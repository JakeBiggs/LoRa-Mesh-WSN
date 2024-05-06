/*
Basic LoRa Mesh Network - Jacob Biggs
Using unofficial Heltec LoRa Library found at: https://github.com/ropg/Heltec_ESP32_LoRa_v3
*/

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <map>
#include <WiFi.h>
//#include <MySQL_Connection.h>
//#include <MySQL_Cursor.h>
#include <MySQL.h>
//Set your SSID and Password Here:
const char* ssid = "XXXXX";
const char* password = "XXXXX";
bool WifiEnabled = 1; //Disable wifi here

//Define MySQL login and password
WiFiClient client;
//MySQL_Connection conn((Client*)&client);
const char* user = "lora";                   // MySQL user login username
const char* pass = "XXXXX";           // MySQL user login password
const char* dbHost = "XXXX";            // MySQL hostname or IP address
const char* database = "loradata";         // Database name
const char* table = "nodeInfo";               // Table name
uint16_t dbPort = 3306;                        // MySQL host port
uint32_t pollTime = 5000;  
MySQL sql(&client, dbHost, dbPort);
#define MAX_QUERY_LEN 128

bool SQL_connection_status;
//char* usera = "lora";
//char* pass = "1234";
char* db = "loradata";

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
unsigned long BROADCAST_TIME = 60000;//1 minute    300000; //5 minutes
unsigned long lastBroadcast = 0;
unsigned long lastSQLBroadcast =0;
bool firstLoop;


uint64_t chipId = 0;
String rxdata; //Recieved data buffer
size_t rxDataLength = 0;

volatile bool rxFlag = false;
float temp; 

uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

State current_state = NORMAL;
State prevState = NORMAL;
static bool justSwitched = false;
int currentNodeIndex = 0;
int numNodes = 0;

void setup() {
  heltec_setup();
  Serial.println(); 
  // put your setup code here, to run once:
  /*
  //Same as heltec_setup()
  Serial.begin(115200);
  #ifndef HELTEC_NO_DISPLAY_INSTANCE
    heltec_display_power(true);
    display.init();
    display.setContrast(255);
    display.flipScreenVertically();
  #endif
  */

  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);  
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
  delay(1000);
  display.cls();

  if (WifiEnabled){
    // Connect to WiFi
    both.print("Connecting to Wifi...\n");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      both.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED){
      both.print("\nConnected:\n" + String(WiFi.localIP().toString()+"\n"));  
      // Connect to MySQL
   //   both.print("Connecting to "+ dbHost);
      //IPAddress server_addr(192,168,1,148);  // IP of the MySQL *server* here

      delay(500);
      if (sql.connect(user,pass, database)){//conn.connect(server_addr, 3306, user, pass)) {
        delay(1000);
        
        both.print("Connected to MySQL\n");
        //Create a cursor
        //MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);
        SQL_connection_status = true;
        delay(2000);

      } else {
        both.print("Connection failed.\n");
        SQL_connection_status = false;
        delay(3000);
      }
      
    }
  } 
  delay(2000);
  clearDisplay();
  //Initialise Temperature
  temp = heltec_temperature();
  both.printf("Current Temp: %.1f °C\n", temp);
  both.printf("Packet Log:\n");
  display.display();
  //Get chip ID
  chipId = getChipId();
  //Set up stuff
  unsigned long lastTimeoutCheck = millis();
  unsigned long lastBroadcast = -300000;

  // Start receiving
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  firstLoop = true;
}

void loop() {
  //updates the state of the power button and implements long-press power off if used.
  //Functions same as heltec_loop()
  button.update();
  #ifdef HELTEC_POWER_BUTTON


    // Power off button checking
    if (button.pressedFor(50000)) {
      #ifndef HELTEC_NO_DISPLAY_INSTANCE
        // Visually confirm it's off so user releases button
        display.displayOff();
      #endif
      // Deep sleep (has wait for release so we don't wake up immediately)
      heltec_deep_sleep();
    }
  #endif

  if(firstLoop){
    broadcastNode();
    if (SQL_connection_status){
      display.print("SQL Connected\n");
    }
    if (WifiEnabled && SQL_connection_status){
      broadcastSQL();
    }
    firstLoop=false;
  }

  //Broadcast node every 1 minute
  if (millis() - lastBroadcast > BROADCAST_TIME && current_state != SELECTING_NODE){ //1min
    broadcastNode();
    lastBroadcast = millis();
  }
  //Upload temperature to database every 5 minutes
  if (millis() - lastSQLBroadcast > 300000 && WifiEnabled && current_state != SELECTING_NODE){
    broadcastSQL();
    lastSQLBroadcast = millis();
  }

  if (current_state!=prevState) {
    // State change
    justSwitched = true;
    prevState = current_state;
   
  }
  
  switch(current_state){
    case NORMAL:
      //Normal operation

      if (justSwitched){
        radio.setDio1Action(rx);
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
        clearDisplay();
        temp = heltec_temperature();
        both.printf("Current Temp: %.1f °C\n", temp);
        both.printf("Packet Log:\n");
        justSwitched = false;
      }
      if ((button.isDoubleClick())){
        justSwitched = true;
        current_state = SELECTING_NODE;
      }

      //On packet recieved
      if(rxFlag){
        rxFlag = false;
        rxDataLength = radio.getPacketLength();
        
        both.printf("Reading data...\n");
        radio.readData(rxdata);
        
        if (_radiolib_status == RADIOLIB_ERR_NONE){
            if (!rxdata.isEmpty()){
              //both.printf("RCV->%s\n", rxdata.c_str());
              //both.printf("Decoding Packet...\n");
              Packet decodedPacket = decodePacket(rxdata);
              handlePacket(decodedPacket);
          }
          RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
        }
      }
      //Remove nodes that have not broadcasted 
      if (millis() - lastTimeoutCheck > 120000){ //Check every 2 minutes
        removeTimeoutNodes(900000); //Remove nodes that have not broadcasted in 15 minutes
        lastTimeoutCheck = millis();
      }

      break;
    case SELECTING_NODE:
      //Selecting node to send data to
      if (justSwitched){
        clearDisplay();
        temp = heltec_temperature();
        both.printf("Select Node:     TMP:%f\n", temp);
        for (const auto& node : nodeList){
          both.printf("%s\n", String(node.second.nodeId).c_str());
        }
        justSwitched = false;
        display.display();
      }
      
      if ((button.isSingleClick())){
        // Select node
        clearDisplay();
        both.printf("Select Node:     TMP:%f\n", temp);
        if (!nodeList.empty()){
          int i = 0;
          for (const auto& node : nodeList){
            if (i == currentNodeIndex){
              both.printf("->%s\n ", String(node.first).c_str());
            } else {
              both.printf("%s \n", String(node.first).c_str());
            }
            i++;
          }
          currentNodeIndex = (currentNodeIndex + 1) % nodeList.size();
        }else{
          both.printf("No nodes in network\n");
        }
        display.display();
      }

      ///TODO: Add maximum duty cycle limits?
      if ((button.isDoubleClick())){
        //Send data to selected node
        clearDisplay();
        if (!nodeList.empty()){
          auto it = nodeList.begin();
          std::advance(it, currentNodeIndex);
          both.printf("\nSending Data to: \n[%s]\n", String(it->first).c_str());
          radio.clearDio1Action();
          heltec_led(50); //50% Brightness
          tx_time = millis();
          //Create packet
          String packetStr = createPacketStr(chipId, it->first, temp, 0);

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
        }{
          both.printf("Returning to Log...\n");
        }
        delay(1000);
        justSwitched = true;
        current_state = NORMAL;
      }
      break;
    case SLEEP:
      //Sleeping
      break;
  }
  
  
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

  packet = String(String((uint64_t)senderId)+","+String((uint64_t)recvId)+","+String(temperature,1)+","+String(hopCount));
  
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
  String packetStr = createPacketStr(chipId, uint64_t(0), 0.0f, 0);

  //Transmit packet
  radio.clearDio1Action();
  RADIOLIB(radio.transmit(String(packetStr).c_str()));
  //both.printf("->Broadcasting To Network...\n");
  if (_radiolib_status == RADIOLIB_ERR_NONE)
  {
    last_tx = millis();
    lastBroadcast=millis();
    radio.setDio1Action(rx);
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    //both.printf("->Broadcast OK\n");
    return 0;
  }else{
    both.printf("->Broadcast Fail :(\n");
    return -1;
  
  }
}

static const char insertQuery[] PROGMEM = R"string_literal(
INSERT INTO `%s`
  (`MAC address`, `CPU0 reset reason`, `CPU1 reset reason`)
  VALUES ('%s', '%s', '%s');
)string_literal";

// Variadic function that will execute the query selected with passed parameters
bool queryExecute(DataQuery_t& data, const char* queryStr, ...) {
  if (sql.connected()) {
    char buf[MAX_QUERY_LEN];
    va_list args;
    va_start (args, queryStr);
    vsnprintf (buf, sizeof(buf), queryStr, args);
    va_end (args);

    Serial.printf("Executing SQL query: %s\n", buf);
    // Execute the query
    return sql.query(data, buf);
  }
  return false;
}

DataQuery_t broadcastSQL(){
  both.printf("Broadcasting to Database...\n");
  temp = heltec_temperature();
  DataQuery_t data;
  if (queryExecute(data,"INSERT INTO nodeinfo (nodeID, temp) VALUES (%llu, %f);", chipId, temp)){
    display.print("Data Inserted to SQL.\n");
    //sql.printResult(data, display);
    return data;
  }else{
    display.print("Data Insert Failed");
    return data;
  }
  
}

  // Create a cursor for the connection
  //MySQL_Cursor *cur_mem = new MySQL_Cursor(&conn);

  // Insert data into the table
  //char query[128];
  //sprintf(query, "INSERT INTO nodeInfo (nodeID, temperature) VALUES (%llu, %f);", chipId, temp);
  //bool exec_status = cur_mem->execute(query);

  // Delete the cursor to free up memory
  //delete cur_mem;

  //if (//exec_status){
    //both.printf("Temperature uploaded to database\n");
  //}else{
    //both.printf("Failed to upload temperature to database\n");
  //}


int handlePacket(Packet packet){
  //Handle recieved packet
  //Returns 0 if successful, -1 if failed
  if (packet.senderId == chipId && packet.recieverId != 0){
    //ignore
    //both.printf("Recieved own packet that wasnt broadcast\n");
    return 0;
  }else if (packet.senderId == chipId && packet.recieverId == 0){
    //Broadcast packet
    //both.printf("Recieved own broadcast packet\n");
    return 0;
  }
  if(packet.recieverId == 0 && packet.senderId != chipId){
    updateNodeList(packet.senderId, radio.getRSSI());
    return 0;
  }
  if(packet.recieverId == chipId){
    both.printf("RX From [%s]\n", String(packet.senderId).c_str());
    //both.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
    //both.printf("  SNR: %.2f dB\n", radio.getSNR());

    both.printf("Temperature recieved: %.1f \n", packet.data);
    return 0;
  }
  if(packet.recieverId != chipId && packet.recieverId != 0){
    both.printf(String("\nRouting Packet...").c_str());
    
    //routePacket(packet);
    return 0;
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
  if (!nodeList.empty() && packet.recieverId != 0 && packet.recieverId != chipId && nodeList.find(packet.recieverId) != nodeList.end()){
    if (nodeList.find(packet.recieverId) != nodeList.end()){
      //Node exists in list
      if (packet.hopCount < MAX_HOPS){
        //Packet has not reached maximum hops
        packet.hopCount++;
        both.printf("Routing packet to [%s]\n", String(packet.recieverId).c_str());
        String packetStr = createPacketStr(packet.senderId, packet.recieverId, packet.data, packet.hopCount);
        RADIOLIB(radio.transmit(String(packetStr).c_str()));

      }else{
        both.printf("Packet has reached maximum hops, discarding\n");
      }
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

//Clears display and updates
void clearDisplay(){
  display.cls();
  display.clear();
  display.display();
}


