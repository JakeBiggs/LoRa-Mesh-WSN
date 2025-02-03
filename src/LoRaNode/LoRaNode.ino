/*
Basic LoRa Mesh Network - Jacob Biggs
Using unofficial Heltec LoRa Library found at: https://github.com/ropg/Heltec_ESP32_LoRa_v3
*/

// Turns the 'PRG' button into the power button, long press is off
#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>
#include <ArduinoJson.h>
#include <MySQL.h>
#include <map>
#include <WiFi.h>
#include "secrets.h"
#include "ECC.h"
#include "Encryption.h"

bool WifiEnabled = 0; // Disable wifi here, 1 for gateway node 0 for end node,
WiFiClient client;

MySQL sql(&client, dbHost, dbPort);
#define MAX_QUERY_LEN 128
bool SQL_connection_status;

// Pause between transmited packets in seconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE 300

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY 866.3 // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0 kHz.
#define BANDWIDTH 250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference.
#define SPREADING_FACTOR 9

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER 0

#define MAX_LOG_ENTRIES 5
#define FONT_HEIGHT 10 //Guessing at the moment

enum PacketType
{
  BROADCAST,
  ACK,
  SECURE,
  DIRECT_COMM
};

enum DataType
{
  FLOAT,
  UINT64,
  KEY,
  ENCRYPTED_FLOAT

};

struct Packet
{
  uint64_t senderId;
  uint64_t receiverId;
  union
  {
    float floatData;
    uint64_t uint64Data;
    char keyData[65]; // 64 hex characters + null terminator
    uint8_t encryptedData[AES_BLOCK_SIZE]; // 16 bytes paddedd for floats if needed
  } data;

  int hopCount;
  PacketType packetType;
  DataType dataType;
};

struct NodeInfo
{
  uint64_t nodeId;
  float rssi;
  unsigned long lastBroadcastTime;
  bool gatewayNode;
  bool secured;
  std::array<uint8_t, 32> publicKey;
};

enum State
{
  NORMAL,
  SELECTING_NODE,
  SLEEP
};

std::array<uint8_t, 32> privateKey;
std::array<uint8_t, 32> publicKey;

// Define routing table
std::map<uint64_t, NodeInfo> nodeList;
const int MAX_HOPS = 1;
unsigned long lastTimeoutCheck = 0;
unsigned long BROADCAST_TIME = 60000 + random(-5000, 5000); // 1 minute    300000; //5 minutes
unsigned long lastBroadcast = 0;
unsigned long lastSQLBroadcast = 0;
bool firstLoop;

uint64_t chipId = 0;
String rxdata; // Recieved data buffer
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

String logBuffer[MAX_LOG_ENTRIES];
int logStartIndex = 0;
int logEndIndex = 0;
int logEntryCount = 0;
int logScrollOffset = 0;

void addLogEntry(const String& entry) {
    logBuffer[logEndIndex] = entry;
    logEndIndex = (logEndIndex + 1) % MAX_LOG_ENTRIES;
    if (logEntryCount < MAX_LOG_ENTRIES) {
        logEntryCount++;
    } else {
        logStartIndex = (logStartIndex + 1) % MAX_LOG_ENTRIES;
    }
}

void drawLog() {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    String header = "Packet Log:           " + String(heltec_temperature()) + "°C";
    display.drawString(0, 0, header);
    display.drawLine(0, FONT_HEIGHT+1, display.getWidth(), FONT_HEIGHT+1);

    int y = FONT_HEIGHT + 3; // Use the font height to determine the starting y position
    int index = (logStartIndex + logScrollOffset) % MAX_LOG_ENTRIES;
    for (int i = 0; i < logEntryCount && y < display.getHeight(); i++) {
        display.drawString(0, y, logBuffer[index]);
        y += FONT_HEIGHT; // Use the font height to determine the spacing
        index = (index + 1) % MAX_LOG_ENTRIES;
    }

    display.display();
}

void drawProgressBar(int current, int total, String label)
{
  int progress = (current * 100) / total;
  // draw the progress bar
  display.cls();
  display.drawProgressBar(0, 32, 120, 10, progress);

  // draw the percentage as String
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  // display.drawString(64, 15, String(progress) + "%");
  display.drawString(64, 15, label);
  display.display();
}

void setup()
{
  // For progress bar
  int totalSteps = 8;
  if (WifiEnabled)
  {
    totalSteps += 3;
  }
  int currentStep = 0;
  heltec_setup();
  Serial.println();
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Generate ECC keys
  auto keys = generate_keys();

  both.printf("\n");
  delay(1000);

  // Initialise Radio
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "Radio Initialising...");
  // both.println("Radio Initialising...");
  RADIOLIB_OR_HALT(radio.begin());
  // Set the callback function for recieved packets
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "Setting Callback Function...");
  radio.setDio1Action(rx);

  // Set radio parameters
  // both.printf("Frequency: %.2f MHz\n", FREQUENCY);
  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "Frequency: " + String(FREQUENCY) + " MHz");
  // both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "Bandwidth: " + String(BANDWIDTH) + " kHz");
  // both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "Spreading Factor: " + String(SPREADING_FACTOR));
  // both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
  currentStep++;
  delay(500);
  drawProgressBar(currentStep, totalSteps, "TX power: " + String(TRANSMIT_POWER) + " dBm");
  // display.cls();

  if (WifiEnabled)
  {
    // Connect to WiFi
    // both.print("Connecting to Wifi...\n");

    WiFi.begin(ssid, password);
    currentStep++;
    delay(500);
    drawProgressBar(currentStep, totalSteps, "Connecting to Wifi...");
    while (WiFi.status() != WL_CONNECTED && millis())
    {
      delay(100);
      // both.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      currentStep++;
      delay(500);
      drawProgressBar(currentStep, totalSteps, "Connecting to MySQL...");
      if (sql.connect(user, pass, database))
      { // conn.connect(server_addr, 3306, user, pass)) {
        // delay(1000);
        currentStep++;
        drawProgressBar(currentStep, totalSteps, "Connected to MySQL...");
        SQL_connection_status = true;
      }
      else
      {
        currentStep++;
        drawProgressBar(currentStep, totalSteps, "Failed to connect to MySQL...");
        SQL_connection_status = false;
        delay(2000);
      }
    }
  }

  // Get chip ID
  currentStep++;
  drawProgressBar(currentStep, totalSteps, "Getting Chip ID...");
  chipId = getChipId();
  delay(500);
  // Initialise Temperature
  temp = heltec_temperature();

  // Set up timed stuff
  unsigned long lastKeyGen = millis();
  unsigned long lastTimeoutCheck = millis();
  unsigned long lastBroadcast = millis();

  // Start receiving
  currentStep++;
  drawProgressBar(currentStep, totalSteps, "Starting Receive...");
  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
  delay(1000);
  clearDisplay();

  both.printf("Packet Log:\n");
  display.display();
  firstLoop = true;
}

void loop()
{
  // updates the state of the power button and implements long-press power off if used.
  // Functions same as heltec_loop()
  button.update();
#ifdef HELTEC_POWER_BUTTON

  // Power off button checking

  if (button.pressedFor(50000))
  {
#ifndef HELTEC_NO_DISPLAY_INSTANCE
    // Visually confirm it's off so user releases button
    display.displayOff();
#endif
    // Deep sleep (has wait for release so we don't wake up immediately)
    heltec_deep_sleep();
  }
#endif

  if (firstLoop)
  {
    broadcastNode();
    if (WifiEnabled && SQL_connection_status)
    {
      insertToSQL(chipId, temp);
    }
    firstLoop = false;
  }

  // Broadcast node every 1 minute
  if (millis() - lastBroadcast > BROADCAST_TIME && current_state != SELECTING_NODE)
  { // 1min
    broadcastNode();
    lastBroadcast = millis();
  }
  // Upload temperature to database every 5 minutes
  if (millis() - lastSQLBroadcast > 300000 && WifiEnabled && current_state != SELECTING_NODE)
  {
    temp = heltec_temperature();
    insertToSQL(chipId, temp);
    lastSQLBroadcast = millis();
  }

  if (current_state != prevState)
  {
    // State change
    justSwitched = true;
    prevState = current_state;
  }

  switch (current_state)
  {
  case NORMAL:
    // Normal operation

    if (justSwitched)
    {
      radio.setDio1Action(rx);
      RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      clearDisplay();
      temp = heltec_temperature();
      justSwitched = false;
    }
    if ((button.isDoubleClick()))
    {
      justSwitched = true;
      current_state = SELECTING_NODE;
    }

    // On packet recieved
    if (rxFlag)
    {
      rxFlag = false;
      rxDataLength = radio.getPacketLength();

      addLogEntry("Reading data...");
      radio.readData(rxdata);

      if (_radiolib_status == RADIOLIB_ERR_NONE)
      {
        if (!rxdata.isEmpty())
        {
          // both.printf("RCV->%s\n", rxdata.c_str());
          // both.printf("Decoding Packet...\n");
          Packet decodedPacket = decodePacket(rxdata);
          handlePacket(decodedPacket);
        }
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      }
    }
    // Remove nodes that have not broadcasted
    if (millis() - lastTimeoutCheck > 120000)
    {                             // Check every 2 minutes
      removeTimeoutNodes(900000); // Remove nodes that have not broadcasted in 15 minutes
      lastTimeoutCheck = millis();
    }
    drawLog();
    break;
  case SELECTING_NODE:
    // Selecting node to send data to
    if (justSwitched)
    {
      clearDisplay();
      temp = heltec_temperature();
      both.printf("Select Node:     TMP:%f\n", temp);
      for (const auto &node : nodeList)
      {
        both.printf("%s\n", String(node.second.nodeId).c_str());
      }
      justSwitched = false;
      display.display();
    }

    if ((button.isSingleClick()))
    {
      // Select node
      clearDisplay();
      both.printf("Select Node:     TMP:%f\n", temp);
      if (!nodeList.empty())
      {
        int i = 0;
        for (const auto &node : nodeList)
        {
          if (i == currentNodeIndex)
          {
            both.printf("->%s\n ", String(node.first).c_str());
          }
          else
          {
            both.printf("%s \n", String(node.first).c_str());
          }
          i++;
        }
        currentNodeIndex = (currentNodeIndex + 1) % nodeList.size();
      }
      else
      {
        both.printf("No nodes in network\n");
      }
      display.display();
    }

    /// TODO: Add maximum duty cycle limits?
    if ((button.isDoubleClick())) {
    // Send data to selected node
      clearDisplay();
      if (!nodeList.empty()) {
        auto it = nodeList.begin();
        std::advance(it, currentNodeIndex);
        both.printf("\nSending Data to: \n[%s]\n", String(it->first).c_str());
        radio.clearDio1Action();
        heltec_led(50); // 50% Brightness
        tx_time = millis();

        // Create packet
        String packet;
        if (it->second.secured) {
          // Encrypt the data
          std::array<uint8_t, 32> sharedSecret = compute_shared(privateKey, it->second.publicKey);
          uint8_t data[AES_BLOCK_SIZE] = {0};
          floatToBytes(temp, data); // Convert float to byte array

          // Debug: Print the byte array
          both.printf("Data to Encrypt: ");
          for (int i = 0; i < AES_BLOCK_SIZE; i++) {
              both.printf("%02X ", data[i]);
          }
          both.printf("\n");

          uint8_t encryptedData[AES_BLOCK_SIZE];
          encryptData(sharedSecret.data(), data, encryptedData);

          // Debug: Print the encrypted data
          both.printf("Encrypted Data: ");
          for (int i = 0; i < AES_BLOCK_SIZE; i++) {
              both.printf("%02X ", encryptedData[i]);
          }
          both.printf("\n");

          packet = createPacket(chipId, it->first, encryptedData, AES_BLOCK_SIZE, 0, PacketType::SECURE, DataType::ENCRYPTED_FLOAT);
        } else {
            packet = createPacket(chipId, it->first, temp, 0, PacketType::DIRECT_COMM, DataType::FLOAT);
        }
  
          RADIOLIB(radio.transmit(packet.c_str()));
          tx_time = millis() - tx_time;
          heltec_led(0); // Turn off LED
          if (_radiolib_status == RADIOLIB_ERR_NONE) {
              both.printf("->TX OK (%i ms)\n", tx_time);
          } else {
              both.printf("\n->TX fail (%i)\n", _radiolib_status);
          }
  
          last_tx = millis();
          radio.setDio1Action(rx);
  
          RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      }
      both.printf("Returning to Log...\n");
      delay(1000);
      justSwitched = true;
      current_state = NORMAL;
    }
    break;
  case SLEEP:
    // Sleeping
    break;
  }
}

void rx()
{
  rxDataLength = radio.getPacketLength();
  rxFlag = true;
}

String createPacket(uint64_t senderId, uint64_t recvId, float data, int hopCount, PacketType type, DataType dataType)
{
    StaticJsonDocument<256> doc;
    doc["sender"] = senderId;
    doc["recipient"] = recvId;
    doc["hop_count"] = hopCount;
    doc["packet_type"] = static_cast<int>(type);
    doc["data_type"] = static_cast<int>(dataType);

    if (dataType == DataType::FLOAT)
    {
        doc["data"] = data;
    }
    else if (dataType == DataType::UINT64)
    {
        doc["data"] = String(data, HEX);
    }

    String packetStr;
    serializeJson(doc, packetStr);
    return packetStr;
}

String createPacket(uint64_t senderId, uint64_t recvId, const std::array<uint8_t, AES_BLOCK_SIZE>& data, int hopCount, PacketType type, DataType dataType)
{
    StaticJsonDocument<256> doc;
    doc["sender"] = senderId;
    doc["recipient"] = recvId;
    doc["hop_count"] = hopCount;
    doc["packet_type"] = static_cast<int>(type);
    doc["data_type"] = static_cast<int>(dataType);

    String dataHex;
    for (const auto& byte : data) {
        char hex[3];
        sprintf(hex, "%02X", byte);
        dataHex += hex;
    }
    doc["data"] = dataHex;

    String packetStr;
    serializeJson(doc, packetStr);
    return packetStr;
}

String createPacket(uint64_t senderId, uint64_t recvId, const String& data, int hopCount, PacketType type, DataType dataType)
{
    StaticJsonDocument<256> doc;
    doc["sender"] = senderId;
    doc["recipient"] = recvId;
    doc["hop_count"] = hopCount;
    doc["packet_type"] = static_cast<int>(type);
    doc["data_type"] = static_cast<int>(dataType);

    doc["data"] = data;

    String packetStr;
    serializeJson(doc, packetStr);
    return packetStr;
}

String createPacket(uint64_t senderId, uint64_t recvId, const uint8_t* data, size_t dataSize, int hopCount, PacketType type, DataType dataType)
{
    StaticJsonDocument<256> doc;
    doc["sender"] = senderId;
    doc["recipient"] = recvId;
    doc["hop_count"] = hopCount;
    doc["packet_type"] = static_cast<int>(type);
    doc["data_type"] = static_cast<int>(dataType);

    String dataHex;
    for (size_t i = 0; i < dataSize; i++) {
        char hex[3];
        sprintf(hex, "%02X", data[i]);
        dataHex += hex;
    }
    doc["data"] = dataHex;

    String packetStr;
    serializeJson(doc, packetStr);
    return packetStr;
}

Packet decodePacket(const String& packetStr)
{
    Packet packet;
    StaticJsonDocument<256> doc;
    deserializeJson(doc, packetStr);

    packet.senderId = doc["sender"];
    packet.receiverId = doc["recipient"];
    packet.hopCount = doc["hop_count"];
    packet.packetType = static_cast<PacketType>(doc["packet_type"]);
    packet.dataType = static_cast<DataType>(doc["data_type"]);

    if (packet.dataType == DataType::FLOAT) {
        packet.data.floatData = doc["data"];
    } else if (packet.dataType == DataType::UINT64) {
        packet.data.uint64Data = doc["data"];
    } else if (packet.dataType == DataType::KEY) {
        for (int i = 0; i < 32; ++i) {
            packet.data.keyData[i] = strtol(doc["data"].as<String>().substring(2 * i, 2 * i + 2).c_str(), nullptr, 16);
        }
    } else if (packet.dataType == DataType::ENCRYPTED_FLOAT) {
        for (int i = 0; i < AES_BLOCK_SIZE; ++i) {
            packet.data.encryptedData[i] = strtol(doc["data"].as<String>().substring(2 * i, 2 * i + 2).c_str(), nullptr, 16);
        }
    }

    return packet;
}

int broadcastNode()
{
    // Broadcasts a packet to all nodes
    // Returns 0 if successful, -1 if failed

    // Create packet
    // Receive ID of 0 its a broadcast
    // PacketType  = BROADCAST;
    //float data = 0.0f;
    if (WifiEnabled && SQL_connection_status)
    {
        float data = 1.0f; // Node is a gateway node
    }

    String publicKeyHex;
    for (const auto& byte : publicKey) {
        char hex[3];
        sprintf(hex, "%02X", byte);
        publicKeyHex += hex;
    }

    String packetStr = createPacket(chipId, uint64_t(0), publicKeyHex, 0, PacketType::BROADCAST, DataType::KEY);
    RADIOLIB(radio.transmit(packetStr.c_str()));
    // both.printf("->Broadcasting To Network...\n");
    if (_radiolib_status == RADIOLIB_ERR_NONE)
    {
        last_tx = millis();
        lastBroadcast = millis();
        radio.setDio1Action(rx);
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
        // both.printf("->Broadcast OK\n");
        return 0;
    }
    else
    {
        both.printf("->Broadcast Fail :(\n");
        return -1;
    }
}

// Variadic function that will execute the query selected with passed parameters
bool queryExecute(DataQuery_t &data, const char *queryStr, ...)
{
  if (sql.connected())
  {
    char buf[MAX_QUERY_LEN];
    va_list args;
    va_start(args, queryStr);
    vsnprintf(buf, sizeof(buf), queryStr, args);
    va_end(args);

    Serial.printf("Executing SQL query: %s\n", buf);
    // Execute the query
    return sql.query(data, buf);
  }
  return false;
}

DataQuery_t insertToSQL(uint64_t NodeId, float temp)
{
  both.printf("Uploading to Database...\n");
  DataQuery_t data;
  if (queryExecute(data, "INSERT INTO nodeinfo (nodeID, temp) VALUES (%llu, %f);", NodeId, temp))
  {
    display.print("Data Inserted to SQL.\n");
    // sql.printResult(data, display);
    return data;
  }
  else
  {
    display.print("Data Insert Failed");
    return data;
  }
}

void updateNodeList(uint64_t nodeId, float rssi, bool isGatewayNode = false, bool secured = false, std::array<uint8_t, 32> publicKey = {})
{
  // Updates the list of nodes in the network
  NodeInfo node;
  node.nodeId = nodeId;
  node.rssi = rssi;
  node.lastBroadcastTime = millis();
  node.gatewayNode = isGatewayNode;
  node.secured = secured;
  node.publicKey = publicKey;
  // Add node to list or overwrite existing node info
  nodeList[nodeId] = node;
}

NodeInfo *findNode(uint64_t nodeId)
{
  // Finds a node in the list
  auto it = nodeList.find(nodeId);
  if (it != nodeList.end())
  {
    return &(it->second); // Return pointer to node info
  }
  else
  {
    return nullptr;
  }
}

int handlePacket(Packet packet) {
    // Handle received packet
    // Returns 0 if successful, -1 if failed
    if (packet.senderId == chipId && packet.packetType != PacketType::BROADCAST) {
        return 0;
    } else if (packet.senderId == chipId && packet.packetType == PacketType::BROADCAST) {
        return 0;
    }
    if (packet.packetType == PacketType::BROADCAST && packet.senderId != chipId) {
        NodeInfo* senderNode = findNode(packet.senderId);
        if (senderNode && senderNode->secured) {
            // Key exchange already completed, no need to initiate again
            return 0;
        }

        KeyExchangeRequest(packet.senderId);
        std::array<uint8_t, 32> otherPublicKey;
        for (int i = 0; i < 32; ++i) {
            otherPublicKey[i] = strtol(packet.data.keyData + 2 * i, nullptr, 16);
        }
        auto shared_secret = compute_shared(privateKey, otherPublicKey);
        String sharedSecretStr = "Shared Secret: ";
        for (const auto& byte : shared_secret) {
            char hex[3];
            sprintf(hex, "%02X", byte);
            sharedSecretStr += hex;
        }
        addLogEntry(sharedSecretStr);
        updateNodeList(packet.senderId, radio.getRSSI(), false, true, otherPublicKey);
        return 0;
    }
    if (packet.packetType == PacketType::ACK) {
      addLogEntry("Received Acknowledgment from [" + String(packet.senderId) + "]");
      std::array<uint8_t, 32> otherPublicKey;
      for (int i = 0; i < 32; ++i) {
          otherPublicKey[i] = strtol(packet.data.keyData + 2 * i, nullptr, 16);
      }
      auto shared_secret = compute_shared(privateKey, otherPublicKey);
      String sharedSecretStr = "Shared Secret: ";
      for (const auto& byte : shared_secret) {
          char hex[3];
          sprintf(hex, "%02X", byte);
          sharedSecretStr += hex;
      }
      addLogEntry(sharedSecretStr);
      updateNodeList(packet.senderId, radio.getRSSI(), false, true, otherPublicKey);
      return 0;
    }
    if (packet.packetType == PacketType::SECURE) {
        addLogEntry("Received Secure Packet from [" + String(packet.senderId) + "]");
    
        // Decrypt the data
        std::array<uint8_t, 32> sharedSecret = compute_shared(privateKey, findNode(packet.senderId)->publicKey);
        uint8_t encryptedData[AES_BLOCK_SIZE];
        for (int i = 0; i < AES_BLOCK_SIZE; i++) {
            encryptedData[i] = packet.data.encryptedData[i];
        }
    
        // Debug: Print the encrypted data
        both.printf("Encrypted Data: ");
        for (int i = 0; i < AES_BLOCK_SIZE; i++) {
            both.printf("%02X ", encryptedData[i]);
        }
        both.printf("\n");
    
        uint8_t decryptedData[AES_BLOCK_SIZE];
        decryptData(sharedSecret.data(), encryptedData, decryptedData);
    
        // Debug: Print the decrypted byte array
        both.printf("Decrypted Data: ");
        for (int i = 0; i < AES_BLOCK_SIZE; i++) {
            both.printf("%02X ", decryptedData[i]);
        }
        both.printf("\n");
    
        float decryptedFloat = bytesToFloat(decryptedData);
    
        addLogEntry("Decrypted Data: " + String(decryptedFloat) + "°C");
        both.printf("Decrypted Data: %f", decryptedFloat);
        return 0;
    }
    if (packet.receiverId == chipId && packet.packetType == PacketType::DIRECT_COMM) {
        addLogEntry("RX From [" + String(packet.senderId) + "]");
        addLogEntry("Temp received: " + String(packet.data.floatData) + "°C");
        NodeInfo* senderNode = findNode(packet.senderId);

        if (!senderNode) {
            updateNodeList(packet.senderId, radio.getRSSI(), false);
            senderNode = findNode(packet.senderId);
        }

        if (WifiEnabled && SQL_connection_status && !senderNode->gatewayNode) {
            addLogEntry("Inserting To Database...");
            insertToSQL(packet.senderId, packet.data.floatData);
        }

        return 0;
    }
    if (packet.receiverId != chipId && packet.receiverId != 0) {
        addLogEntry("Routing Packet...");

        routePacket(packet);
        return 0;
    }
    return 0;
}

void removeTimeoutNodes(unsigned long timeout)
{
  // Removes nodes from the list that have not broadcasted in a while
  for (auto it = nodeList.begin(); it != nodeList.end();)
  {
    if (millis() - it->second.lastBroadcastTime > timeout)
    {
      it = nodeList.erase(it); // Erase node if timeout and increment iterator
    }
    else
    {
      it++; // Increment iterator if not erased
    }
  }
}

void routePacket(Packet packet) {
    // Routes a packet to the correct node
    if (!nodeList.empty() && packet.packetType != PacketType::BROADCAST && packet.receiverId != chipId && nodeList.find(packet.receiverId) != nodeList.end()) {
        // Node exists in list and needs to be routed
        if (packet.hopCount < MAX_HOPS) {
            // Packet has not reached maximum hops
            packet.hopCount++;
            both.printf("Routing packet to [%s]\n", String(packet.receiverId).c_str());
            String packetStr;
            if (packet.dataType == DataType::FLOAT) {
                packetStr = createPacket(packet.senderId, packet.receiverId, packet.data.floatData, packet.hopCount, PacketType::DIRECT_COMM, DataType::FLOAT);
            } else if (packet.dataType == DataType::UINT64) {
                packetStr = createPacket(packet.senderId, packet.receiverId, packet.data.uint64Data, packet.hopCount, PacketType::DIRECT_COMM, DataType::UINT64);
            }
            RADIOLIB(radio.transmit(packetStr.c_str()));
            if (_radiolib_status == RADIOLIB_ERR_NONE)
              {
                  radio.setDio1Action(rx);
                  RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
              }
            radio.setDio1Action(rx);
            RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
        } else {
            both.printf("Packet has reached maximum hops, discarding\n");
        }
    }
}

void KeyExchangeRequest(uint64_t NodeId)
{
    String publicKeyHex;
    for (const auto& byte : publicKey) {
        char hex[3];
        sprintf(hex, "%02X", byte);
        publicKeyHex += hex;
    }

    String packet = createPacket(chipId, NodeId, publicKeyHex, 0, PacketType::ACK, DataType::UINT64);
    RADIOLIB(radio.transmit(packet.c_str()));
    if (_radiolib_status == RADIOLIB_ERR_NONE)
      {
          radio.setDio1Action(rx);
          RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
      }
}

String getChipIdStr()
{
  uint64_t chipId = ESP.getEfuseMac();   // Unique chip id (essentially mac address)
  char chipIdStr[17];                    // Buffer to hold the chip ID string and null terminator
  sprintf(chipIdStr, "%016llX", chipId); // Convert the chip ID to a string
                                         //% format specifier
                                         // 016: minimum field width
                                         // ll: same as long long int (uint64_t)
                                         // x: formatted as unsigned hex int
  return String(chipIdStr);
}

uint64_t getChipId()
{
  uint64_t chipId = ESP.getEfuseMac();
  return chipId;
}

// Clears display and updates
void clearDisplay()
{
  display.cls();
  display.clear();
  display.display();
}