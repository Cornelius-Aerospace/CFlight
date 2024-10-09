#include "comms.h"
#define PAYLOAD_MAX_SIZE 128  // the maximum size of the payload
#define PACKET_HEADER_SIZE 16 // the size of the packet header
#define PACKET_MAX_SIZE PACKET_HEADER_SIZE + PAYLOAD_MAX_SIZE
const uint8_t networkId = 0; // the network ID we are using
uint8_t connectionStage = 0;

char personalMacAddress[18];

unsigned long lastPingRec = 0;
unsigned long pingId = 0;
unsigned long receivedPingId = 0;
unsigned long pingLatency = 0;

uint8_t ledPattern = 0; // 0 = off, 1 = on, 2 = blink, 3 = fast blink

uint8_t baseStationMacAddress[] = {0xC0, 0x49, 0xEF, 0x65, 0x52, 0x00}; // MAC address of the base station (C0:49:EF:65:52:00)

uint8_t packetBuffer[PACKET_MAX_SIZE];
uint8_t packetBufferTx[PACKET_MAX_SIZE];
uint8_t packetTxLength = 0;

unsigned long packetTime;
PacketType packetType;
unsigned long packetSalt;
uint8_t payloadLength;
uint8_t packetChecksum;
char *packetPayload;
bool packetValid;
bool freshPacket;

unsigned long txSalt = 0;
// Command packet decoded payload holders
unsigned long commandSalt = 0;
uint8_t commandInt = 0;
Command command = Command::NONE;
uint8_t commandArgLength = 0;
uint8_t commandArgCount = 0;
byte commandArgBuffer[MAX_ARGS * 4];

std::function<bool(uint8_t *responsePacketBuffer, uint8_t *responsePacketLength, uint8_t *args, uint8_t argsCount, uint8_t argsArrayLength, Command cmd, unsigned long time, unsigned long salt)> cmdPacketCallback;

void registerCmdPacketCallback(std::function<bool(uint8_t *responsePacketBuffer, uint8_t *responsePacketLength, uint8_t *args, uint8_t argsCount, uint8_t argsArrayLength, Command cmd, unsigned long time, unsigned long salt)> cb)
{
    cmdPacketCallback = cb;
}

void handshakeResponse(const uint8_t *macAddr)
{
    char buffer[128];
    uint8_t messageLength = sprintf(buffer, "h%i", 1);
    if (sendMessage(buffer, messageLength, macAddr))
    {
        connectionStage = 1;
        setBaseMacAddress(macAddr);
    }
    else
    {
        connectionStage = 0;
    }
}

void setBaseMacAddress(const uint8_t *macAddr)
{
    memcpy(baseStationMacAddress, macAddr, 6);
}

bool sendMessage(const char *msg, uint8_t messageLength, const uint8_t *macAddr)
{
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, macAddr, 6);
    if (!esp_now_is_peer_exist(macAddr))
    {
        esp_now_add_peer(&peerInfo);
    }
    esp_err_t result = esp_now_send(macAddr, (const uint8_t *)msg, messageLength);

    
    if (result == ESP_OK)
        return true;
    else
    {

        if (result == ESP_ERR_ESPNOW_NOT_INIT)
        {
            Serial.println("!NOT_INIT");
        }
        else if (result == ESP_ERR_ESPNOW_ARG)
        {
            Serial.println("!INVALID_ARG");
        }
        else if (result == ESP_ERR_ESPNOW_INTERNAL)
        {
            Serial.println("!INTERNAL_ERR");
        }
        else if (result == ESP_ERR_ESPNOW_NO_MEM)
        {
            Serial.println("!ESP_ERR_ESPNOW_NO_MEM");
        }
        else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
        {
            Serial.println("!NOT_FOUND");
        }
        else
        {
            Serial.println("!UNKNOWN_SEND_ERROR");
        }

        return false;
    }
}

void broadcast(const String &message)
{
    // this will broadcast a message to everyone in range
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
    if (!esp_now_is_peer_exist(broadcastAddress))
    {
        esp_now_add_peer(&peerInfo);
    }
    esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)message.c_str(), message.length());
    // Check the result
    if (result == ESP_OK)
    {
        return;
        Serial.println("Broadcast message success");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_INIT)
    {
        Serial.println("ESPNOW not Init.");
    }
    else if (result == ESP_ERR_ESPNOW_ARG)
    {
        Serial.println("Invalid Argument");
    }
    else if (result == ESP_ERR_ESPNOW_INTERNAL)
    {
        Serial.println("Internal Error");
    }
    else if (result == ESP_ERR_ESPNOW_NO_MEM)
    {
        Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
    {
        Serial.println("Peer not found.");
    }
    else
    {
        Serial.println("Unknown error");
    }
}

void formatMacAddress(const uint8_t *macAddr, char *buffer, uint8_t maxLength)
{
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

uint8_t closePayload(uint8_t *packetBuffer, uint8_t payloadLength)
{
    packetBuffer[PACKET_HEADER_SIZE + payloadLength] = 0xFF;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 1] = 0xFE;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 2] = 0xFB;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 3] = 0xFF;
    uint8_t checksum = calculateChecksum((char *)packetBuffer + PACKET_HEADER_SIZE, payloadLength);
    memcpy(packetBuffer + 8, &checksum, 1);
    return PACKET_HEADER_SIZE + payloadLength + 4;
}

void sendPong(unsigned long pingId, const uint8_t *macAddr)
{
    lastPingRec = millis();
    formPacketHeader(packetBufferTx, lastPingRec, PacketType::PONG, 4, 0, txSalt);
    packetBufferTx[PACKET_HEADER_SIZE] = pingId >> 24;
    packetBufferTx[PACKET_HEADER_SIZE + 1] = pingId >> 16;
    packetBufferTx[PACKET_HEADER_SIZE + 2] = pingId >> 8;
    packetBufferTx[PACKET_HEADER_SIZE + 3] = pingId;
    sendPacket(packetBufferTx, closePayload(packetBufferTx, 4), macAddr);
}

uint8_t formCmdAckPacket(uint8_t *txBuffer, unsigned long rxSalt, Command rxCmd, bool ack, uint8_t reason)
{
    formPacketHeader(txBuffer, millis(), PacketType::ACK, 6, 0, txSalt);
    memcpy(txBuffer + 12, &rxSalt, 4); // Copy arm packet salt (for reference) to first payload entry
    txBuffer[15] = (uint8_t)rxCmd;         // Then the command we are acking
    txBuffer[16] = (uint8_t)ack;           // Is this ack or nack
    txBuffer[17] = reason;
    return closePayload(txBuffer, 7);
}

uint8_t calculateChecksum(char *payload, uint8_t payloadLength)
{
    uint8_t checksum = 0;
    for (uint8_t i = PACKET_HEADER_SIZE; i < payloadLength; i++)
    {
        checksum += payload[i];
    }
    return checksum;
}

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
    // only allow a maximum of 250 characters in the message + a null terminating byte
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    uint8_t msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
    strncpy(buffer, (const char *)data, msgLen);
    // make sure we are null terminated
    buffer[msgLen] = 0;
#ifdef DEBUG
    Serial.printf("[NET] Recieved: %s", buffer);
#endif
    // Parse the packet into a header and the payload
    packetTime = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6]; // Time (unsigned long)
    packetType = (PacketType)buffer[7];                                          // Packet type (enum)
    payloadLength = buffer[9];                                                   // Payload length (uint8_t)
    packetChecksum = buffer[8];                                                  // Checksum (uint8_t)
    packetPayload = buffer + 16;
    packetSalt = buffer[10] << 24 | buffer[11] << 16 | buffer[12] << 8 | buffer[13];
    // Payload (char array)
    // Check if the packet is valid
    if (dataLen != payloadLength + PACKET_HEADER_SIZE)
    {
        Serial.println("!INVALID_PACKET");
        return;
    }
    if (packetChecksum != calculateChecksum(packetPayload + PACKET_HEADER_SIZE, payloadLength))
    {
        Serial.println("!INVALID_CHECKSUM");
        return;
    }
    freshPacket = true;
    packetValid = true;
    if (packetType == PacketType::COMMAND)
    {
        parseCommandPayload((uint8_t *)packetPayload, payloadLength, packetTime, packetSalt);
    } else if (packetType == PacketType::PING) {
        unsigned long pingId = packetPayload[0] << 24 | packetPayload[1] << 16 | packetPayload[2] << 8 | packetPayload[3];
        sendPong(pingId, macAddr);
    }
}

// callback when data is sent
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status)
{
#ifdef DEBUG
    char macStr[18];
    formatMacAddress(macAddr, macStr, 18);
    Serial.print("Last Packet Sent to: ");
    Serial.println(macStr);
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

void testComms() {
    unsigned long len = formTelemetryPacket(packetBufferTx, 1000, 10.9, -0.12, 51.0, -90.1, 9.2, State::DESCENT);
    sendPacket(packetBufferTx, len, baseStationMacAddress);
}

void formPacketHeader(uint8_t *packetBuffer, unsigned long time, uint8_t packetType, uint8_t payloadLength, uint8_t checksum, unsigned long packetSalt)
{
    // Packet format:
    // 12 byte header
    // 0-2: 0xFF, 0xFF, 0xFF (sync bytes)
    // 3-6: Time (unsigned long)
    // 7: Packet type (uint8_t)
    // 8: Checksum (uint8_t)
    // 9: Payload length (uint8_t)
    // 10-13: Packet salt (unsigned long)
    // 14-15: 0xFF, 0xFF (sync bytes)
    packetBuffer[0] = 0xFF;
    packetBuffer[1] = 0xFF;
    packetBuffer[2] = 0xFF;
    memcpy(packetBuffer + 3, &time, 4);
    memcpy(packetBuffer + 7, &packetType, 1);
    memcpy(packetBuffer + 8, &checksum, 1);
    memcpy(packetBuffer + 9, &payloadLength, 1);
    memcpy(packetBuffer + 10, &packetSalt, 4);
    packetBuffer[13] = 0xFF;
    packetBuffer[14] = 0xFF;
}

unsigned long formTelemetryPacket(uint8_t *packetBuffer, unsigned long time, float altitude, float dAltitude, float gpsLong, float gpsLat, float gpsAltitude, State currentState)
{

    // Payload format:
    // 0-3: Altitude (float)
    // 4-7: dAltitude (float)
    // 8-11: GPS Longitude (float)
    // 12-15: GPS Latitude (float)
    // 16-19: GPS Altitude (float)
    // 20-23: State (uint8_t)
    // 24-27: 0xFF, 0xFE, 0xFE, 0xFF (sync bytes)
    // Payload length: 28 bytes, total packet length: 40 bytes
    formPacketHeader(packetBuffer, time, PacketType::TELEMETRY, 28, 0, txSalt);
    memcpy(packetBuffer + PACKET_HEADER_SIZE, &altitude, 4); // 12 is the start of the payload
    memcpy(packetBuffer + 16, &dAltitude, 4);
    memcpy(packetBuffer + 20, &gpsLong, 4);
    memcpy(packetBuffer + 24, &gpsLat, 4);
    memcpy(packetBuffer + 28, &gpsAltitude, 4);
    memcpy(packetBuffer + 32, &currentState, 1);
    // Calculate checksum (only for payload)
    return closePayload(packetBuffer, 32-PACKET_HEADER_SIZE);
}

bool parseCommandPayload(uint8_t *payloadBuffer, uint8_t payloadLength, unsigned long rxTime, unsigned long rxSalt)
{
    // Command packet payload format:
    // 0-3: Command salt (unsigned long)
    // 4-7: Command (uint8_t)
    // 8: Command argument byte count (uint8_t)
    // 9: Command argument count (uint8_t)
    // 10-...: Command arguments (byte array)

    commandSalt = 0;
    commandInt = 0;
    command = Command::NONE;
    commandArgLength = 0;
    commandArgCount = 0;
    memcpy(&commandSalt, payloadBuffer, 4);
    memcpy(&commandInt, payloadBuffer + 4, 4);
    memcpy(&commandArgLength, payloadBuffer + 8, 1);
    memcpy(&commandArgCount, payloadBuffer + 9, 1);

    command = (Command)commandInt;
    if (commandArgCount > 0 && commandArgCount < MAX_ARGS)
    {

        for (uint8_t i = 0; i < commandArgLength; i++)
        {
            commandArgBuffer[i] = payloadBuffer[10 + i];
        }
    }
    else if (commandArgCount == 0)
    {
        // No arguments
    }
    else
    {
        Serial.println("!INVALID_COMMAND_ARG_COUNT");
        return false;
    }
    if (cmdPacketCallback != nullptr)
    {
        cmdPacketCallback((uint8_t *)packetBufferTx, (uint8_t *)packetTxLength, (byte *)commandArgBuffer,
                          commandArgCount, commandArgLength, command, rxTime, rxSalt);
    }
    return true;
}

bool sendPacket(uint8_t *packetBuffer, unsigned long packetLength, const uint8_t *macAddr)
{

    Serial.println("-");
    for (uint8_t i = 0; i < packetLength; i++)
    {
        Serial.write(packetBuffer[i]);
    }
    Serial.println("\n-");
    if (connectionStage == 1)
    {
        // Packet buffer is uint8_t array, we need to cast it to char array
        txSalt++;
        return sendMessage((char *)packetBuffer, packetLength, macAddr);
    }
    else
    {
        Serial.println("Error sending packet: not connected");
        return false;
    }
}

void initComms()
{
    // Set device in STA mode to begin with to get MAC address
    WiFi.mode(WIFI_STA);
    strcpy(personalMacAddress, WiFi.macAddress().c_str());
    WiFi.disconnect();
    Serial.printf("FC MAC: %s\n", personalMacAddress);

    // startup ESP Now
    if (esp_now_init() == ESP_OK)
    {
        Serial.println("ESPNow Init Success");
        esp_now_register_recv_cb(receiveCallback);
        esp_now_register_send_cb(sentCallback);
        // ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K));
    }
    else
    {
        Serial.println("ESPNow Init Failed");
        initErrorLoop(SystemSettings());
    }
}
