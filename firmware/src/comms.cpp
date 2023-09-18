#include "comms.h"
#define PAYLOAD_MAX_SIZE 128  // the maximum size of the payload
#define PACKET_HEADER_SIZE 12 // the size of the packet header
#define PACKET_MAX_SIZE PACKET_HEADER_SIZE + PAYLOAD_MAX_SIZE
const int networkId = 0; // the network ID we are using
int connectionStage = 0;

char personalMacAddress[18];

unsigned long lastPingRec = 0;
unsigned long pingId = 0;
unsigned long receivedPingId = 0;
unsigned long pingLatency = 0;

int ledPattern = 0; // 0 = off, 1 = on, 2 = blink, 3 = fast blink

uint8_t baseStationMacAddress[] = {0xC0, 0x49, 0xEF, 0x65, 0x52, 0x00}; // MAC address of the base station (C0:49:EF:65:52:00)

uint8_t packetBuffer[PACKET_MAX_SIZE];
uint8_t packetBufferTx[PACKET_MAX_SIZE];

unsigned long packetTime;
PacketType packetType;
int payloadLength;
int packetChecksum;
char *packetPayload;
bool packetValid;
bool freshPacket;

// Command packet decoded payload holders
unsigned long commandSalt = 0;
int commandInt = 0;
Command command = Command::NONE;
int commandArgLength = 0;
int commandArgCount = 0;
byte commandArgBuffer[MAX_ARGS*4];

void handshakeResponse(const uint8_t *macAddr)
{
    char buffer[128];
    int messageLength = sprintf(buffer, "h%i", 1);
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

bool sendMessage(const char *msg, int messageLength, const uint8_t *macAddr)
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
    // and this will send a message to a specific device
    /*uint8_t peerAddress[] = {0x3C, 0x71, 0xBF, 0x47, 0xA5, 0xC0};
    esp_now_peer_info_t peerInfo = {};
    memcpy(&peerInfo.peer_addr, peerAddress, 6);
    if (!esp_now_is_peer_exist(peerAddress))
    {
      esp_now_add_peer(&peerInfo);
    }
    esp_err_t result = esp_now_send(peerAddress, (const uint8_t *)message.c_str(), message.length());*/
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

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
    snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}

int closePayload(uint8_t *packetBuffer, int payloadLength)
{
    packetBuffer[PACKET_HEADER_SIZE + payloadLength] = 0xFF;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 1] = 0xFE;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 2] = 0xFE;
    packetBuffer[PACKET_HEADER_SIZE + payloadLength + 3] = 0xFF;
    return payloadLength + 4;
}

void sendPong(unsigned long pingId, const uint8_t *macAddr)
{
    lastPingRec = millis();
    formPacketHeader(packetBufferTx, lastPingRec, PacketType::PONG, 4, 0);
    packetBufferTx[PACKET_HEADER_SIZE] = pingId >> 24;
    packetBufferTx[PACKET_HEADER_SIZE + 1] = pingId >> 16;
    packetBufferTx[PACKET_HEADER_SIZE + 2] = pingId >> 8;
    packetBufferTx[PACKET_HEADER_SIZE + 3] = pingId;
    int checksum = calculateChecksum((char *)(packetBufferTx + 12), 28);
    memcpy(packetBufferTx + 8, &checksum, 1); // Update checksum in header

    sendPacket(packetBufferTx, closePayload(packetBufferTx, 4), macAddr);
}

int calculateChecksum(char *payload, int payloadLength)
{
    int checksum = 0;
    for (int i = 0; i < payloadLength; i++)
    {
        checksum += payload[i];
    }
    return checksum;
}

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen)
{
    // only allow a maximum of 250 characters in the message + a null terminating byte
    char buffer[ESP_NOW_MAX_DATA_LEN + 1];
    int msgLen = min(ESP_NOW_MAX_DATA_LEN, dataLen);
    strncpy(buffer, (const char *)data, msgLen);
    // make sure we are null terminated
    buffer[msgLen] = 0;
#ifdef DEBUG
    Serial.printf("[NET] Recieved: %s", buffer);
#endif
    // Parse the packet into a header and the payload
    {
        if (buffer[0] == 'c')
        {
            if (buffer[1] == 'd')
            {
                // Close down the connection
                connectionStage = 0;
                Serial.println("!CLOSED");
            }
            else if (buffer[1] == 'p')
            {
                // Ping received
                receivedPingId = atol(buffer + 3);
                sendPong(receivedPingId, macAddr);
            }
        }
        else if (buffer[0] == 'm')
        {
            // We have received a message
            Serial.printf("MSG:%s\n", buffer + 2);
        }
    }
    packetTime = buffer[3] << 24 | buffer[4] << 16 | buffer[5] << 8 | buffer[6]; // Time (unsigned long)
    packetType = (PacketType)buffer[7];                                          // Packet type (enum)
    payloadLength = buffer[9];                                                   // Payload length (int)
    packetChecksum = buffer[8];                                                  // Checksum (int)
    packetPayload = buffer + 12;                                                 // Payload (char array)
    // Check if the packet is valid
    if (dataLen != payloadLength + 12)
    {
        Serial.println("!INVALID_PACKET");
        return;
    }
    if (packetChecksum != calculateChecksum(packetPayload, payloadLength))
    {
        Serial.println("!INVALID_CHECKSUM");
        return;
    }
    freshPacket = true;
    packetValid = true;
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

void formPacketHeader(uint8_t *packetBuffer, unsigned long time, int packetType, int payloadLength, int checksum)
{
    // Packet format:
    // 12 byte header
    // 0-2: 0xFF, 0xFF, 0xFF (sync bytes)
    // 3-6: Time (unsigned long)
    // 7: Packet type (int)
    // 8: Checksum (int)
    // 9: Payload length (int)
    // 10-11: 0xFF, 0xFF (sync bytes)
    packetBuffer[0] = 0xFF;
    packetBuffer[1] = 0xFF;
    packetBuffer[2] = 0xFF;
    memcpy(packetBuffer + 3, &time, 4);
    memcpy(packetBuffer + 7, &packetType, 1);
    memcpy(packetBuffer + 8, &checksum, 1);
    memcpy(packetBuffer + 9, &payloadLength, 1);
    packetBuffer[10] = 0xFF;
    packetBuffer[11] = 0xFF;
}

unsigned long formTelemetryPacket(uint8_t *packetBuffer, unsigned long time, float altitude, float dAltitude, float gpsLong, float gpsLat, float gpsAltitude, State currentState)
{

    // Payload format:
    // 0-3: Altitude (float)
    // 4-7: dAltitude (float)
    // 8-11: GPS Longitude (float)
    // 12-15: GPS Latitude (float)
    // 16-19: GPS Altitude (float)
    // 20-23: State (int)
    // 24-27: 0xFF, 0xFE, 0xFE, 0xFF (sync bytes)
    // Payload length: 28 bytes, total packet length: 40 bytes
    formPacketHeader(packetBuffer, time, PacketType::TELEMETRY, 28, 0);
    memcpy(packetBuffer + PACKET_HEADER_SIZE, &altitude, 4); // 12 is the start of the payload
    memcpy(packetBuffer + 16, &dAltitude, 4);
    memcpy(packetBuffer + 20, &gpsLong, 4);
    memcpy(packetBuffer + 24, &gpsLat, 4);
    memcpy(packetBuffer + 28, &gpsAltitude, 4);
    memcpy(packetBuffer + 32, &currentState, 1);
    // Calculate checksum (only for payload)
    closePayload(packetBuffer, 32);
    int checksum = calculateChecksum((char *)(packetBuffer + PACKET_HEADER_SIZE), 28);
    memcpy(packetBuffer + 8, &checksum, 1); // Update checksum in header
    return 40;
}

bool parseCommandPayload(uint8_t *payloadBuffer, unsigned long payloadLength)
{
    // Command packet payload format:
    // 0-3: Command salt (unsigned long)
    // 4-7: Command (int)
    // 8: Command argument byte count (int)
    // 9: Command argument count (int)
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

        for (int i = 0; i < commandArgLength; i++)
        {
            commandArgBuffer[i] = payloadBuffer[10 + i];
        }
    } else if (commandArgCount == 0) {
        // No arguments
    } else {
        Serial.println("!INVALID_COMMAND_ARG_COUNT");
        return false;
    }
    return true;
}

bool sendPacket(uint8_t *packetBuffer, unsigned long packetLength, const uint8_t *macAddr)
{
    if (connectionStage == 1)
    {
        // Packet buffer is uint8_t array, we need to cast it to char array
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
        initErrorLoop();
    }
}
