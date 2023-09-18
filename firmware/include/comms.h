#ifndef COMMS_H
#define COMMS_H
#include <WiFi.h>
#include <esp_now.h>
#include "const.h"
#include "common.h"

enum PacketType
{
    PING = 0,
    PONG = 1,
    TELEMETRY = 2,
    COMMAND = 3,
    SETTING = 4,
    ACK = 5,
    CODE = 6,
    HANDSHAKE = 7,
};

extern unsigned long packetTime;
extern PacketType packetType;
extern int payloadLength;
extern int packetChecksum;
extern char *packetPayload;
extern bool packetValid;
extern bool freshPacket;

extern unsigned long commandSalt;
extern int commandInt;
extern Command command;
extern int commandArgLength;
extern int commandArgCount;
extern byte commandArgBuffer[MAX_ARGS*4];

void handshakeResponse(const uint8_t *macAddr);
void setBaseMacAddress(const uint8_t *macAddr);

bool sendMessage(const char *msg, int messageLength, const uint8_t *macAddr);
void broadcast(const String &message);

void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength);
void sendPong(unsigned long pingId);

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen);
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status);
void formPacketHeader(uint8_t *packetBuffer, unsigned long time, int packetType, int payloadLength, int checksum);
unsigned long formTelemetryPacket(uint8_t *packetBuffer, unsigned long time, float altitude, float dAltitude, float gpsLong, float gpsLat, float gpsAltitude, State currentState);
bool sendPacket(uint8_t *packetBuffer, unsigned long packetLength, const uint8_t *macAddr);
int calculateChecksum(char *payload, int payloadLength);
int closePayload(uint8_t *packetBuffer, int payloadLength);

void initComms();
#endif