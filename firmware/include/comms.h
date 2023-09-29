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
extern uint8_t payloadLength;
extern uint8_t packetChecksum;
extern char *packetPayload;
extern bool packetValid;
extern bool freshPacket;

extern unsigned long commandSalt;
extern uint8_t commandInt;
extern Command command;
extern uint8_t commandArgLength;
extern uint8_t commandArgCount;
extern byte commandArgBuffer[MAX_ARGS*4];

void handshakeResponse(const uint8_t *macAddr);
void setBaseMacAddress(const uint8_t *macAddr);

void testComms();

bool sendPacket(uint8_t *packetBuffer, unsigned long packetLength, const uint8_t *macAddr);
bool sendMessage(const char *msg, uint8_t messageLength, const uint8_t *macAddr);
void broadcast(const String &message);

void formatMacAddress(const uint8_t *macAddr, char *buffer, uint8_t maxLength);
void sendPong(unsigned long pingId, const uint8_t *macAddr);

void receiveCallback(const uint8_t *macAddr, const uint8_t *data, uint8_t dataLen);
void sentCallback(const uint8_t *macAddr, esp_now_send_status_t status);

void formPacketHeader(uint8_t *packetBuffer, unsigned long time, uint8_t packetType, uint8_t payloadLength, uint8_t checksum, unsigned long txSalt);
unsigned long formTelemetryPacket(uint8_t *packetBuffer, unsigned long time, float altitude, float dAltitude, float gpsLong, float gpsLat, float gpsAltitude, State currentState);
uint8_t formCmdAckPacket(uint8_t *txBuffer, unsigned long rxSalt, Command rxCmd, bool ack, uint8_t reason);

uint8_t calculateChecksum(char *payload, uint8_t payloadLength);
uint8_t closePayload(uint8_t *packetBuffer, uint8_t payloadLength);

bool parseCommandPayload(uint8_t *payloadBuffer, uint8_t payloadLength, unsigned long rxTime, unsigned long rxSalt);

void initComms();
#endif