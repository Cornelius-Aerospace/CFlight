#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include "SPIMemory.h"

#define CS_PIN 5 // Pin D5 is connected to the SPI flash chip's CS pin
#define STORAGE_SPACE 16 // Total storage space in MB

SPIFlash storage(CS_PIN);

uint32_t flight_log_header_start; // Address where the current FL header begins
uint32_t flight_log_next_entry; // Address where the next FL entry will begin
unsigned long flight_log_entry_count; // The number of FL entries so far
uint8_t file_slot_id; // The number (0-10) of the current FL's file slot

bool writeFLHeader(uint32_t address, uint16_t dataVersion, uint16_t flightID, uint32_t timestamp, uint8_t entryFrequency);
bool writeFLEntry(uint32_t address, uint32_t time, float aX, float aY, float aZ, float gX, float gY, float gZ, float pressure, float temperature, float baroAltitude, float baroDeltaAltitude, byte state, byte pyroState);

bool isSlotEmpty(uint8_t slotID);
bool deleteFlightLog(uint8_t slotID);
uint8_t nextSlotID();

bool createPartitionTable(uint32_t address); // Creates FL lookup/partition table
bool createFlightLog(uint8_t slotID, uint16_t flightID, uint32_t timestamp); // Creates a new FL (called when armed)
uint32_t getFlightLog(uint8_t slotID); // Reads partition table and returns the address where the FL header begins (returns 0 if FL does not exist or slotID invalid)
bool closeFlightLog(uint32_t headerStart, uint32_t lastEntry, unsigned long entryCount); // Updates FL header with final values (called after flight ends)

bool readMemoryBlock(uint32_t start, uint32_t end, uint8_t* buffer); // Used to read back flight log files to groundstation, in blocks
bool writeMemoryBlock(uint32_t start, uint8_t* buffer, unsigned long length);

#endif