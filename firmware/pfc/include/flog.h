#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#include "SPIMemory.h"

#define CS_PIN 5         // Pin D5 is connected to the SPI flash chip's CS pin
#define STORAGE_SPACE 16 // Total storage space in MB
#define DATA_VERSION 1

SPIFlash storage(CS_PIN);

// Look-up table details
uint32_t lookup_table_begining; // Address where the look-up table begins
uint32_t lookup_table_end;

uint32_t flight_log_header_start;     // Address where the current FL header begins
uint32_t flight_log_next_entry;       // Address where the next FL entry will begin
unsigned long flight_log_entry_count; // The number of FL entries so far
uint8_t flight_log_slot_id;                 // The number of the current FL's file slot
bool flight_log_open;

// Checksum functions
uint8_t XORChecksum8(const byte *data, size_t dataLength);
uint16_t XORChecksum16(const byte *data, size_t dataLength);

// Flight log functions
bool writeFLHeader(uint32_t address, uint16_t dataVersion, uint16_t flightID, uint32_t timestamp, uint8_t entryFrequency, bool errorCheck = true);
bool writeFLEntry(uint32_t address, uint32_t time, float aX, float aY, float aZ, float gX, float gY, float gZ, float pressure, float temperature, float baroAltitude, float baroDeltaAltitude, byte state, byte pyroState, bool errorCheck = true);
bool createFlightLog(uint8_t slotID, uint8_t entryFrequency, uint16_t flightID, uint32_t timestamp);             // Creates a new FL (called when armed)
uint32_t getFlightLog(uint8_t slotID);                                                   // Reads partition table and returns the address where the FL header begins (returns 0 if FL does not exist or slotID invalid)
bool closeFlightLog(uint8_t slotID, uint32_t headerStart, uint32_t lastEntry, unsigned long entryCount); // Updates FL header with final values (called after flight ends)
bool checkFLHeaderChecksum(uint32_t address);
bool updateFLHeaderChecksum(uint32_t address);

// Slot functions
bool createSlot(uint32_t size, byte status);
bool allocateMemory(uint8_t slotID, uint32_t startAddress, uint32_t endAddress, bool ignoreTests = false);
bool isSlotEmpty(uint8_t slotID);
bool updateSlotStatus(uint8_t slotID, byte newStatus, bool errorCheck = true);
bool getSlotStatus(uint8_t slotID, byte &output);
bool deleteSlot(uint8_t slotID, bool errorCheck = true, bool fastRead = false);
uint8_t nextSlotID();

// Lookup table functions
bool createLookupTable(uint32_t address); // Creates FL lookup/partition table
bool updateLookupTableChecksum();
bool doesLookupTableExist();

// Block R/W functions
bool readMemoryBlock(uint32_t start, uint32_t end, uint8_t *buffer, bool fastRead = false); // Used to read back flight log files to groundstation, in blocks
bool writeMemoryBlock(uint32_t start, uint8_t *buffer, size_t length, bool errorCheck = true);

#endif