#include <flog.h>

bool writeFLHeader(uint32_t address, uint16_t dataVersion, uint16_t flightID, uint32_t timestamp, uint8_t entryFrequency) {

}

bool writeFLEntry(uint32_t address, uint32_t time, float aX, float aY, float aZ, float gX, float gY, float gZ, float pressure, float temperature, float baroAltitude, float baroDeltaAltitude, byte state, byte pyroState) {

}

bool isSlotEmpty(uint8_t slotID) {

}

bool deleteFlightLog(uint8_t slotID) {

}

uint8_t nextSlotID() {

}

bool createPartitionTable(uint32_t address) {

}

bool createFlightLog(uint8_t slotID, uint16_t flightID, uint32_t timestamp) {

}

uint32_t getFlightLog(uint8_t slotID) {

}
bool closeFlightLog(uint32_t headerStart, uint32_t lastEntry, unsigned long entryCount) {

}

bool readMemoryBlock(uint32_t start, uint32_t end, uint8_t* buffer) {

}

bool writeMemoryBlock(uint32_t start, uint8_t* buffer, unsigned long length) {
    
}