#include <flog.h>

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
// Both XOR checksums are from https://www.luisllamas.es/en/arduino-checksum/
uint8_t XORChecksum8(const byte *data, size_t dataLength)
{
    uint8_t value = 0;
    for (size_t i = 0; i < dataLength; i++)
    {
        value ^= (uint8_t)data[i];
    }
    return ~value;
}

uint16_t XORChecksum16(const byte *data, size_t dataLength)
{
    uint16_t value = 0;
    for (size_t i = 0; i < dataLength / 2; i++)
    {
        value ^= data[2 * i] + (data[2 * i + 1] << 8);
    }
    if (dataLength % 2)
        value ^= data[dataLength - 1];
    return ~value;
}

// - Flight-log functions -
// Writes flight log header to storage, starting at specified address
bool writeFLHeader(uint32_t address, uint16_t dataVersion, uint16_t flightID, uint32_t timestamp, uint8_t entryFrequency, bool errorCheck)
{
    uint8_t header[18];
    header[0] = (dataVersion && 0xFF);
    header[1] = (dataVersion >> 8);
    header[2] = (flightID & 0xFF);
    header[3] = (flightID >> 8);
    memcpy(header + 4, &timestamp, sizeof(timestamp));
    // header[8->11] = entryCount
    header[12] = entryFrequency;
    // header[13->16] = freeAddress (lastEntryAddress + 1)
    // header[17] = checksum (calculate after flight log)
    return storage.writeByteArray(address, header, 18, errorCheck);
}

// Writes a flight log entry to storage, starting at specified address
bool writeFLEntry(uint32_t address, uint32_t time, float aX, float aY, float aZ, float gX, float gY, float gZ, float pressure, float temperature, float baroAltitude, float baroDeltaAltitude, byte state, byte pyroState, bool errorCheck)
{
    uint8_t entry[46];
    memcpy(entry, &time, 4U);
    // Accelerometer
    memcpy(entry + 4, &aX, 4U);
    memcpy(entry + 8, &aY, 4U);
    memcpy(entry + 12, &aZ, 4U);
    // Gyroscope
    memcpy(entry + 16, &gX, 4U);
    memcpy(entry + 20, &gY, 4U);
    memcpy(entry + 24, &gZ, 4U);
    // Barometer
    memcpy(entry + 28, &pressure, 4U);
    memcpy(entry + 32, &temperature, 4U);
    memcpy(entry + 36, &baroAltitude, 4U);
    memcpy(entry + 40, &baroDeltaAltitude, 4U);

    entry[44] = state;
    entry[45] = pyroState;

    return storage.writeByteArray(address, entry, 46, errorCheck);
}

bool createFlightLog(uint8_t slotID, uint8_t entryFrequency, uint16_t flightID, uint32_t timestamp)
{
    flight_log_header_start = getFlightLog(slotID);

    if (flight_log_header_start == 0)
        return false;

    if (!writeFLHeader(flight_log_header_start, DATA_VERSION, flightID, timestamp, entryFrequency))
        return false;

    byte slotStatus;
    if (!getSlotStatus(slotID, slotStatus))
        return false;

    slotStatus = slotStatus & 0b11111000; // Remove any incorrect file-types
    slotStatus = slotStatus | 0b10001000; // Add contains data flag and flight-log filetype (in case it has not been set)
    if (!updateSlotStatus(slotID, slotStatus))
        return false;

    flight_log_next_entry = flight_log_header_start + 18;
    flight_log_slot_id = slotID;
    flight_log_open = true;
    flight_log_entry_count = 0;
    return true;
}

uint32_t getFlightLog(uint8_t slotID)
{
    uint8_t lastSlotID = storage.readByte(lookup_table_begining);
    if (slotID == 0 || slotID > lastSlotID)
        return 0;

    return storage.readULong(lookup_table_begining + 10 + (9 * slotID));
}

bool closeFlightLog(uint8_t slotID, uint32_t headerStart, uint32_t lastEntry, unsigned long entryCount)
{
    if (!storage.writeULong(headerStart + 8, entryCount))
        return false;
    if (!storage.writeULong(headerStart + 13, lastEntry + 1))
        return false;
    byte slotStatus;
    if (!getSlotStatus(slotID, slotStatus))
        return false;

    slotStatus = slotStatus & 0b10111000; // Remove "selected" flag and any incorrect file-types
    slotStatus = slotStatus | 0b10001000; // Add contains data flag and flight-log filetype (in case it has not been set)
    if (!updateSlotStatus(slotID, slotStatus))
        return false;

    return updateFLHeaderChecksum(headerStart);
}

bool checkFLHeaderChecksum(uint32_t address)
{
    uint8_t header[17];
    if (!storage.readByteArray(address, header, 17))
        return false;
    uint8_t readChecksum = storage.readByte(address + 17);
    uint8_t calculatedChecksum = XORChecksum8(header, 17);
    return readChecksum == calculatedChecksum;
}

bool updateFLHeaderChecksum(uint32_t address)
{
    uint8_t header[17];
    if (!storage.readByteArray(address, header, 17))
        return false;
    uint8_t checksum = XORChecksum8(header, 17);
    return storage.writeByte(address + 17, checksum);
}

// - Slot functions -
// Creates a slot (with the next slotID) of specified size (in bytes) starting at the address
// directly after the previous slot (or the lookup table if there are no other slots).
// Updates the lookup table and sets the status bytes
bool createSlot(uint32_t size, byte status)
{
    uint8_t slotID = storage.readByte(lookup_table_begining);
    slotID++;

    if (!storage.writeByte(lookup_table_begining, slotID))
    {
        Serial.println("ERROR: Failed to update slotID while creating new slot!");
        return false;
    }

    uint8_t slotRegister[9];
    uint32_t startAddress;
    uint32_t endAddress;

    if (slotID == 1)
    {
        startAddress = lookup_table_end + 1;
    }
    else
    {
        startAddress = storage.readULong(lookup_table_begining + 10);
    }

    endAddress = startAddress + size;
    memcpy(slotRegister, &startAddress, 4U);
    memcpy(slotRegister + 4, &endAddress, 4U);
    slotRegister[8] = status;

    if (slotID == 1)
    {
        if (!storage.writeByteArray(lookup_table_begining + 10, slotRegister, 9))
            return false;
    }
    else
    {
        if (!storage.writeByteArray(lookup_table_begining + 10 + (9 * (slotID - 1)), slotRegister, 9))
            return false;
    }

    uint32_t freeSpace = storage.readULong(lookup_table_begining + 5);
    freeSpace -= size;

    if (!storage.writeULong(lookup_table_begining + 5, freeSpace))
        return false;
    return updateLookupTableChecksum();
}

// Updates an existing slot's allocated memory/ address range (and updates
// the slot register) WARNING: avoid using this! It is missing some needed safe-gaurd tests
// (such as checking if the new address range overlaps with allocated memory)
// Only use if you know the address range is unallocated and does not overlap with anything.
bool allocateMemory(uint8_t slotID, uint32_t startAddress, uint32_t endAddress, bool ignoreTests)
{
    
    if (!ignoreTests && startAddress >= endAddress)
    {
        Serial.printf("ERROR: Tried to allocate storage to slot %u but the start address (%X) is larger than (or equal to) the end address (%X)!\n", slotID, startAddress, endAddress);
        return false;
    }

    uint32_t capacity = storage.getCapacity();
    if (!ignoreTests && (startAddress <= lookup_table_end || endAddress <= lookup_table_end))
    {
        Serial.printf("ERROR: Tried to allocate storage to slot %u between %X -> %X , one or more addresses overlap with lookup table (%X -> %X)!\n", slotID, startAddress, endAddress, lookup_table_begining, lookup_table_end);
        return false;
    }
    else if (!ignoreTests && (startAddress >= capacity || endAddress >= capacity))
    {
        Serial.printf("ERROR: Tried to allocate storage to slot %u between %X -> %X, one or more addresses are larger than max address (%X)!\n", slotID, startAddress, endAddress, capacity);
        return false;
    }

    uint8_t lastSlotID = storage.readByte(lookup_table_begining);
    if (!ignoreTests && (slotID > lastSlotID || slotID == 0))
    {
        Serial.printf("ERROR: Tried to allocate storage to slot %u between %X -> %X, slot does not exist (last slotID = %u)!\n", slotID, startAddress, endAddress, lastSlotID);
        return false;
    }

    // TODO: check each slot register to see if requested memory range overlaps with any existing slot allocation?

    // Update the freeSpace counter in the look-up table
    uint32_t oldStartAddress = storage.readULong(lookup_table_begining + 10 + (9 * slotID));
    uint32_t oldEndAddress = storage.readULong(lookup_table_begining + 10 + (9 * slotID) + 4);
    uint32_t oldSize = oldEndAddress - oldStartAddress;
    uint32_t newSize = endAddress - startAddress;

    uint32_t freeSpace = storage.readULong(lookup_table_begining + 5);
    freeSpace -= oldSize;
    freeSpace += newSize;
    if (!storage.writeULong(lookup_table_begining + 5, freeSpace))
        return false;

    if (!storage.writeULong(lookup_table_begining + 10 + (9 * slotID), startAddress))
        return false;
    if (!storage.writeULong(lookup_table_begining + 10 + (9 * slotID) + 4, endAddress))
        return false;
    if (!updateLookupTableChecksum())
        return false;

    Serial.printf("DEBUG: Updated slot %u address space from %X -> %X to %X -> %X; Old slot size: %f kB, new slot size: %f kB.", slotID, oldStartAddress, oldEndAddress, startAddress, endAddress, (float)oldSize / 1000.0, (float)newSize / 1000.0);
    return true;
}

bool isSlotEmpty(uint8_t slotID)
{
    uint8_t lastSlotID = storage.readByte(lookup_table_begining);
    if (slotID > lastSlotID || slotID == 0)
    {
        Serial.printf("WARNING: checked if slot %u is empty, slot does not exist\n", slotID);
        return true;
    }

    byte slotStatus = storage.readByte(lookup_table_begining + 10 + (9 * slotID) + 8);
    return (1 & slotStatus) != 1; // Check the first bit of status byte (if set to 1 the slot is not empty)
}

bool updateSlotStatus(uint8_t slotID, byte newStatus, bool errorCheck)
{
    uint8_t lastSlotID = storage.readByte(lookup_table_begining);
    if (slotID > lastSlotID || slotID == 0)
    {
        Serial.printf("ERROR: Tried to update status of slot %u to %p (%X), slot does not exist\n", slotID, newStatus);
        return false;
    }

    return storage.writeByte(lookup_table_begining + 10 + (9 * slotID) + 8, newStatus, errorCheck);
}

bool getSlotStatus(uint8_t slotID, byte &output)
{
    uint8_t lastSlotID = storage.readByte(lookup_table_begining);
    if (slotID > lastSlotID || slotID == 0)
    {
        Serial.printf("ERROR: Tried to get status of slot %u, slot does not exist\n", slotID);
        return false;
    }

    output = storage.readByte(lookup_table_begining + 10 + (9 * slotID) + 8);
    return true;
}

uint8_t nextSlotID()
{
    return storage.readByte(lookup_table_begining) + 1;
}

// TODO: reevaluate how we handle this
// Currently the slot's status byte is changed and the address space set to 0 -> 0. 
// The slot register cannot be deleted currently (may cause issues with rest of code)
// and there is no method for the space to be automatically reused.
// A better system would defragment the memory, moving the slots after this one back (to reuse the memory space)
// Maybe we should switch to a PROPER file system?
// For now we will work with this, keeping as little slots as posible and deleting the first slot ASAP
// We can then just create the next slot at position zero (reuse the register and reset everything)
// Avoid deleting slots that are "sandwhiched" between other slots (ie dont delete a slot that has slots on each side of it)
bool deleteSlot(uint8_t slotID, bool errorCheck, bool fastRead)
{
    byte slotStatus;
    if (!getSlotStatus(slotID, slotStatus))
        return false;

    if (slotStatus & 0b00010000 != 0)
    {
        Serial.printf("WARNING: tried to delete protected slot %u (slot status: %p/ %X)", slotID, slotStatus, slotStatus);
        return false;
    }

    if (!updateSlotStatus(slotID, 0b10000000, errorCheck))
        return false;

    return allocateMemory(slotID, 0, 0, true);
}

// - Lookup table functions -
bool createLookupTable(uint32_t address)
{
    // Lookup table format
    // 0: slot count (int)
    // 1-4: storage capacity (uint32_t)
    // 5-8: free space (uint32_t)
    // 9: checksum/ validity test (byte)
    // 10+: Slot register
    uint8_t lookupTable[9];
    uint32_t capacity = storage.getCapacity();
    memcpy(lookupTable + 1, &capacity, 4U);
    memcpy(lookupTable + 5, &capacity, 4U); // Free space = capacity as there are no slots yet
    if (!storage.writeByteArray(address, lookupTable, 9))
        return false;
    return updateLookupTableChecksum();
}

bool updateLookupTableChecksum()
{
    uint8_t lookupTable[9];
    storage.readByteArray(lookup_table_begining, lookupTable, 9);
    byte calculatedChecksum = XORChecksum8(lookupTable, 9);
    return storage.writeByte(lookup_table_begining + 9, calculatedChecksum);
}

bool doesLookupTableExist()
{
    uint8_t lookupTable[9];
    storage.readByteArray(lookup_table_begining, lookupTable, 9);
    byte readChecksum = storage.readByte(lookup_table_begining + 9);
    byte calculatedChecksum = XORChecksum8(lookupTable, 9);
    return readChecksum == calculatedChecksum;
}

// - Block R/W functions -
bool readMemoryBlock(uint32_t start, uint32_t end, uint8_t *buffer, bool fastRead)
{
    uint32_t capacity = storage.getCapacity();
    if (start > end || start >= capacity || end >= capacity)
        return false;

    size_t blockSize = end - start;
    return storage.readByteArray(start, buffer, blockSize, fastRead);
}

bool writeMemoryBlock(uint32_t start, uint8_t *buffer, size_t length, bool errorCheck)
{
    uint32_t capacity = storage.getCapacity();
    uint32_t end = start + length;
    if (end >= capacity || length < 0)
        return false;

    return storage.writeByteArray(start, buffer, length, errorCheck);
}