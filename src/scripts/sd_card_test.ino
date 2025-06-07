#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Pin definitions for ESP32 SD Card (avoiding conflicts with other sensors)
#define SD_CS 15   // Chip Select pin
#define SD_MOSI 13 // Master Out Slave In
#define SD_MISO 12 // Master In Slave Out
#define SD_SCK 14  // Serial Clock

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("ESP32 SD Card Test");
    Serial.println("==================");

    // Initialize SPI with custom pins
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // Initialize SD card
    if (!SD.begin(SD_CS))
    {
        Serial.println("Card Mount Failed");
        return;
    }

    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // Test file operations
    testFileOperations();

    Serial.println("\nSD Card test completed!");
}

void loop()
{
    // Nothing to do here
    delay(1000);
}

void testFileOperations()
{
    Serial.println("\n--- File Operations Test ---");

    // Test 1: Write to file
    Serial.println("1. Testing file write...");
    writeFile(SD, "/test.txt", "Hello ESP32 SD Card!");

    // Test 2: Read from file
    Serial.println("2. Testing file read...");
    readFile(SD, "/test.txt");

    // Test 3: Append to file
    Serial.println("3. Testing file append...");
    appendFile(SD, "/test.txt", "\nAppended line!");
    readFile(SD, "/test.txt");

    // Test 4: List directory
    Serial.println("4. Testing directory listing...");
    listDir(SD, "/", 0);

    // Test 5: File info
    Serial.println("5. Testing file info...");
    getFileInfo(SD, "/test.txt");

    // Test 6: Delete file
    Serial.println("6. Testing file delete...");
    deleteFile(SD, "/test.txt");
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }

    if (file.print(message))
    {
        Serial.println("File written successfully");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("File content: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
    Serial.println();
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }

    if (file.print(message))
    {
        Serial.println("Message appended successfully");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);

    if (fs.remove(path))
    {
        Serial.println("File deleted successfully");
    }
    else
    {
        Serial.println("Delete failed");
    }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }

    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void getFileInfo(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file");
        return;
    }

    Serial.printf("File: %s\n", path);
    Serial.printf("Size: %d bytes\n", file.size());
    Serial.printf("Is Directory: %s\n", file.isDirectory() ? "Yes" : "No");

    file.close();
}