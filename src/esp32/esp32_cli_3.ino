#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"       // UUID for the service and characteristic
#define INTEGER_CHAR_UUID "12345678-1234-1234-1234-1234567890ab"  // UUID for the integer characteristic

// The MAC address of the server device
#define SERVER_MAC_ADDRESS "24:ec:4a:c9:97:12"

// Client ID consts
const str cliID = "3";
const str cliName = "ESP32C3_Client_3";

// Timing consts
const int delayMs = 200;

// Declare pointers
BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pRemoteCharacteristic;

// Start with a default invalid value
int currentRSSI = -1000; 

// This function is called when the server is found and connected
bool connectToServer()
{
    BLEAdvertisedDevice* advertisedDevice = nullptr;

    // Start scanning for 5 seconds
    BLEDevice::getScan()->start(5, false); 

    // Get the results of the scan
    BLEScanResults* scanResults = BLEDevice::getScan()->getResults();

    for (int i = 0; i < scanResults->getCount(); i++) 
    {
        BLEAdvertisedDevice device = scanResults->getDevice(i);
        if (device.getAddress().toString() == SERVER_MAC_ADDRESS) 
        {
            advertisedDevice = &device;  // Use pointer to BLEAdvertisedDevice
            break;
        }
    }

    if (advertisedDevice) 
    {
        pClient = BLEDevice::createClient();
        if (pClient->connect(advertisedDevice)) 
        {
            Serial.println("Connected to server successfully.");
            return true;
        }
        else
        {
            Serial.println("Failed to connect to server.");
            return false;
        }
    }

    Serial.println("Server not found during scan.");
    return false;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Client is starting...");

    BLEDevice::init(cliName);
}

void loop() 
{
    // Continuously check RSSI and maintain connection
    if (pClient && pClient->isConnected()) 
    {
        // Retrieve the RSSI value from the client object
        int rssi = pClient->getRssi();

        if (rssi != currentRSSI)
        {
            currentRSSI = rssi;
            Serial.print("Current RSSI: ");
            Serial.println(currentRSSI);
        }

        // After connecting, we can get the service and characteristic
        pRemoteService = pClient->getService(SERVICE_UUID);
        if (pRemoteService) 
        {
            pRemoteCharacteristic = pRemoteService->getCharacteristic(INTEGER_CHAR_UUID);
        }
        else
        {
            Serial.println("Service not found.");
        }

        // Write the (positive) integer value to the server in  the correct format
        String value = String(-1*currentRSSI);
        while(value.length() < 3)
        {
            value = "0" + value;
        }
        value = cliID + value;
        Serial.println(value);

        String dataToSend = value;
        pRemoteCharacteristic->writeValue(dataToSend.c_str(), dataToSend.length());

    } 
    else
    {
        Serial.println("Client not connected. Attempting to reconnect...");
        if (connectToServer()) 
        {
            Serial.println("Reconnected to server.");
        }
    }

    delay(delayMs);  // Wait for a while before checking the RSSI again
}
