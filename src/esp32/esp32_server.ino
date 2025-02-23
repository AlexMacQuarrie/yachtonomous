#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"       // UUID for the service and characteristic
#define INTEGER_CHAR_UUID "12345678-1234-1234-1234-1234567890ab"  // UUID for the integer characteristic

// Timing consts
const int delayMs = 2000;

// Declare pointers
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pIntegerCharacteristic;
BLEAdvertising* pAdvertising;

// Variables to track connected clients
int clientCount = 0;

// Custom callback class to handle characteristic writes
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic* pCharacteristic)
    {
        // When the client writes to the characteristic, we read the value
        String value = pCharacteristic->getValue();
        int receivedInteger = value.toInt();
        Serial.print("Received Integer from client: ");
        Serial.println(receivedInteger);
    }
};

// Custom callback class to handle server connection and disconnection events
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer) 
    {
        // Increment the client count when a client connects
        clientCount++;
        Serial.print("Client connected. Total clients: ");
        Serial.println(clientCount);

        // After connecting, the server stops advertising, so we start it again
        pAdvertising->start();
        Serial.println("Advertising restarted after client connection.");
    }

    void onDisconnect(BLEServer* pServer) 
    {
        // Decrement the client count when a client disconnects
        clientCount--;
        Serial.print("Client disconnected. Total clients: ");
        Serial.println(clientCount);

        // After disconnecting, restart advertising so new clients can connect
        pAdvertising->start();
        Serial.println("Advertising restarted after client disconnection.");
    }
};

void setup()
{
    Serial.begin(115200);
    Serial.println("Server is starting...");

    BLEDevice::init("ESP32C3_Server");

    // Get the MAC address of the device
    String macAddress = BLEDevice::getAddress().toString().c_str();
    Serial.print("Server MAC Address: ");
    Serial.println(macAddress);

    // Create the server and set the callbacks to handle connection events
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the service
    pService = pServer->createService(SERVICE_UUID);

    // Create the characteristic for receiving integer data from the client
    pIntegerCharacteristic = pService->createCharacteristic(
        INTEGER_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE  // This characteristic will be written by the client
    );

    // Set the callback function to handle the characteristic write
    pIntegerCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

    // Start the service and begin advertising
    pService->start();
    pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    Serial.println("Server is advertising...");
}

void loop() 
{
    // The server does nothing here since it advertises and handles events w/ callbacks
    delay(delayMs);
}
