#include <BLEDevice.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>  // Include the ArduinoJson library

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"       // UUID for the service and characteristic
#define INTEGER_CHAR_UUID "12345678-1234-1234-1234-1234567890ab"  // UUID for the integer characteristic

// Timing consts
const int delayMs = 5;

StaticJsonDocument<200> doc;
JsonArray arr;  // Declare the JSON array

// Wifi config
const char* ssid = "PicoW_Network";      // Replace with your Raspberry Pi Pico W SSID
const char* password = "micropython123"; // Replace with your Raspberry Pi Pico W password

// UDP parameters
WiFiUDP udp;
const char* udpAddress = "192.168.4.1";   // Raspberry Pi Pico W IP (replace with the correct IP)
const int udpPort = 12345;                 // Port to send data on

// Declare pointers
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pIntegerCharacteristic;
BLEAdvertising* pAdvertising;

// Variables to track connected clients
int clientCount = 0;

void updateRSSIList(int newRSSI)
{
    int cli_id   = newRSSI / 1000;
    int rssi_val = newRSSI % 1000;
    arr[cli_id]  = rssi_val;
}

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

        // Update list of RSSI values for all clients
        updateRSSIList(receivedInteger);

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

    arr = doc.createNestedArray("numbers");
    arr.add(0);
    arr.add(0);
    arr.add(0);
    arr.add(0);

    BLEDevice::init("ESP32C3_Server");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
    udp.begin(udpPort);

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
    StaticJsonDocument<200> doc;
    // Add key-value pairs to the JSON object
    doc["command"] = "ESP32";
    doc["number"] = arr;

    // Serialize the JSON object into a string
    String jsonMessage;
    serializeJson(doc, jsonMessage);

    // Send the JSON message over UDP to the Raspberry Pi Pico W
    udp.beginPacket(udpAddress, udpPort);
    udp.write(reinterpret_cast<const uint8_t*>(jsonMessage.c_str()), jsonMessage.length());
    udp.endPacket();

    Serial.print("Sent JSON message: ");
    Serial.println(jsonMessage);

    delay(delayMs);
}