#include <WiFi.h>
#include <WiFiUdp.h>
#include "BluetoothSerial.h"
#include <Preferences.h>

Preferences preferences;

String ssid = "";
String password = "";
String serverIP = "";
int serverPort = 0;

WiFiUDP udp;
BluetoothSerial SerialBT;


const int MAX_ARRAY_SIZE = 2; // Adjust this based on your expected array size

void setupBluetooth() {
    SerialBT.begin("ESP32_Node"); // Bluetooth device name
    sendLogViaBT("Bluetooth started. Send WiFi credentials and server info.");
}

void sendLogViaBT(const String& log) {
    SerialBT.println(log);
}

void setup() {
    Serial.begin(115200);
    setupBluetooth();
    
    preferences.begin("wifi-config", false);
    loadWiFiConfig();
    
    if (ssid != "" && password != "" && serverIP != "" && serverPort != 0) {
        setupWiFi();
    }
}

void loop() {
    if (SerialBT.available()) {
        String btMessage = SerialBT.readStringUntil('\n');
        handleBluetoothMessage(btMessage);
    }
    
    if (Serial.available()) {
        byte byteArray[MAX_ARRAY_SIZE * sizeof(float)];
        int bytesRead = readByteArray(byteArray, sizeof(byteArray));
        
        if (bytesRead > 0) {
            // Byte array to float array
            // float floatArray[bytesRead / sizeof(float)];
            // memcpy(floatArray, byteArray, bytesRead);
            
            // sendDataToServer(byteArray, bytesRead / sizeof(float));
            sendDataToServer(byteArray, bytesRead);
        }
    }
}

int readByteArray(byte* array, int maxSize) {
    int bytesRead = 0;
    while (Serial.available() && bytesRead < maxSize) {
        array[bytesRead] = Serial.read();
        bytesRead++;
    }
    return bytesRead;
}

void sendDataToServer(byte* data, int length) {
    if (WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(serverIP.c_str(), serverPort);
        udp.write(data, length);
        udp.endPacket();
        sendLogViaBT("Data sent via UDP");
    } else {
        sendLogViaBT("WiFi Disconnected");
    }
}

void sendDataToServer(float* data, int length) {
    // Convert float array to byte array
    byte byteArray[length * sizeof(float)];
    memcpy(byteArray, data, length * sizeof(float));
    sendDataToServer(byteArray, length * sizeof(float));
}

void loadWiFiConfig() {
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", "");
    serverIP = preferences.getString("serverIP", "");
    serverPort = preferences.getInt("serverPort", 0);
}

void saveWiFiConfig() {
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("serverIP", serverIP);
    preferences.putInt("serverPort", serverPort);
}

void handleBluetoothMessage(String message) {
    if (message.startsWith("SSID:")) {
        ssid = message.substring(5);
        sendLogViaBT("SSID set: " + ssid);
    } else if (message.startsWith("PASS:")) {
        password = message.substring(5);
        sendLogViaBT("Password set");
    } else if (message.startsWith("IP:")) {
        serverIP = message.substring(3);
        sendLogViaBT("Server IP set: " + serverIP);
    } else if (message.startsWith("PORT:")) {
        serverPort = message.substring(5).toInt();
        sendLogViaBT("Server port set: " + String(serverPort));
    } else if (message.startsWith("CONNECT")) {
        if (ssid != "" && password != "" && serverIP != "" && serverPort != 0) {
            saveWiFiConfig();
            setupWiFi();
        } else {
            sendLogViaBT("Error: Missing configuration");
        }
    } else if (message.startsWith("DISCONNECT")) {
        WiFi.disconnect();
        sendLogViaBT("Disconnected from WiFi");
    } else if (message.startsWith("STATUS")) {
      String status = "SSID: " + ssid + " PASS: " + password + " IP: " + serverIP + " PORT: " + String(serverPort);
        sendLogViaBT("WiFi status: " + status);
    } else if (message.startsWith("TOSTM32:")) {
        Serial.println(message.substring(8));
        sendLogViaBT("Data sent to STM32: " + message.substring(8));
    } else {
        sendLogViaBT("Unknown command");
    }
}

void setupWiFi() {
  int retry = 0;
    WiFi.begin(ssid.c_str(), password.c_str());
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        sendLogViaBT("Connecting to WiFi...");
        retry++;
        if (retry > 5) {
          sendLogViaBT("Failed to connect to WiFi");
          return;
        }
    }
    sendLogViaBT("Connected to WiFi");
}
