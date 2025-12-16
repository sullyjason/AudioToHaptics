/* 

Version 0.1
16.12.2025

SBS HAPTICS / Wifi_Actuators.ino

Compatible with Wifi_Actuators.html

Compatible with Seeed Studios Xiao ESP32S3 and Adafruit Audio BFF
https://learn.adafruit.com/adafruit-audio-bff

See hardware folder for details

-----

Created in 2025 by Silvan Jason Roth for

Dr Jacqueline Borgstedt
SBS Lab, ETH Zürich

sbs.ethz.ch/people/postdocs/jacqueline-borgstedt.html
silvanjason.me/

*/


#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/i2s.h"

// --- USER CONFIGURATION ---
const char* ssid        = "SSID";                        // Enter Wifi credentials here
const char* password    = "PASSWORD";                    // Use the same wifi network as the computer you are running the dashboard on
const char* mqtt_server = "broker.emqx.io"; 
const char* mqtt_topic_sub  = "SBShaptics/1";            // CONFIGURE THIS
const char* mqtt_topic_pub  = "SBShaptics/1/files";      // AND THIS. Eg Device 2 should be called SBShaptics/2 and SBShaptics/2/files

// --- PINS ---
#define SD_CS      1
#define I2S_BCLK   A3   
#define I2S_LRC    A2   
#define I2S_DOUT   A1   
#define STATUS_LED LED_BUILTIN 

#define SAMPLE_RATE 44100 

WiFiClient espClient;
PubSubClient client(espClient);
File audioFile;

// --- STATE ---
bool isPlaying = false;
bool shouldLoop = false; 
bool manualMode = false; 
uint8_t i2s_buffer[1024];
String deviceID; 

void flashLED(int times) {
    for(int i=0; i<times; i++) {
        digitalWrite(STATUS_LED, LOW); delay(100); digitalWrite(STATUS_LED, HIGH); delay(100);
    }
}

// --- HELPER: List Files ---
void listWavFilesSerial(File dir) {
    Serial.println("------ SD CARD FILES ------");
    dir.rewindDirectory();
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break; 
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
            
            // FIX: Create a separate uppercase string for checking
            String upperName = fname;
            upperName.toUpperCase();

            if (!fname.startsWith("._") && upperName.endsWith(".WAV")) {
                Serial.print("\t"); Serial.println(fname);
            }
        }
        entry.close();
    }
    Serial.println("---------------------------");
}

void publishFileListMQTT() {
    String fileList = "";
    File root = SD.open("/");
    root.rewindDirectory();
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
            
            // FIX: Create a separate uppercase string for checking
            String upperName = fname;
            upperName.toUpperCase();

            if (!fname.startsWith("._") && upperName.endsWith(".WAV")) {
                if (fileList.length() > 0) fileList += ",";
                fileList += fname;
            }
        }
        entry.close();
    }
    root.close();
    Serial.print(">> Sharing list of .wav files: "); Serial.println(fileList);
    client.publish(mqtt_topic_pub, fileList.c_str());
}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK, .ws_io_num = I2S_LRC, .data_out_num = I2S_DOUT, .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void stopPlayback() {
    if (isPlaying) {
        size_t bytes_written;
        memset(i2s_buffer, 0, sizeof(i2s_buffer));
        i2s_write(I2S_NUM_0, i2s_buffer, sizeof(i2s_buffer), &bytes_written, portMAX_DELAY);
        i2s_zero_dma_buffer(I2S_NUM_0);
        audioFile.close();
        isPlaying = false; shouldLoop = false; 
        Serial.println("Playback finished/stopped.");
    }
}

void startPlayback(const char* filename, bool loopMode) {
    if (isPlaying) audioFile.close(); // Close previous if running
    
    // Check for stop command
    String sName = String(filename);
    if (sName == "/stop.wav" || sName == "/stop") {
        stopPlayback();
        return;
    }

    if (!SD.exists(filename)) {
        Serial.print("ERROR: Not found -> "); Serial.println(filename);
        flashLED(5); return;
    }

    audioFile = SD.open(filename);
    audioFile.seek(44); 
    isPlaying = true;
    shouldLoop = loopMode; 
    
    Serial.print(shouldLoop ? "LOOPING: " : "PLAYING: "); Serial.println(filename);
    flashLED(1);
}

// --- COMMAND PARSER (Shared by MQTT and Serial) ---
void parseCommand(String message) {
    message.trim(); // Remove whitespace/newlines
    Serial.print("CMD RX: "); Serial.println(message);

    if (message == "list") {
        if(manualMode) {
             File root = SD.open("/");
             listWavFilesSerial(root);
             root.close();
        } else {
             publishFileListMQTT();
        }
        return;
    }

    String cleanName;
    bool requestLoop = false;
    
    if (message.startsWith("loop:")) {
        requestLoop = true;
        cleanName = message.substring(5); 
    } else {
        requestLoop = false;
        cleanName = message;
    }
    
    // Logic to handle "stop" or filenames
    String filename = "/" + cleanName;
    if (!filename.endsWith(".wav") && !filename.endsWith(".WAV") && cleanName != "stop") {
        filename += ".wav";
    }
    
    if(cleanName == "stop") filename = "/stop.wav";

    startPlayback(filename.c_str(), requestLoop);
}

// --- MQTT CALLBACK ---
void callback(char* topic, byte* payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    parseCommand(String(msg));
}

void reconnect() {
    if (!client.connected()) {
        Serial.print("MQTT Disconnected. Trying to connect with ID: "); Serial.print(deviceID); Serial.print("... ");
        if (client.connect(deviceID.c_str())) {
            Serial.println("CONNECTED!");
            client.subscribe(mqtt_topic_sub);
            flashLED(3);
        } else {
            Serial.print("Failed rc="); Serial.println(client.state());
        }
    }
}

// --- Serial Input Handler ---
void checkSerialCommand() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parseCommand(cmd);
    }
}

void setup() {
    pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, HIGH);
    Serial.begin(115200);
    
    // Wait for Serial
    while(!Serial) { delay(10); }

    deviceID = "Titan-" + String(random(0xffff), HEX);

    Serial.println("\n\n=== SBS HAPTIC ACTUATOR START ===");
    
    // 1. Setup Hardware (Common to both modes)
    pinMode(SD_CS, OUTPUT);
    if (!SD.begin(SD_CS)) {
        Serial.println("SD MOUNT FAILED! Halting.");
        while(1) { flashLED(1); delay(100); }
    }
    Serial.println("SD Mounted - OK.");
    
    setupI2S();
    Serial.println("I2S Driver - OK.");

    // List files immediately for diagnostics
    File root = SD.open("/");
    listWavFilesSerial(root);
    root.close();

    // 2. MODE SELECTION
    Serial.println("\n\n*** PRESS ANY KEY NOW FOR MANUAL MODE ***");
    Serial.println("\n... Waiting 3 seconds ...\n");
    
    unsigned long startWait = millis();
    while (millis() - startWait < 3000) {
        if (Serial.available()) {
            // Read and discard the input trigger
            while(Serial.available()) Serial.read(); 
            manualMode = true;
            flashLED(5);
            break;
        }
        delay(10);
    }

    if (manualMode) {
        Serial.println("\n#####################################");
        Serial.println("#                                     #");
        Serial.println("#   MANUAL MODE ACTIVE                #");
        Serial.println("#                                     #");
        Serial.println("#   Type 'filename', 'loop:filename'  #");
        Serial.println("#   or 'stop' to control.             #");
        Serial.println("#                                     #");  
        Serial.println("#######################################\n");
    } else {
        // 3. WiFi/MQTT Setup (Only if not manual)
        Serial.println(">> Continuing to WiFi Mode...");
        Serial.print("Connecting to "); Serial.println(ssid);
        WiFi.begin(ssid, password);
        
        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 20) {
            delay(500); Serial.print("."); retries++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(" WiFi Connected.");
            Serial.print("IP: "); Serial.println(WiFi.localIP());
            client.setServer(mqtt_server, 1883);
            client.setCallback(callback);
            client.setKeepAlive(60); 
            client.setSocketTimeout(60);
        } else {
            Serial.println("\nWiFi Failed. Switching to manual mode.");
            manualMode = true; // Fallback to manual if WiFi fails
        }
    }
}

void loop() {
    // --- CONTROL LOOP ---
    if (manualMode) {
        // Listen to USB Serial
        checkSerialCommand();
    } else {
        // Listen to MQTT Broker
        if (!client.connected()) {
            static unsigned long lastReconnect = 0;
            if (millis() - lastReconnect > 5000) {
                reconnect();
                lastReconnect = millis();
            }
        } else {
            client.loop();
        }
    }

    // --- AUDIO LOOP (Non-blocking, works in both modes) ---
    if (isPlaying) {
        if (audioFile.available()) {
            // Read small chunk
            size_t bytes_read = audioFile.read(i2s_buffer, sizeof(i2s_buffer));
            size_t bytes_written;
            // Send to I2S (DMA)
            i2s_write(I2S_NUM_0, i2s_buffer, bytes_read, &bytes_written, 0); 
        } else {
            // End of file reached
            if (shouldLoop) {
                audioFile.seek(44); // Loop back to start
            } else {
                stopPlayback();
            }
        }
    } else {
        // Yield slightly when idle to prevent watchdog crashes
        delay(10); 
    }
}
