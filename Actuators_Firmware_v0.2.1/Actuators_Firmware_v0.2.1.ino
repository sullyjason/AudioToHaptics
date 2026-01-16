/*
SBS Haptic Actuator Firmware 
v0.2.1 - Robust SD & Non-Blocking Boot

Compatible with Actuator_Dashboard.html v0.1.0

Written for Seeed Studios Xiao ESP32S3 and Adafruit Audio BFF
https://learn.adafruit.com/adafruit-audio-bff
See hardware folder for details

Capabilities:
 * 1. Hybrid Control: Works over WiFi (MQTT) OR USB (Serial).
 * 2. Waveform file Playback: Plays .wav files from SD card via I2S.

-----
Created in 2025 by Silvan Jason Roth for Dr Jacqueline Borgstedt, SBS Lab, ETH Zürich
*/

// --- LIBRARIES ---
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/i2s.h"

// --- USER CONFIGURATION ---
const char* ssid        = "SSID";           // Wifi Name
const char* password    = "PASSWORD";       // Wifi password
const char* mqtt_server = "broker.emqx.io"; 
// Topics are specific to Device 1. Change these for Device 2, 3, etc.
const char* mqtt_topic_sub  = "SBShaptics/1";       
const char* mqtt_topic_pub  = "SBShaptics/1/files"; 

// --- PIN DEFINITION ---
// Specific to Adafruit SD Card/Amp "BFF". Change if using other hardware.
#define SD_CS      1        // Chip Select for SD Card

// I2S Audio Pins
#define I2S_BCLK   A3       // Bit Clock
#define I2S_LRC    A2       // Left/Right Clock (Word Select)
#define I2S_DOUT   A1       // Data Out (to Amplifier)

#define STATUS_LED LED_BUILTIN 

// Audio Configuration
#define SAMPLE_RATE 44100   // Standard CD quality audio sample rate

// --- GLOBAL OBJECTS ---
WiFiClient espClient;           // TCP Network Client
PubSubClient client(espClient); // MQTT Client wrapper
File audioFile;                 // Handle for the currently open .wav file

// --- STATE VARIABLES ---
bool isPlaying = false;         // Flag: Is audio currently running?
bool shouldLoop = false;        // Flag: Should we restart the file when it ends?
bool manualMode = false;        // Flag: True = USB control, False = WiFi/MQTT control
bool sdAvailable = false;       // Tracks if SD is working or not
uint8_t i2s_buffer[1024];       // Buffer to hold audio data chunk before sending to I2S
String deviceID;                // Unique ID generated at startup
float masterVolume = 1.0;       // Volume multiplier (0.0 to 1.0)

/**
 * Helper: Blinks the onboard LED
 * Used to indicate errors or status changes.
 */
void flashLED(int times) {
    for(int i=0; i<times; i++) {
        digitalWrite(STATUS_LED, LOW); 
        delay(100); 
        digitalWrite(STATUS_LED, HIGH); 
        delay(100);
    }
}

/**
 * Helper: Lists files to Serial Monitor
 * Filters out hidden files (start with ._) and non-WAV files.
 */
void listWavFilesSerial(File dir) {
    if (!sdAvailable) {
        Serial.println("Warning: Cannot list files. SD Card not available.");
        return;
    }
    Serial.println("------ SD CARD FILES ------");
    dir.rewindDirectory(); // Go back to start of directory
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break; // No more files
        
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
            
            // Convert to uppercase for case-insensitive checking
            String upperName = fname;
            upperName.toUpperCase();

            // Filter: Must not start with "._" (Mac hidden files) AND must end in .WAV
            if (!fname.startsWith("._") && upperName.endsWith(".WAV")) {
                Serial.print("\t"); Serial.println(fname);
            }
        }
        entry.close();
    }
    Serial.println("---------------------------");
}

/**
 * Helper: Publish file list to MQTT
 * reads SD card, creates a CSV string (file1.wav,file2.wav), and sends it to broker, Reports error string if SD card is missing.
 */
void publishFileListMQTT() {
    // If SD is missing, report the error to the dashboard
    if (!sdAvailable) {
        client.publish(mqtt_topic_pub, "ERROR: NO SD CARD MOUNTED");
        return;
    }

    String fileList = "";
    File root = SD.open("/");
    if(!root) {
        client.publish(mqtt_topic_pub, "ERROR: SD READ FAIL");
        return;
    }
    
    root.rewindDirectory();
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
            String upperName = fname;
            upperName.toUpperCase();

            if (!fname.startsWith("._") && upperName.endsWith(".WAV")) {
                // Add comma separator if this isn't the first file
                if (fileList.length() > 0) fileList += ",";
                fileList += fname;
            }
        }
        entry.close();
    }
    root.close();
    Serial.print(">> Sharing list of .wav files: "); Serial.println(fileList);
    // Publish the CSV string to the specific topic
    client.publish(mqtt_topic_pub, fileList.c_str());
}

/**
 * I2S Initialization
 * Configures the ESP32 audio hardware driver.
 */
void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // Master Transmitter
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,        // Stereo
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,   // Standard I2S protocol
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,            // Interrupt priority
        .dma_buf_count = 8,                                  // Number of DMA buffers
        .dma_buf_len = 64,                                   // Size of each buffer
        .use_apll = false,                                   // Don't use Audio PLL (saves power)
        .tx_desc_auto_clear = true                           // Auto silence on underflow
    };
    
    // Define which ESP32 pins map to which I2S signals
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK, 
        .ws_io_num = I2S_LRC, 
        .data_out_num = I2S_DOUT, 
        .data_in_num = I2S_PIN_NO_CHANGE // We are not recording audio, so input is unused
    };
    
    // Install driver and apply settings
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

/**
 * Stops Audio Logic
 * Clears buffers, closes files, and resets flags.
 */
void stopPlayback() {
    if (isPlaying) {
        size_t bytes_written;
        // Fill buffer with zeros (silence)
        memset(i2s_buffer, 0, sizeof(i2s_buffer));
        // Flush the I2S driver to ensure sound stops immediately
        i2s_write(I2S_NUM_0, i2s_buffer, sizeof(i2s_buffer), &bytes_written, portMAX_DELAY);
        i2s_zero_dma_buffer(I2S_NUM_0);
        
        if (sdAvailable) audioFile.close();
        
        isPlaying = false; 
        shouldLoop = false; 
        Serial.println("Playback finished/stopped.");
    }
}

/**
 * Starts Audio Logic
 * Opens file, skips header, and sets flags.
 */
void startPlayback(const char* filename, bool loopMode) {
    // 1. Guard Clause: Check SD Status
    if (!sdAvailable) {
        Serial.println("ERROR: Cannot play. No SD Card.");
        flashLED(5); // Fast error blink to indicate hardware failure
        return;
    }

    if (isPlaying) audioFile.close(); // Safety: Close any currently running file
    
    // Check if the command was actually to STOP
    String sName = String(filename);
    if (sName == "/stop.wav" || sName == "/stop") {
        stopPlayback();
        return;
    }

    // Verify file exists on SD
    if (!SD.exists(filename)) {
        Serial.print("ERROR: Not found -> "); Serial.println(filename);
        flashLED(5); // Fast blink error
        return;
    }

    audioFile = SD.open(filename);
    
    // IMPORTANT: Skip the first 44 bytes of a WAV file.
    // These bytes contain header info (metadata), not audio data. 
    audioFile.seek(44); 
    
    isPlaying = true;
    shouldLoop = loopMode; 
    
    Serial.print(shouldLoop ? "LOOPING: " : "PLAYING: "); Serial.println(filename);
    flashLED(1); // Short blink confirmation
}

/**
 * Helper: Send CSV list over Serial
 * Used specifically for the Web Serial API (USB Mode)
 */
void sendFileListSerial() {
    if (!sdAvailable) {
        Serial.println("FILES:ERROR_NO_SD");
        return;
    }
    String fileList = "";
    File root = SD.open("/");
    root.rewindDirectory();
    while (true) {
        File entry = root.openNextFile();
        if (!entry) break;
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
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
    // Protocol: The web interface looks for the "FILES:" prefix to identify this data
    Serial.print("FILES:"); Serial.println(fileList); 
}

/**
 * Main Command Parser
 * Deciphers strings like "list", "vol:80", "loop:track1.wav"
 */
void parseCommand(String message) {
    message.trim(); // Remove whitespace/newlines
    Serial.print("CMD RX: "); Serial.println(message);

    // 1. Volume Handler
    if (message.startsWith("vol:")) {
        int volPercent = message.substring(4).toInt();
        // Clamp values between 0 and 100
        if (volPercent < 0) volPercent = 0;
        if (volPercent > 100) volPercent = 100;
        masterVolume = volPercent / 100.0; // Convert to float (0.0 - 1.0)
        Serial.print("Volume set to: "); Serial.println(masterVolume);
        return;
    }

    // 2. List Handler
    if (message == "list") {
        sendFileListSerial(); // Always send to Serial
        if(!manualMode) publishFileListMQTT(); // Only publish to MQTT if we are online
        return;
    }

    // 3. Playback Handler
    String cleanName;
    bool requestLoop = false;
    
    // Check for "loop:" prefix
    if (message.startsWith("loop:")) {
        requestLoop = true;
        cleanName = message.substring(5); // Remove "loop:"
    } else {
        requestLoop = false;
        cleanName = message;
    }
    
    // Format filename (add leading slash and .wav extension if missing)
    String filename = "/" + cleanName;
    if (!filename.endsWith(".wav") && !filename.endsWith(".WAV") && cleanName != "stop") {
        filename += ".wav";
    }
    // Handle "stop" keyword
    if(cleanName == "stop") filename = "/stop.wav";

    // Trigger playback
    startPlayback(filename.c_str(), requestLoop);
}

// --- MQTT CALLBACK ---
// Called automatically when a message arrives on the subscribed topic
void callback(char* topic, byte* payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0'; // Null-terminate the string
    parseCommand(String(msg)); // Pass to the common parser
}

/**
 * MQTT Reconnection Logic
 * Attempts to reconnect to broker if signal is lost.
 */
void reconnect() {
    if (!client.connected()) {
        Serial.print("MQTT Disconnected. Trying to connect with ID: "); Serial.print(deviceID); Serial.print("... ");
        // Attempt connection using the unique device ID
        if (client.connect(deviceID.c_str())) {
            Serial.println("CONNECTED!");
            // Resubscribe to topic
            client.subscribe(mqtt_topic_sub);
            
            // Publish status immediately on connect
            if(sdAvailable) {
                // Publish file list automatically so dashboard updates
                publishFileListMQTT();
            } else {
                client.publish(mqtt_topic_pub, "ERROR: NO SD CARD");
            }
            
            flashLED(3);
        } else {
            Serial.print("Failed rc="); Serial.println(client.state());
        }
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, HIGH);
    Serial.begin(115200);
    
    WiFi.setSleep(false);

    // wait 2s.
    // This allows the device to boot if plugged into a wall/battery.
    unsigned long serialStart = millis();
    while(!Serial && (millis() - serialStart < 2000)) { 
        delay(10); 
    }

    deviceID = "Titan-" + String(random(0xffff), HEX);

    Serial.println("\n\n=== SBS HAPTIC ACTUATOR START ===");
    
    // --- 2. SD CARD MOUNTING ---
    // We try 3 times to mount the SD card before proceeding.
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH); // Ensure CS is high (deselected) to reset state
    
    int sdRetries = 0;
    sdAvailable = false;
    
    while(sdRetries < 3) {
        Serial.print("Mounting SD (Attempt "); Serial.print(sdRetries+1); Serial.print(")... ");
        
        // Try mounting at 4MHz (safer speed for older cards)
        if (SD.begin(SD_CS, SPI, 4000000)) {
            Serial.println("SUCCESS.");
            sdAvailable = true;
            break; // Exit loop on success
        } else {
            Serial.println("FAILED.");
            sdRetries++;
            digitalWrite(SD_CS, HIGH); // Reset pin
            delay(100); // Wait a bit before retrying
        }
    }

    if (!sdAvailable) {
        Serial.println("WARNING: Could not mount SD Card. Proceeding without SD Card");
        // Flash LED to warn user locally
        flashLED(10);
    }
    
    setupI2S();
    Serial.println("I2S Driver - OK.");

    if (sdAvailable) {
        File root = SD.open("/");
        listWavFilesSerial(root);
        root.close();
    }

    // 3. MODE SELECTION
    Serial.println("\n\n*** PRESS ANY KEY NOW FOR MANUAL MODE ***");
    Serial.println("\n... Waiting 3 seconds ...\n");
    
    unsigned long startWait = millis();
    bool userInterrupted = false;
    
    while (millis() - startWait < 3000) {
        if (Serial.available()) {
            while(Serial.available()) Serial.read(); // Clear buffer
            userInterrupted = true;
            break;
        }
        delay(10);
    }

    if (userInterrupted) {
        manualMode = true;
        flashLED(5);
        Serial.println("\n#####################################");
        Serial.println("#   MANUAL MODE ACTIVE                #");
        if(!sdAvailable) Serial.println("#   WARNING: SD CARD FAILED/MISSING   #");
        Serial.println("#   - 'filename.wav' to play          #");
        Serial.println("#   - 'loop:filename.wav' to loop     #");
        Serial.println("#   - 'vol:50' to set volume          #");
        Serial.println("#   - 'stop' to stop                  #");
        Serial.println("#   - 'reset' to reboot               #");
        Serial.println("#######################################\n");
    } else {
        // 4. WiFi Setup
        Serial.println(">> No input detected. Starting WiFi Mode...");
        manualMode = false;
        
        Serial.print("Connecting to "); Serial.println(ssid);
        WiFi.begin(ssid, password);
        
        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 20) {
            delay(500); Serial.print("."); retries++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Connected.");
            Serial.print("IP: "); Serial.println(WiFi.localIP());
            Serial.print("MAC Address: ");
            Serial.println(WiFi.macAddress());
            Serial.println("(Type 'menu' or 'reset' here to reboot into Manual Mode)");
            client.setServer(mqtt_server, 1883);
            client.setCallback(callback);
            client.setKeepAlive(15); 
            client.setSocketTimeout(15);
        } else {
            Serial.println("\nWiFi Failed. Switching to manual mode.");
            manualMode = true;
        }
    }
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
    // --- 1. CONTROL LOGIC ---
    if (manualMode) {
        // Only listen to USB Serial
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if(cmd == "reset" || cmd == "reboot") ESP.restart(); // Add soft reset here too
            parseCommand(cmd);
        }
    } else {
        // --- 2. WIFI MODE ---
        if (!client.connected()) {
            static unsigned long lastReconnect = 0;
            if (millis() - lastReconnect > 5000) {
                reconnect();
                lastReconnect = millis();
            }
        } else {
            client.loop();
        }

        // --- HOT-PLUG LISTENER ---
        // Even in WiFi mode, listen for a user plugging in USB.
        // If they type "menu" or "reset", we reboot the board.
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "menu" || input == "reset") {
                Serial.println("\nCommand received. Rebooting to Diagnostics/Manual Menu...");
                delay(500);
                ESP.restart(); // Soft Restart
            }
        }
    }

    // --- 2. AUDIO PLAYBACK LOGIC ---
    // Only run if playing AND SD is confirmed available
    if (isPlaying && sdAvailable) {
        if (audioFile.available()) {
            size_t bytes_read = audioFile.read(i2s_buffer, sizeof(i2s_buffer));
            
            int16_t* samples = (int16_t*)i2s_buffer;
            int sampleCount = bytes_read / 2; 

            for (int i = 0; i < sampleCount; i++) {
                samples[i] = (int16_t)(samples[i] * masterVolume);
            }

            size_t bytes_written;
            i2s_write(I2S_NUM_0, i2s_buffer, bytes_read, &bytes_written, 0); 
            
        } else {
            if (shouldLoop) {
                audioFile.seek(44); 
            } else {
                stopPlayback();
            }
        }
    } else {
        delay(10); 
    }
}
