/*
SBS Haptic Actuator Firmware 
v0.1.0

Compatible with Actuator_Dashboard.html v0.1.0

Written for Seeed Studios Xiao ESP32S3 and Adafruit Audio BFF
https://learn.adafruit.com/adafruit-audio-bff
See hardware folder for details

Capabilities:
 * 1. Hybrid Control: Works over WiFi (MQTT) OR USB (Serial).
 * 2. Audio Playback: Plays .wav files from SD card via I2S.
 * 3. Features: Volume control, Looping, Remote file listing.

-----
Created in 2025 by Silvan Jason Roth for Dr Jacqueline Borgstedt, SBS Lab, ETH Zürich

sbs.ethz.ch/people/postdocs/jacqueline-borgstedt.html
silvanjason.me/

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
const char* mqtt_topic_sub  = "SBShaptics/2";       
const char* mqtt_topic_pub  = "SBShaptics/2/files"; 

// --- PIN DEFINITION ---
// Specific to Adafruit SD Card/Amp "BFF", see Hardware folder for details. Change if using other hardware
// SD Card Pins (Standard SPI)
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
 * reads SD card, creates a CSV string (file1.wav,file2.wav), and sends it to broker.
 */
void publishFileListMQTT() {
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
        
        audioFile.close();
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
    // Playing them sounds like a loud static "pop".
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
        sendFileListSerial(); // Always send to Serial (Web interface needs this even in hybrid mode)
        if(!manualMode) {
             publishFileListMQTT(); // Only publish to MQTT if we are online
        }
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
            flashLED(3);
        } else {
            Serial.print("Failed rc="); Serial.println(client.state());
        }
    }
}

/**
 * Serial Input Handler
 * Checks if data is coming via USB cable.
 */
void checkSerialCommand() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n'); // Read until newline
        parseCommand(cmd); // Pass to the common parser
    }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    // Basic init
    pinMode(STATUS_LED, OUTPUT); 
    digitalWrite(STATUS_LED, HIGH); // On
    Serial.begin(115200);
    
    // Ensure WiFi radio doesn't sleep (improves responsiveness)
    WiFi.setSleep(false);

    // Wait for Serial Monitor to be ready (prevents missed messages on boot)
    while(!Serial) { delay(10); }

    // Generate a random ID for MQTT (prevents conflicts if you have multiple devices)
    deviceID = "Titan-" + String(random(0xffff), HEX);

    Serial.println("\n\n=== SBS HAPTIC ACTUATOR START ===");
    
    // 1. Setup Hardware (Common to both modes)
    pinMode(SD_CS, OUTPUT);
    if (!SD.begin(SD_CS)) {
        Serial.println("SD MOUNT FAILED! Halting.");
        // Fatal error: Trap in infinite loop
        while(1) { flashLED(1); delay(100); }
    }
    Serial.println("SD Mounted - OK.");
    
    setupI2S();
    Serial.println("I2S Driver - OK.");

    // List files immediately for diagnostics
    File root = SD.open("/");
    listWavFilesSerial(root);
    root.close();

    // 2. MODE SELECTION (The "Hybrid" Logic)
    Serial.println("\n\n*** PRESS ANY KEY NOW FOR MANUAL MODE ***");
    Serial.println("\n... Waiting 3 seconds ...\n");
    
    unsigned long startWait = millis();
    // Loop for 3 seconds listening for ANY keypress on Serial
    while (millis() - startWait < 3000) {
        if (Serial.available()) {
            // If user typed something, clear the buffer
            while(Serial.available()) Serial.read(); 
            // Activate Manual Mode
            manualMode = true;
            flashLED(5); // Acknowledge with blinks
            break;
        }
        delay(10);
    }

    if (manualMode) {
        Serial.println("\n#######################################");
        Serial.println("#                                     #");
        Serial.println("#   MANUAL MODE ACTIVE                #");
        Serial.println("#                                     #");
        Serial.println("#   - Type 'filename' to play         #");
        Serial.println("#   - 'loop:filename' to loop         #");
        Serial.println("#   - 'vol:50' to set volume to 50%.  #");
        Serial.println("#   - 'stop' to control.              #");
        Serial.println("#                                     #");  
        Serial.println("#######################################\n");
    }else {
        // --- WIFI MODE (MQTT) ---
        Serial.println(">> Continuing to WiFi Mode...");
        Serial.print("Connecting to "); Serial.println(ssid);
        WiFi.begin(ssid, password);
        
        int retries = 0;
        // Wait for WiFi connection (up to 10 seconds)
        while (WiFi.status() != WL_CONNECTED && retries < 20) {
            delay(500); Serial.print("."); retries++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(" WiFi Connected.");
            Serial.print("IP: "); Serial.println(WiFi.localIP());
            
            // Configure MQTT
            client.setServer(mqtt_server, 1883);
            client.setCallback(callback); // Register the callback function
            client.setKeepAlive(15); 
            client.setSocketTimeout(15);
        } else {
            // Failover: If WiFi fails, default back to Manual Mode
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
        checkSerialCommand();
    } else {
        // WiFi Mode: Maintain MQTT connection
        if (!client.connected()) {
            static unsigned long lastReconnect = 0;
            // Non-blocking reconnect timer (every 5 seconds)
            if (millis() - lastReconnect > 5000) {
                reconnect();
                lastReconnect = millis();
            }
        } else {
            // Process incoming MQTT messages
            client.loop();
        }
    }

    // --- 2. AUDIO PLAYBACK LOGIC ---
    if (isPlaying) {
        if (audioFile.available()) {
            // Step A: Read raw bytes from SD card into buffer
            size_t bytes_read = audioFile.read(i2s_buffer, sizeof(i2s_buffer));
            
            // Step B: SOFTWARE VOLUME CONTROL
            // To change volume, we must multiply every sample by a float (0.0 to 1.0).
            // We cast the buffer to int16_t because audio samples are 16-bit signed integers.
            int16_t* samples = (int16_t*)i2s_buffer;
            int sampleCount = bytes_read / 2; // 2 bytes per sample (16-bit)

            for (int i = 0; i < sampleCount; i++) {
                // Apply volume scaling
                samples[i] = (int16_t)(samples[i] * masterVolume);
            }

            // Step C: Send processed audio to I2S hardware
            size_t bytes_written;
            i2s_write(I2S_NUM_0, i2s_buffer, bytes_read, &bytes_written, 0); 
            
        } else {
            // End of file reached
            if (shouldLoop) {
                // If looping, jump back to start (skipping header again)
                audioFile.seek(44); 
            } else {
                // Otherwise, stop cleanly
                stopPlayback();
            }
        }
    } else {
        // If not playing, small delay to prevent CPU hogging
        delay(10); 
    }
}