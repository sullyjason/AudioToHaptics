#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/i2s.h"

// --- USER CONFIGURATION ---
const char* ssid        = "Sully";
const char* password    = "1234567890";
const char* mqtt_server = "broker.hivemq.com"; 
const char* mqtt_topic  = "SBShaptics/1";

// --- PINS ---
#define SD_CS      D2
#define I2S_BCLK   1
#define I2S_LRC    2
#define I2S_DOUT   6
#define STATUS_LED LED_BUILTIN 

#define SAMPLE_RATE 44100 

WiFiClient espClient;
PubSubClient client(espClient);
File audioFile;

// --- PLAYBACK STATE ---
bool isPlaying = false;
bool shouldLoop = false; 
uint8_t i2s_buffer[1024];
String deviceID; 

void flashLED(int times) {
    for(int i=0; i<times; i++) {
        digitalWrite(STATUS_LED, LOW); 
        delay(100);
        digitalWrite(STATUS_LED, HIGH);
        delay(100);
    }
}

// --- UPDATED FILE LISTER ---
void listWavFiles(File dir) {
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break; 
        
        if (!entry.isDirectory()) {
            String fname = String(entry.name());
            
            // --- NEW FILTER: Ignore macOS '._' files ---
            if (!fname.startsWith("._")) {
                
                String upperName = fname;
                upperName.toUpperCase();
                
                if (upperName.endsWith(".WAV")) {
                    Serial.print("\t");
                    Serial.print(fname);
                    Serial.print("\t\t");
                    Serial.println(entry.size());
                }
            }
        }
        entry.close();
    }
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
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void startPlayback(const char* filename, bool loopMode) {
    if (isPlaying) audioFile.close();
    
    if (String(filename) == "/stop.wav") {
        isPlaying = false;
        shouldLoop = false;
        Serial.println("Playback STOPPED by user.");
        return;
    }

    if (!SD.exists(filename)) {
        Serial.print("ERROR: File not found -> "); Serial.println(filename);
        flashLED(5); 
        return;
    }

    audioFile = SD.open(filename);
    audioFile.seek(44); 
    
    isPlaying = true;
    shouldLoop = loopMode; 
    
    Serial.print(shouldLoop ? "LOOPING: " : "PLAYING: "); 
    Serial.println(filename);
    flashLED(1);
}

void stopPlayback() {
    if (isPlaying) {
        size_t bytes_written;
        memset(i2s_buffer, 0, sizeof(i2s_buffer));
        i2s_write(I2S_NUM_0, i2s_buffer, sizeof(i2s_buffer), &bytes_written, portMAX_DELAY);
        i2s_zero_dma_buffer(I2S_NUM_0);
        audioFile.close();
        isPlaying = false;
        shouldLoop = false; 
        Serial.println("STOPPED playback (End of File).");
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    
    String message = String(msg);
    String cleanName;
    bool requestLoop = false;

    if (message.startsWith("loop:")) {
        requestLoop = true;
        cleanName = message.substring(5); 
    } else {
        requestLoop = false;
        cleanName = message;
    }
    
    Serial.print("MQTT CMD: "); Serial.println(message);
    
    String filename = "/" + cleanName + ".wav";
    startPlayback(filename.c_str(), requestLoop);
}

void reconnect() {
    if (!client.connected()) {
        Serial.print("MQTT Disconnected. Trying ID: "); Serial.print(deviceID); Serial.print("... ");
        if (client.connect(deviceID.c_str())) {
            Serial.println("CONNECTED!");
            client.subscribe(mqtt_topic);
            flashLED(3);
        } else {
            Serial.print("Failed rc="); Serial.println(client.state());
        }
    }
}

void setup() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    Serial.begin(115200);
    while(!Serial) {
        digitalWrite(STATUS_LED, LOW); delay(50); digitalWrite(STATUS_LED, HIGH); delay(50);
    }
    
    deviceID = "Titan-" + String(random(0xffff), HEX);

    Serial.println("\n\n========================================");
    Serial.println("       TITAN DIAGNOSTIC START");
    Serial.println("========================================");
    
    Serial.println("--- CONFIGURATION ---");
    Serial.print("Device Name:   "); Serial.println(deviceID);
    Serial.print("WiFi SSID:     "); Serial.println(ssid);
    Serial.print("MQTT Server:   "); Serial.println(mqtt_server);
    Serial.println("---------------------");

    pinMode(SD_CS, OUTPUT);
    Serial.print("Mounting SD Card... ");
    if (!SD.begin(SD_CS)) {
        Serial.println("FAILED!");
        while(1) { flashLED(1); delay(100); }
    }
    Serial.println("OK");
    
    Serial.println("--- WAV FILES (ROOT) ---");
    File root = SD.open("/");
    listWavFiles(root);
    Serial.println("------------------------");

    setupI2S();
    Serial.println("I2S Driver OK");

    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500); Serial.print("."); retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK!");
    } else {
        Serial.println("\nCRITICAL ERROR: WiFi Failed!");
    }

    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    client.setKeepAlive(60); 
    client.setSocketTimeout(60);
    
    Serial.println("MQTT Configured. Ready.");
    Serial.println("========================================");
}

void loop() {
    if (!client.connected()) {
        static unsigned long lastReconnect = 0;
        if (millis() - lastReconnect > 5000) {
            reconnect();
            lastReconnect = millis();
        }
    } else {
        client.loop();
    }

    if (isPlaying) {
        if (audioFile.available()) {
            size_t bytes_read = audioFile.read(i2s_buffer, sizeof(i2s_buffer));
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
