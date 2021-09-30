#include <M5StickC.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "sensor_config.h"
#include "wifi_config.h"
#include "logo.h"
// Replace with your network credentials
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

#if ENABLE_AUDIO
#include <driver/i2s.h>
/* ----- i2s hardware constants ----- */
const i2s_port_t kI2S_Port = I2S_NUM_0;
const int kI2S_PinClk = 0;
const int kI2S_PinData = 34;
/* ----- i2s constants ----- */
const i2s_bits_per_sample_t audio_bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
const uint8_t kI2S_BytesPerSample = audio_bits_per_sample / 8;
const uint16_t kI2S_ReadSizeBytes = SAMPLES_PER_PACKET * kI2S_BytesPerSample;
const uint16_t kI2S_BufferSizeSamples = 1024;
const uint16_t kI2S_BufferSizeBytes = kI2S_BufferSizeSamples * kI2S_BytesPerSample;
const uint16_t kI2S_BufferCount = (3 * SAMPLES_PER_PACKET) / (2 * kI2S_BufferSizeSamples);
const int kI2S_QueueLength = 16;
/* ----- i2s variables ----- */
int16_t micReadBuffer_[SAMPLES_PER_PACKET] = {0};

QueueHandle_t pI2S_Queue_ = nullptr;
#endif //ENABLE_AUDIO

WiFiServer server(WIFI_SERVER_PORT);

DynamicJsonDocument config_message(WRITE_BUFFER_SIZE);
static char         config_output_str[WRITE_BUFFER_SIZE];

// Variable to store the HTTP request
String  header;
int     bytesRead = 0;
int16_t timer;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 20000;


/* IMU Variables */
int16_t accX = 0;
int16_t accY = 0;
int16_t accZ = 0;
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

int16_t pData[PACKET_SIZE];
#if USE_SAWTOOTH_PATTERN
static void get_sawtooth(int16_t* ax, int16_t* ay, int16_t* az, int16_t sign)
{
    *ax = sign * timer;
    *ay = sign * (timer + (int16_t) 10);
    *az = sign * (timer + (int16_t) 20);
    timer++;

    if (sign == 1 && timer > 500)
    {
        timer = 0;
    }
}
#endif

static size_t get_audio_data()
{
    esp_err_t i2sErr = ESP_OK;

    size_t i2sBytesRead = 0;

    // Store time stamp for debug output
    unsigned long timeBeforeReadMicros = micros();

    // Note: If the I2S DMA buffer is empty, 'i2s_read' blocks the current thread until data becomes available
    i2sErr = i2s_read(kI2S_Port, micReadBuffer_, kI2S_ReadSizeBytes, &i2sBytesRead, 100 / portTICK_PERIOD_MS);
    // Check i2s error state after reading
    if (i2sErr)
    {
        log_e("i2s_read failure. ESP error: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
    }

    return i2sBytesRead;
}

bool setupI2Smic()
{
    esp_err_t i2sErr;

    // i2s configuration for sampling 16 bit mono audio data
    //
    // Notes related to i2s.c:
    // - 'dma_buf_len', i.e. the number of samples in each DMA buffer, is limited to 1024
    // - 'dma_buf_len' * 'bytes_per_sample' is limted to 4092
    // - 'I2S_CHANNEL_FMT_ONLY_RIGHT' means "mono", i.e. only one channel to be received via i2s
    //   In the M5StickC microphone example 'I2S_CHANNEL_FMT_ALL_RIGHT' is used which means two channels.
    //   Afterwards, i2s_set_clk is called to change the DMA configuration to just one channel.
    //
    i2s_config_t i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate = SENSOR_SAMPLE_RATE,
        .bits_per_sample = audio_bits_per_sample,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = kI2S_BufferCount,
        .dma_buf_len = kI2S_BufferSizeSamples
    };

    i2sErr = i2s_driver_install(kI2S_Port, &i2sConfig, kI2S_QueueLength, &pI2S_Queue_);

    if (i2sErr)
    {
        Serial.println("Failed to start i2s driver. ESP error");//: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
        return false;
    }

    if (pI2S_Queue_ == nullptr)
    {
        Serial.println("Failed to setup i2s event queue.");
        return false;
    }

    // Configure i2s pins for sampling audio data from the built-in microphone of the M5StickC
    i2s_pin_config_t i2sPinConfig = {
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = kI2S_PinClk,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = kI2S_PinData
    };

    i2sErr = i2s_set_pin(kI2S_Port, &i2sPinConfig);

    if (i2sErr)
    {
        log_e("Failed to set i2s pins. ESP error: %s (%x)", esp_err_to_name(i2sErr), i2sErr);
        return false;
    }

    return true;
}

static void clear_screen(void)
{
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
}

static void set_stream_idle_lcd(void)
{
    clear_screen();
    M5.Lcd.println(WiFi.localIP());
    M5.Lcd.println("");
    M5.Lcd.println("Stream Idle");
}

static void set_stream_active_lcd(void)
{
    clear_screen();
    M5.Lcd.println(WiFi.localIP());
    M5.Lcd.println("");
    M5.Lcd.println("Streaming");
}

static void build_config_message()
{
    int column_index                     = 0;
    config_message["sample_rate"]        = SENSOR_SAMPLE_RATE;
    config_message["samples_per_packet"] = SAMPLES_PER_PACKET;
#if ENABLE_ACCEL
    config_message["column_location"]["AccelerometerX"] = column_index++;
    config_message["column_location"]["AccelerometerY"] = column_index++;
    config_message["column_location"]["AccelerometerZ"] = column_index++;
#endif
#if ENABLE_GYRO
    config_message["column_location"]["GyroscopeX"] = column_index++;
    config_message["column_location"]["GyroscopeY"] = column_index++;
    config_message["column_location"]["GyroscopeZ"] = column_index++;
#endif
#if ENABLE_AUDIO
    config_message["column_location"]["Microphone"] = column_index++;
#endif
}


void setup()
{
    // put your setup code here, to run once:
    M5.begin();
#if ENABLE_ACCEL || ENABLE_GYRO
    M5.IMU.Init();
#endif
#if ENABLE_AUDIO
    setupI2Smic();
#endif
    M5.Lcd.setRotation(3);
    clear_screen();
    M5.Lcd.drawBitmap(6,5,sensiml_logoWidth, sensiml_logoHeight, sensiml_logo);
    M5.Lcd.setTextSize(2);
    Serial.begin(115200);
    build_config_message();
    serializeJson(config_message, config_output_str, WRITE_BUFFER_SIZE);

    // Connect to Wi-Fi network with SSID and password
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
    set_stream_idle_lcd();
    server.begin();
}

void get_imu_data(int16_t* pData, int* pDataIndex)
{
#if USE_SAWTOOTH_PATTERN
    get_sawtooth(&gyroX, &gyroY, &gyroZ, (int16_t) -1);
    get_sawtooth(&accX, &accY, &accZ, (int16_t) 1);
    return;
#endif

#if ENABLE_ACCEL
    M5.IMU.getAccelAdc(&accX, &accY, &accZ);
#endif
#if ENABLE_GYRO
    M5.IMU.getGyroAdc(&gyroX, &gyroY, &gyroZ);
#endif
}

void loop()
{
    WiFiClient client = server.available();  // Listen for incoming clients

    if (client)
    {  // If a new client connects,


        currentTime  = millis();
        previousTime = currentTime;
        Serial.println("New Client.");  // print a message out in the serial port
        String currentLine = "";        // make a String to hold incoming data from the client
        while (client.connected() && currentTime - previousTime <= timeoutTime)
        {  // loop while the client's connected
            currentTime = millis();
            if (client.available())
            {                            // if there's bytes to read from the client,
                char c = client.read();  // read a byte, then
                Serial.write(c);         // print it out the serial monitor
                header += c;
                if (c == '\n')
                {  // if the byte is a newline character
                    // if the current line is blank, you got two newline characters in a row.
                    // that's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0)
                    {
                        // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                        // and a content-type so the client knows what's coming, then a blank line:
                        if (header.indexOf("GET /config") >= 0)
                        {
                            client.println("HTTP/1.1 200 OK");
                            client.println("Content-type:text/json");
                            client.println();
                            client.println(config_output_str);

                            // Break out of the while loop
                            break;
                        }
                        else if (header.indexOf("GET /stream") >= 0)
                        {
                            set_stream_active_lcd();
                            client.println("HTTP/1.1 200 OK");
                            client.println("Content-type:application/octet-stream");
                            client.println();
                            int pIndex = 0;
                            while (client.connected())
                            {
#if (ENABLE_ACCEL || ENABLE_GYRO)
                                get_imu_data(pData, &pIndex);
                                delay(5);

                                pData[pIndex++] = accX;
                                pData[pIndex++] = accY;
                                pData[pIndex++] = accZ;
                                pData[pIndex++] = gyroX;
                                pData[pIndex++] = gyroY;
                                pData[pIndex++] = gyroZ;

                                if (pIndex == PACKET_SIZE)
                                {
                                    bytesRead = client.write((const char*) pData, PACKET_SIZE * 2);
                                    // Serial.println(bytesRead);
                                    pIndex = 0;
                                }
#elif ENABLE_AUDIO
                                get_audio_data();
                                client.write((uint8_t*)micReadBuffer_, kI2S_ReadSizeBytes/2);
#endif

                                
                            }
                            set_stream_idle_lcd();
                        }

                        // The HTTP response ends with another blank line
                        client.println();
                    }
                    else
                    {  // if you got a newline, then clear currentLine
                        currentLine = "";
                    }
                }
                else if (c != '\r')
                {                      // if you got anything else but a carriage return character,
                    currentLine += c;  // add it to the end of the currentLine
                }
            }
        }
        // Clear the header variable
        header = "";
        // Close the connection
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
    
}
