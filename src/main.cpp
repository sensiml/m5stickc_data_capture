#include <M5StickC.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "sensor_config.h"
#include "wifi_config.h"
#include "logo.h"
// Replace with your network credentials
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

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

static void clear_screen(void)
{
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
}

static void set_stream_idle_lcd(void)
{
    clear_screen();
    M5.Lcd.println(WiFi.localIP());
    M5.Lcd.println("Stream Idle");
}

static void set_stream_active_lcd(void)
{
    clear_screen();
    M5.Lcd.println(WiFi.localIP());
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
    M5.IMU.Init();
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
#elif ENABLE_AUDIO
                                get_audio_data();
#endif

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
    else
    {
    }
}
