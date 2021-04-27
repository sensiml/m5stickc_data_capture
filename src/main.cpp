#include "wifi_config.h"
#include "sensor_config.h"
#include <M5StickC.h>
#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

const char* json_config = "{"
                          "\"sample_rate\":200,"
                          "\"samples_per_packet\":10,"
                          "\"column_location\":{"
                          "  \"AccelerometerX\":0,"
                          "  \"AccelerometerY\":1,"
                          "  \"AccelerometerZ\":2,"
                          "  \"GyroscopeX\":3,"
                          "  \"GyroscopeY\":4,"
                          "  \"GyroscopeZ\":5"
                          "}"
                          "}\r\n";

// Set web server port number to 80
WiFiServer server(80);

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

void setup()
{
    // put your setup code here, to run once:
    M5.begin();
    M5.IMU.Init();
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(0, 0);

    Serial.begin(115200);

    // Connect to Wi-Fi network with SSID and password
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    M5.Lcd.println("Attempting WiFi Connection");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        M5.Lcd.print(".");
    }
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    // Print local IP address and start web server
    M5.Lcd.println(WiFi.localIP());
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
                            client.println(json_config);

                            // Break out of the while loop
                            break;
                        }
                        else if (header.indexOf("GET /stream") >= 0)
                        {
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
        // M5.Lcd.fillScreen(BLACK);
        // M5.Lcd.setCursor(0, 0);
        // // Print local IP address and start web server
        // M5.Lcd.println(WiFi.localIP());
        // M5.Lcd.println("Disconnected");
        Serial.println("Client disconnected.");
        Serial.println("");
    }
}
