#include <SPI.h>
#include <Arduino.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ToshibaDaiseikaiHeatpumpIR.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include "DHT.h"

// #define BMP280_ADDRESS (0x76)
#include <Adafruit_BMP280.h>

uint8_t mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 6);
IPAddress myDns(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress mqttServer(192, 168, 1, 3);
unsigned int mqttPort = 1883;

#define DEVICE_ALIAS "my-unoEthernet"
#define DEVICE_LENGTH 15
#define IR_SENSOR_PIN 9
#define LIGHT_SENSOR_PIN A5
#define DHT_PIN 2

const int relays[2] = {
    7, 8};

#define DHT_TYPE DHT11

const char *SECRET = "FSTOP_DEVICE";

const char *DEVICE_ID = DEVICE_ALIAS;
const char *SUB_TOPIC = DEVICE_ALIAS "/#";
const char *PUB_STATUS_TOPIC = "status/" DEVICE_ALIAS;
const char *PUB_DEBUG_TOPIC = "device/" DEVICE_ALIAS "/debug";
const char *PUB_TEMPERATURE_TOPIC = "device/" DEVICE_ALIAS "/temperature";
const char *PUB_HUMIDUTY_TOPIC = "device/" DEVICE_ALIAS "/humiduty";
const char *PUB_AMBIENT_TOPIC = "device/" DEVICE_ALIAS "/ambient";
const char *PUB_FLOAT_TOPIC = "device/" DEVICE_ALIAS "/float";
const char *PUB_RELAY0_TOPIC = "device/" DEVICE_ALIAS "/relay0";
const char *PUB_RELAY1_TOPIC = "device/" DEVICE_ALIAS "/relay1";

const char *PUB_BMP_TEMP_TOPIC = "device/" DEVICE_ALIAS "/bmpTemperature";
const char *PUB_PRESSURE_TOPIC = "device/" DEVICE_ALIAS "/pressure";

const char *IR_CMD = "IR_CMD";
const char *RELAY0_CMD = "RELAY0_CMD";
const char *RELAY1_CMD = "RELAY1_CMD";

// const float divider = 5.0f / 1024.0f;
const unsigned int UDP_PORT = 37020;

void callback(char *topic, uint8_t *payload, unsigned int length);

EthernetClient ethClient;
EthernetUDP Udp;
PubSubClient mqttClient(mqttServer, mqttPort, callback, ethClient);
IRSenderPWM irSender(IR_SENSOR_PIN);
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_BMP280 bmp;

ToshibaDaiseikaiHeatpumpIR *heatpumpIR;

const size_t capacity = JSON_OBJECT_SIZE(3) + 60;
DynamicJsonDocument doc(capacity);

void callback(char *topic, uint8_t *payload, unsigned int length)
{
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.

  // Allocate the correct amount of memory for the payload copy
  uint8_t *p = (uint8_t *)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p, payload, length);

  // Serial.print(topic + DEVICE_LENGTH);
  // Serial.print(">");
  // Serial.write(p, length);
  // Serial.println();

  if (memcmp(topic + DEVICE_LENGTH, IR_CMD, 6) == 0)
  {
    Serial.print(F("Recieved: "));
    Serial.println(IR_CMD);
    irCommand(p, length);
  }
  else if (memcmp(topic + DEVICE_LENGTH, RELAY0_CMD, 10) == 0)
  {
    Serial.print(F("Recieved: "));
    Serial.println(RELAY0_CMD);

    relayCommand(0, (bool)payload);
    mqttClient.publish(PUB_RELAY0_TOPIC, p, 4, true);
  }
  else if (memcmp(topic + DEVICE_LENGTH, RELAY1_CMD, 10) == 0)
  {
    Serial.print(F("Recieved: "));
    Serial.println(RELAY1_CMD);

    relayCommand(1, (bool)payload);
    mqttClient.publish(PUB_RELAY1_TOPIC, p, 4, true);
  }
  else
  {
    // Serial.print(topic);
    // Serial.print(">");
    // Serial.write(p, length);
    // Serial.println();
    mqttClient.publish(PUB_DEBUG_TOPIC, p, length);
  }

  // Free the memory
  free(p);
}

void irCommand(uint8_t *payload, unsigned int length)
{
  Serial.print(F("payload: "));
  Serial.write(payload, length);
  Serial.println();

  deserializeJson(doc, payload);

  uint8_t power = doc["power"];
  uint8_t mode = doc["mode"];
  uint8_t fan = doc["fan"];
  uint8_t temp = doc["temp"];
  uint8_t h_swing = doc["h_swing"];
  uint8_t v_swing = doc["v_swing"];
  bool turbo = doc["turbo"];
  // Serial.print("power: ");
  // Serial.println(power);
  // Serial.print(" mode: ");
  // Serial.println(mode);
  // Serial.print(" fan: ");
  // Serial.println(fan);
  // Serial.print(" temp: ");
  // Serial.println(temp);
  // Serial.print(" h_swing: ");
  // Serial.println(h_swing);
  // Serial.print(" v_swing: ");
  // Serial.println(v_swing);
  // Serial.print(" turbo: ");
  // Serial.println(turbo);
  heatpumpIR->send(irSender, power, mode, fan, temp, v_swing, h_swing);
}

void relayCommand(uint8_t no, bool status)
{
  digitalWrite(relays[no], status);
}

unsigned long lastReadDHT11;
float curTemperature;
float curHumiduty;
float lastTemperature;
float lastHumiduty;
void dhtLoop()
{
  if (millis() - lastReadDHT11 > 2000)
  {
    lastReadDHT11 = millis();

    curTemperature = dht.readTemperature();
    curHumiduty = dht.readHumidity();

    if (isnan(curTemperature) || isnan(curHumiduty))
    {
      // Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    if (lastTemperature != curTemperature)
    {
      lastTemperature = curTemperature;
      // sprintf_P(temperatureStr, (PGM_P)F("%05d"), curTemperature);
      mqttClient.publish(PUB_TEMPERATURE_TOPIC, (uint8_t *)&curTemperature, 4, true);
    }

    if (lastHumiduty != curHumiduty)
    {
      lastHumiduty = curHumiduty;
      mqttClient.publish(PUB_HUMIDUTY_TOPIC, (uint8_t *)&curHumiduty, 4, true);
    }
  }
}

unsigned long lastReadAmbient = 0;
int curAmbient;
int lastAmbient;
void ambientLoop()
{
  if (millis() - lastReadAmbient > 1000)
  {
    lastReadAmbient = millis();
    curAmbient = analogRead(LIGHT_SENSOR_PIN);
    if (lastAmbient != curAmbient)
    {
      lastAmbient = curAmbient;
      float square_ratio = pow(curAmbient / 1023.0, 2.0);
      mqttClient.publish(PUB_AMBIENT_TOPIC, (uint8_t *)&square_ratio, 4, true);
    }
  }
}

// unsigned long lastReadFloat;
// float curFloat;
// int curVal;
// int lastVal;
// void testFloatLoop()
// {
//   if (millis() - lastReadFloat > 1000)
//   {
//     lastReadFloat = millis();

//     curVal = analogRead(A0);

//     if (lastVal != curVal)
//     {
//       lastVal = curVal;

//       curFloat = curVal * divider;

//       mqttClient.publish(PUB_FLOAT_TOPIC, (uint8_t *)&curFloat, 4, true);
//     }
//   }
// }

bool udpOk = true;
char packetBuffer[100];              // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged"; // a string to send back
void udpLoop()
{
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print(F("Received packet size: "));
    Serial.println(packetSize);
    Serial.print(F("From "));
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++)
    {
      Serial.print(remote[i], DEC);
      if (i < 3)
      {
        Serial.print(F("."));
      }
    }
    Serial.print(F(":"));
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, packetSize);
    // Serial.println(F("Contents:"));
    // Serial.println(packetBuffer);

    deserializeJson(doc, packetBuffer);

    const char *secret = doc["secret"];

    Serial.print(F("Secret: "));
    Serial.println(secret);
    if (strcmp(secret, SECRET) == 0)
    {
      Serial.print(F("mqtt host: "));
      Serial.print((char *)doc["mqtthost"]);
      Serial.print(F(":"));
      Serial.println((unsigned int)doc["mqttport"]);

      mqttServer.fromString((char *)doc["mqtthost"]);
      mqttPort = (unsigned int)doc["mqttport"];

      Udp.stop();
      udpOk = true;
    }

    // send a reply to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
  }
}

unsigned long lastReadbmp280 = 0;
float curbmp280Temp;
float lastbmp280Temp;
float curbmp280pressure;
float lastbmp280pressure;
void bmp280Loop()
{
  if (millis() - lastReadbmp280 > 10000)
  {
    lastReadbmp280 = millis();

    curbmp280Temp = bmp.readTemperature();
    curbmp280pressure = bmp.readPressure();

    if (lastbmp280Temp != curbmp280Temp)
    {
      lastbmp280Temp = curbmp280Temp;
      mqttClient.publish(PUB_BMP_TEMP_TOPIC, (uint8_t *)&curbmp280Temp, 4, true);
    }
    if (lastbmp280pressure != curbmp280pressure)
    {
      lastbmp280pressure = curbmp280pressure;
      mqttClient.publish(PUB_PRESSURE_TOPIC, (uint8_t *)&curbmp280pressure, 4, true);
    }
  }
}

long lastReconnectAttempt = 0;
boolean reconnect()
{
  if (mqttClient.connect(DEVICE_ID, PUB_STATUS_TOPIC, 0, true, "disconnected"))
  {
    mqttClient.publish_P(PUB_STATUS_TOPIC, (PGM_P)F("connected"), true);
    mqttClient.subscribe(SUB_TOPIC);
  }
  return mqttClient.connected();
}

void setup()
{
  for (int i = 0; i < 2; i++)
  {
    pinMode(relays[i], OUTPUT);
  }

  Serial.begin(115200);

  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. :("));
    while (true)
    {
      delay(100); // do nothing, no point running without Ethernet hardware
    }
  }

  delay(1500);
  lastReconnectAttempt = 0;
  heatpumpIR = new ToshibaDaiseikaiHeatpumpIR();

  if (!bmp.begin(0x76))
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    // while (1);
  }
  else
  {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }

  Serial.print(F("My IP address: "));
  Serial.println(Ethernet.localIP());

  Udp.begin(UDP_PORT);
  dht.begin();

  if (!udpOk)
  {
    Serial.println(F("Waiting for server broadcast mqtt host..."));
  }
}

unsigned long delayOneSecMillis;
void loop()
{
  if (!udpOk)
  {
    if (millis() - delayOneSecMillis >= 1000)
    {
      delayOneSecMillis = millis();
      udpLoop();
    }
  }
  else
  {
    if (!mqttClient.connected())
    {
      if (millis() - lastReconnectAttempt > 5000)
      {
        lastReconnectAttempt = millis();
        // Attempt to reconnect
        if (reconnect())
        {
          lastReconnectAttempt = 0;
          Serial.print(F("Mqtt connected IP: "));
          Serial.print(mqttServer);
        }
      }
    }
    else // Client connected
    {

      mqttClient.loop();
      dhtLoop();
      ambientLoop();
      bmp280Loop();
      // testFloatLoop();
    }
  }
}
