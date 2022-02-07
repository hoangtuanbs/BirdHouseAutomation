#include <IotWebConf.h>
#include <IotWebConfESP32HTTPUpdateServer.h>
#include <IotWebConfMultipleWifi.h>
#include <IotWebConfOptionalGroup.h>
#include <IotWebConfParameter.h>
#include <IotWebConfSettings.h>
#include <IotWebConfTParameter.h>
#include <IotWebConfTParameterBuilder.h>
#include <IotWebConfUsing.h>
#include <IotWebConfWebServerWrapper.h>

/**
    Humidity sensor
    Publish data read from Bosch Bme280 sensor to a mqttbroker defined according to specs
    https://www.mysensors.org/build/mqtt_gateway
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT_U.h>
#include <DHT.h>

#include <BME280I2C.h>
#include <Wire.h>

#define SERIAL_BAUD 115200
#define GLOBAL_SOFTWARE_VERSION "0.1.0"
#define CONFIG_VERSION "1.0"
// Update these with values suitable for your network.


/**
 * @brief Device configuration region
 *
 */

#include <ESP_EEPROM.h>

#define BROKER_LENGTH 127
struct ControllerData
{
    uint8_t SensorNodeId = 1;
    uint8_t TempSensorId = 20;
    uint8_t HumiditySensorId = 21;
    uint8_t SensorUpdateInterval = 30; // in seconds
    uint8_t SensorForceUpdateInterval = 30; // in minute
    float TemperatureSensorDelta = 0.5;
    float HumiditySensorDelta = 1;
    char MqttServer[BROKER_LENGTH] = "sitecontroller";
    char MqttTopic[BROKER_LENGTH] = "g/sc/f/sensor";
    bool AutomaticFan = true;
} DeviceConfig;

bool saveRomData()
{
    EEPROM.put(0, DeviceConfig);

    return EEPROM.commit();
}

bool loadRomData()
{
    EEPROM.begin(sizeof(ControllerData));

    if (EEPROM.percentUsed()>=0)
    {
        EEPROM.get(0, DeviceConfig);

        return true;
    } else
    {
        return saveRomData();
    }
}

/* End device configuration */

/**
 * @brief Device network configuration region
 *
 */
const char *ssid = "GLink";
const char *password = "Gfarming1512";
const char *mqtt_server = "sitecontroller";
String DeviceName{"CombiSensor" + String(ESP.getChipId())};
// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char WifiInitialPassword[] = "12345678";

#define CONFIG_PIN D2
#define NUMBER_LEN 32
#define STRING_LEN 128
#define STATUS_PIN LED_BUILTIN

WiFiClient LocalEspClient;
PubSubClient LocalMqttClient(LocalEspClient);

DNSServer LocalDnsServer;
WebServer LocalWebServer(80);

char stringParamValue[STRING_LEN];
char intParamValue[NUMBER_LEN];
char floatParamValue[NUMBER_LEN];
char checkboxParamValue[STRING_LEN];
char chooserParamValue[STRING_LEN];
static char chooserValues[][STRING_LEN] = {"red", "blue", "darkYellow"};
static char chooserNames[][STRING_LEN] = {"Red", "Blue", "Dark yellow"};

// -- Method declarations.
void handleRoot();
// -- Callback methods.
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper);

IotWebConf iotWebConf(DeviceName.c_str(), &LocalDnsServer, &LocalWebServer, WifiInitialPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::TextParameter
IotWebConfTextParameter paramMqttServer =
        ("Mqtt Server", "mqttServer", DeviceConfig.MqttServer, BROKER_LENGTH, "text", nullptr, DeviceConfig.MqttServer);
IotWebConfTextParameter paramMqttServer =
        ("Mqtt Topic", "mqttTopic", DeviceConfig.MqttTopic, BROKER_LENGTH, "text", nullptr, DeviceConfig.MqttTopic);

IotWebConfParameterGroup group1 = IotWebConfParameterGroup("Sensor Config", "");
IotWebConfNumberParameter intParam = IotWebConfNumberParameter("Int param", "intParam", intParamValue, NUMBER_LEN, "20", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter paramNodeId = IotWebConfNumberParameter("NodeId", "paramNodeId", DeviceConfig.SensorNodeId, NUMBER_LEN, "1", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter paramHumidityId = IotWebConfNumberParameter("Humidity id", "paramHumidityId", DeviceConfig.HumiditySensorId, NUMBER_LEN, "1", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter paramTempId = IotWebConfNumberParameter("Temperature id", "paramHumidityId", DeviceConfig.TempSensorId, NUMBER_LEN, "2", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter paramUpdateInt = IotWebConfNumberParameter("Temperature id", "paramHumidityId", DeviceConfig.TempSensorId, NUMBER_LEN, "2", "1..100", "min='1' max='100' step='1'");
IotWebConfNumberParameter paramForceUpdateInt = IotWebConfNumberParameter("Temperature id", "paramHumidityId", DeviceConfig.TempSensorId, NUMBER_LEN, "2", "1..100", "min='1' max='100' step='1'");

// -- We can add a legend to the separator
IotWebConfParameterGroup group2 = IotWebConfParameterGroup("c_factor", "Calibration factor");
IotWebConfNumberParameter floatParam = IotWebConfNumberParameter("Float param", "floatParam", floatParamValue, NUMBER_LEN, nullptr, "e.g. 23.4", "step='0.1'");
IotWebConfCheckboxParameter checkboxParam = IotWebConfCheckboxParameter("Check param", "checkParam", checkboxParamValue, STRING_LEN, true);
IotWebConfSelectParameter chooserParam = IotWebConfSelectParameter("Choose param", "chooseParam", chooserParamValue, STRING_LEN, (char *)chooserValues, (char *)chooserNames, sizeof(chooserValues) / STRING_LEN, STRING_LEN);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (250)
#define TOPIC_SIZE (250)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setupMqttTopics()
{
    snprintf(tempTopic, TOPIC_SIZE, "g/sc/f/sensor/%d/%d/1/0/%d", fromNode, tempSensorId, V_TEMP);
    snprintf(humTopic, TOPIC_SIZE, "g/sc/f/sensor/%d/%d/1/0/%d", fromNode, humSensorId, V_HUM);
}

void setupWifi()
{
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setupDevice()
{
    group1.addItem(&intParam);
    group2.addItem(&floatParam);
    group2.addItem(&checkboxParam);
    group2.addItem(&chooserParam);

    iotWebConf.setStatusPin(STATUS_PIN);
    iotWebConf.setConfigPin(CONFIG_PIN);
    iotWebConf.addSystemParameter(&paramMqttServer);
    iotWebConf.addParameterGroup(&group1);
    iotWebConf.addParameterGroup(&group2);
    iotWebConf.setConfigSavedCallback(&configSaved);
    iotWebConf.setFormValidator(&formValidator);
    iotWebConf.getApTimeoutParameter()->visible = true;

    // -- Initializing the configuration.
    iotWebConf.init();

    // -- Set up required URL handlers on the web server.
    server.on("/", handleRoot);
    server.on("/config", []
              { iotWebConf.handleConfig(); });
    server.onNotFound([]()
                      { iotWebConf.handleNotFound(); });
}

void loopDvice()
{
    iotWebConf.doLoop();
}

void handleRoot()
{
    // -- Let IotWebConf test and handle captive portal requests.
    if (iotWebConf.handleCaptivePortal())
    {
        // -- Captive portal request were already served.
        return;
    }
    String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
    s += "<title>IotWebConf 03 Custom Parameters</title></head><body>Hello world!";
    s += "<ul>";
    s += "<li>String param value: ";
    s += stringParamValue;
    s += "<li>Int param value: ";
    s += atoi(intParamValue);
    s += "<li>Float param value: ";
    s += atof(floatParamValue);
    s += "<li>CheckBox selected: ";
    //  s += checkboxParam.isChecked();
    s += "<li>Option selected: ";
    s += chooserParamValue;
    s += "</ul>";
    s += "Go to <a href='config'>configure page</a> to change values.";
    s += "</body></html>\n";

    server.send(200, "text/html", s);
}

void configSaved()
{
    Serial.println("Configuration was updated.");
}

bool formValidator(iotwebconf::WebRequestWrapper *webRequestWrapper)
{
    Serial.println("Validating form.");
    bool valid = true;

    /*
        int l = webRequestWrapper->arg(stringParam.getId()).length();
        if (l < 3)
        {
            stringParam.errorMessage = "Please provide at least 3 characters for this test!";
            valid = false;
        }
        */
    return valid;
}

/**
* @brief Sensor region
*/

BME280I2C BmeSensor;
const uint32_t SENSOR_DELAY_MS = DeviceConfig.SensorUpdateInterval*1000; // 30 seconds
const uint32_t FORCE_SENSOR_PERIOD_MS = DeviceConfig.SensorForceUpdateInterval*60*1000; // 30 minute
#define V_TEMP 0
#define V_HUM 1

int fromNode = DeviceConfig.SensorNodeId;
char tempTopic[TOPIC_SIZE];
int tempSensorId = DeviceConfig.TempSensorId; // Configurable
char humTopic[TOPIC_SIZE];
int humSensorId = DeviceConfig.HumiditySensorId; // Configurable

const float INVALID_SENSOR_VALUE = -10000;
const float TEMPERATURE_DELTA = DeviceConfig.TemperatureSensorDelta;
const float HUMIDITY_DELTA = DeviceConfig.HumiditySensorDelta;

struct SensorDataStruct
{
    float Humidity = INVALID_SENSOR_VALUE;
    float Temperature = INVALID_SENSOR_VALUE;
    float Pressure = INVALID_SENSOR_VALUE;
} SensorData;

void setupSensors()
{
    Wire.begin();

    while(!BmeSensor.begin())
    {
        Serial.println("Could not find BME280 sensor!");
        delay(1000);
    }
}

void readSensors(SensorDataStruct &data)
{
    BmeSensor.read(data.Pressure, data.Temperature, data.Humidity, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
}

/* End sensor configuration */

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1')
    {
        digitalWrite(BUILTIN_LED, LOW); // Turn the LED on (Note that LOW is the voltage level
                                        // but actually the LED is on; this is because
                                        // it is active low on the ESP-01)
    }
    else
    {
        digitalWrite(BUILTIN_LED, HIGH); // Turn the LED off by making the voltage HIGH
    }
}

void reconnect()
{
    // Loop until we're reconnected
    while (!LocalMqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (LocalMqttClient.connect(clientId.c_str()))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            //client.publish("outTopic", "hello world");
            // ... and resubscribe
            //client.subscribe("inTopic");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(LocalMqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    setupMqttTopics();
    setupWifi();
    setupSensors();
    LocalMqttClient.setServer(mqtt_server, 1883);
    LocalMqttClient.setCallback(callback);
}

void loop()
{
    if (!LocalMqttClient.connected())
    {
        reconnect();
    }
    LocalMqttClient.loop();

    // Check conditions for force update
    unsigned long now = millis();
    bool forceUpdate = false;
    if (now - lastMsg > FORCE_SENSOR_PERIOD_MS ||
            SensorData.Humidity == INVALID_SENSOR_VALUE ||
            SensorData.Temperature == INVALID_SENSOR_VALUE
    )
    {
        forceUpdate = true;
    }

    if (now - lastMsg > SENSOR_DELAY)
    {
        lastMsg = now;

        SensorDataStruct data;
        readSensors(data);

        // Ignore invalid sensor data
        if (data.Humidity == INVALID_SENSOR_VALUE || data.Temperature == INVALID_SENSOR_VALUE ||
                data.Humidity == NAN || data.Temperature == NAN)
        {
            return;
        }

        if (forceUpdate || abs(SensorData.Humidity - data.Humidity) > HUMIDITY_DELTA)
        {
            SensorData.Humidity = data.Humidity;
            snprintf(msg, MSG_BUFFER_SIZE, "%.2f", SensorData.Humidity);
            Serial.print("Publish message: ");
            Serial.println(msg);
            LocalMqttClient.publish(humTopic, msg);
        }

        if (forceUpdate || abs(SensorData.Temperature - data.Temperature) > TEMPERATURE_DELTA)
        {
            SensorData.Temperature = data.Temperature;
            snprintf(msg, MSG_BUFFER_SIZE, "%.2f", SensorData.Temperature);
            Serial.print("Publish message: ");
            Serial.println(msg);
            LocalMqttClient.publish(tempTopic, msg);
        }
    }
}