#include "net_mqtt.h"

// constants
#define TAG "net_mqtt"

#define MQTT_BROKER "192.168.1.2"
#define MQTT_PORT 1883
#define MQTT_USER "TGR_GROUP22"
#define MQTT_PW "BV593V"

IPAddress subnet(255, 255, 255, 0);
IPAddress local_IP(192, 168, 1, 73);
IPAddress gateway(192, 168, 1, 2);

// static variables
static WiFiClient wifi_client;
static PubSubClient mqtt_client(wifi_client);

// connect WiFi and MQTT broker
void net_mqtt_init(char *ssid, char *passwd)
{
    // initialize WiFi
    if (!WiFi.config(local_IP, gateway, subnet))
    {
        Serial.println("STA Failed to configure");
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, passwd);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(10);
        // Serial.println("Connecting to WiFi...");
        ESP_LOGI(TAG, "Connecting to WiFi...");
    }
    // Serial.println("Connected to wifi");
    ESP_LOGI(TAG, "Connected to wifi");

    // initialize MQTT
    mqtt_client.setServer(MQTT_BROKER, MQTT_PORT);
}

// connect and subscribe to topic
void net_mqtt_connect(unsigned int dev_id, char *topic, mqtt_callback_t msg_callback)
{
    String client_id = "tgr2023_" + String(dev_id);
    // mqtt_client.setServer(gateway, 1883);
    mqtt_client.setCallback(msg_callback);

    while (!mqtt_client.connected())
    {
        Serial.println("mqtt not connect");
        if (mqtt_client.connect(client_id.c_str(), MQTT_USER, MQTT_PW))
        {
            // Serial.println("Public EMQX MQTT broker connected");
            ESP_LOGI(TAG, "MQTT broker connected");
            mqtt_client.subscribe(topic);
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(mqtt_client.state());
            delay(2000);
        }
    }
    // Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    ESP_LOGI(TAG, "connect to mqtt broker");

    // mqtt_client.connect(client_id.c_str());
    // mqtt_client.subscribe(topic);
}

// publish message to topic
void net_mqtt_publish(char *topic, char *payload)
{
    mqtt_client.publish(topic, payload);
}

// maintain MQTT connection
void net_mqtt_loop(void)
{
    mqtt_client.loop();
}