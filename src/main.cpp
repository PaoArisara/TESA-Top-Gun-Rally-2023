#include "main.h"
#include "hw_camera.h"
#include "net_mqtt.h"

// add header file of Edge Impulse firmware

#include <topgunV1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// constants
#define TAG "main"
#define BTN_PIN 0
#define WIFI_SSID "TGR17_2.4G"
#define WIFI_PASSWORD NULL
#define MQTT_EVT_TOPIC "TGR_22/water_level"
#define MQTT_CMD_TOPIC "TGR_22/#"
#define MQTT_DEV_ID 22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 240
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
#define BMP_BUF_SIZE (EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE)

// static variables
static uint8_t *bmp_buf;
static char buf[128];
int label;
bool capture = false;

// declare ArduinoJson objects for cmd_buf and evt_buf
StaticJsonDocument<128> cmd_buf;
StaticJsonDocument<128> evt_buf;

// static function prototypes
void print_memory(void);
void ei_prepare_feature(uint8_t *img_buf, signal_t *signal);
int ei_get_feature_callback(size_t offset, size_t length, float *out_ptr);
int ei_use_result(ei_impulse_result_t result);
static void mqtt_callback(char *topic, byte *payload, unsigned int length);

// initialize hardware
void setup()
{
  Serial.begin(115200);
  print_memory();
  pinMode(BTN_PIN, INPUT_PULLUP);
  hw_camera_init();
  bmp_buf = (uint8_t *)ps_malloc(BMP_BUF_SIZE);
  if (psramInit())
  {
    ESP_LOGI(TAG, "PSRAM initialized");
  }
  else
  {
    ESP_LOGE(TAG, "PSRAM not available");
  }
  // connect to WiFi
  net_mqtt_init(WIFI_SSID, WIFI_PASSWORD);
  // connect to MQTT broker
  net_mqtt_connect(MQTT_DEV_ID, MQTT_CMD_TOPIC, mqtt_callback);
  Serial.println("start");
}

// main loop
void loop()
{
  static bool press_state = false;
  static uint32_t prev_millis = 0;

  if (digitalRead(BTN_PIN) == 0)
  {
    if ((millis() - prev_millis > 500) && (press_state == false))
    {
      net_mqtt_publish(MQTT_EVT_TOPIC, "pressed");
    }
  }
  if (capture)
  {
    net_mqtt_publish(MQTT_EVT_TOPIC, "captured");
    uint32_t Tstart, elapsed_time;
    uint32_t width, height;

    prev_millis = millis();
    Tstart = millis();
    // get raw data
    ESP_LOGI(TAG, "Taking snapshot...");
    // use raw bmp image
    hw_camera_raw_snapshot(bmp_buf, &width, &height);

    elapsed_time = millis() - Tstart;
    ESP_LOGI(TAG, "Snapshot taken (%d) width: %d, height: %d", elapsed_time, width, height);
    print_memory();
    // prepare feature
    Tstart = millis();
    ei::signal_t signal;
    // generate feature
    ei_prepare_feature(bmp_buf, &signal);
    elapsed_time = millis() - Tstart;
    ESP_LOGI(TAG, "Feature taken (%d)", elapsed_time);
    print_memory();
    // run classifier
    Tstart = millis();
    ei_impulse_result_t result = {0};
    bool debug_nn = false;
    // run classifier
    run_classifier(&signal, &result, debug_nn);
    elapsed_time = millis() - Tstart;
    ESP_LOGI(TAG, "Classification done (%d)", elapsed_time);
    print_memory();
    ei_use_result(result);
    press_state = true;
    capture = false;
  }
  // else
  // {
  //   if (press_state)
  //   {
  //     press_state = false;
  //   }
  // }
  net_mqtt_loop();
  delay(100);
}

// Print memory information
void print_memory()
{
  ESP_LOGI(TAG, "Total heap: %u", ESP.getHeapSize());
  ESP_LOGI(TAG, "Free heap: %u", ESP.getFreeHeap());
  ESP_LOGI(TAG, "Total PSRAM: %u", ESP.getPsramSize());
  ESP_LOGI(TAG, "Free PSRAM: %d", ESP.getFreePsram());
}

// prepare feature
void ei_prepare_feature(uint8_t *img_buf, signal_t *signal)
{
  signal->total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal->get_data = &ei_get_feature_callback;
  if ((EI_CAMERA_RAW_FRAME_BUFFER_ROWS != EI_CLASSIFIER_INPUT_WIDTH) || (EI_CAMERA_RAW_FRAME_BUFFER_COLS != EI_CLASSIFIER_INPUT_HEIGHT))
  {
    ei::image::processing::crop_and_interpolate_rgb888(
        img_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        img_buf,
        EI_CLASSIFIER_INPUT_WIDTH,
        EI_CLASSIFIER_INPUT_HEIGHT);
  }
}

// get feature callback
int ei_get_feature_callback(size_t offset, size_t length, float *out_ptr)
{
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
    out_ptr[out_ptr_ix] = (bmp_buf[pixel_ix] << 16) + (bmp_buf[pixel_ix + 1] << 8) + bmp_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  return 0;
}

// use result from classifier
int ei_use_result(ei_impulse_result_t result)
{
  ESP_LOGI(TAG, "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
           result.timing.dsp, result.timing.classification, result.timing.anomaly);
  bool bb_found = result.bounding_boxes[0].value > 0;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++)
  {
    auto bb = result.bounding_boxes[ix];
    if (bb.value == 0)
    {
      continue;
    }
    ESP_LOGI(TAG, "%s (%f) [ x: %u, y: %u, width: %u, height: %u ]", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    evt_buf["label : "] = bb.label;
    serializeJson(evt_buf, buf);
    net_mqtt_publish(MQTT_EVT_TOPIC, buf);
    Serial.print("water level : ");
    Serial.println(bb.label);
  }
  if (!bb_found)
  {
    ESP_LOGI(TAG, "not valid");
    Serial.println("not valid");
    net_mqtt_publish(MQTT_EVT_TOPIC, "not valid");

    return 0;
  }
  return 1;
}

// callback function to handle MQTT message
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  ESP_LOGI(TAG, "Message arrived on topic %s", topic);
  ESP_LOGI(TAG, "Payload: %.*s", length, payload);
  payload[length] = '\0';
  if (strcmp((char *)payload, "get") == 0)
  {
    capture = true;
  }
  else
  {
    capture = false;
  }

  // (char *topic, byte *payload, unsigned int length)
  // {
  // extract data from payload
  // char tmpbuf[128];
  // memcpy(tmpbuf, payload, length);
  // tmpbuf[length] = 0;
  // // check if the message is for this device
  // deserializeJson(cmd_buf, tmpbuf);
  // if (tmpbuf == "get")
  // {
  //   capture = true;
  // }
  // else
  // {
  //   capture = false;
  // }
  // }
}