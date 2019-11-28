#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"

#include "tcpip_adapter.h"



#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"


// FreeRTOS function
#define INCLUDE_vTaskDelay 1

//constants

#define trig_sensor_R  33 
#define echo_sensor_R  39 
#define trig_sensor_L  32 
#define echo_sensor_L  36 
#define led_R  18
#define led_L  19
#define led_RED  23
#define led_YELLOW  22
#define led_GREEN  21
#define buzzer  4

#define LEFT_TRIG_CH RMT_CHANNEL_0
#define RIGHT_TRIG_CH RMT_CHANNEL_1
#define LEFT_ECHO_CH RMT_CHANNEL_2
#define RIGHT_ECHO_CH RMT_CHANNEL_3


//User configs

#define WIFI_SSID "AndroidAP"
#define WIFI_PASS "1234567891011"
#define MQTT_URL "mqtt://192.168.43.178"
#define RED_THRESH 80
#define YELLOW_THRESH 130

//global variables
tcpip_adapter_ip_info_t ipInfo; 
esp_mqtt_client_handle_t client;
const int CONNECTED_BIT = BIT0;
uint8_t mac_addr[6] = {};
static EventGroupHandle_t wifi_event_group;
bool online = 0;
typedef struct mqtt_msq  {
    uint8_t len;
    char strbuf[30];
} mqtt_msq_t;

mqtt_msq_t car_available;
mqtt_msq_t car_taken;
uint32_t previousMillis = 0;
uint8_t buzzerState = 0;
uint8_t dist_err_left = 0;
uint8_t dist_err_right = 0;
int period = 0;




//forward declarations
void gpio_setup();
void ultrasonic_config(void);
int wifi_init(void);
static esp_err_t event_handler(void *ctx, system_event_t *event);
inline void pinMode(uint8_t pin, uint8_t mode);
void blinkBuzzer(int dist);


//main function
int app_main(){
    gpio_setup();
    ultrasonic_config();
    wifi_init();
        rmt_item32_t ping;
        ping.level0 = 1;
        ping.duration0 = 10;//us
        ping.level1 = 0;
        ping.duration1 = 10;//us

     ESP_LOGI("ParkSense","Starting up main loop");
     vTaskDelay(1000 / portTICK_PERIOD_MS);
  
    RingbufHandle_t rb_left = NULL;
    rmt_get_ringbuf_handle(LEFT_ECHO_CH,&rb_left);

    RingbufHandle_t rb_right = NULL;
    rmt_get_ringbuf_handle(RIGHT_ECHO_CH,&rb_right);




    int distance_L = 0;
    int distance_R = 0;
    int distance_average = 0;
    int one_shot = 1;
    size_t rx_size = 0;

     while(1){ //Arduino loop() 


        rmt_rx_start(LEFT_ECHO_CH,1);
        rmt_rx_start(RIGHT_ECHO_CH,1);
        rmt_write_items(LEFT_TRIG_CH,&ping,1,true);
        rmt_write_items(RIGHT_TRIG_CH,&ping,1,true);
        //rmt_tx_start(0,true);
        rmt_wait_tx_done(0,10);
        
        rmt_item32_t *item_left = (rmt_item32_t *)xRingbufferReceive(rb_left,&rx_size,20);
        rmt_item32_t *item_right = (rmt_item32_t *)xRingbufferReceive(rb_right,&rx_size,20);
        rmt_rx_stop(LEFT_ECHO_CH);
        rmt_rx_stop(RIGHT_ECHO_CH);
        int ctr = 0;

        if (item_left){
            distance_L = item_left->duration0;
            //printf("Left pulse description: %d %d, %d %d data %d\n",item_left->level0,item_left->duration0,item_left->level1,item_left->duration1,ctr++); // distance in centimeters
            if (item_left->level0 == 1) {
                distance_L = item_left->duration0 / 20; }
            else {
                distance_L = 999999;
            }
            vRingbufferReturnItem(rb_left, (void*) item_left);
            dist_err_left = 0;
        } else {
            ESP_LOGE("ram","skipped left buffer!");
            dist_err_left = 1;
        }

        if (item_right){
            distance_R = item_right->duration0;
            //printf("Right pulse description: %d %d, %d %d data %d\n",item_right->level0,item_right->duration0,item_right->level1,item_right->duration1,ctr++); // distance in centimeters
            if (item_right->level0 == 1) {
                distance_R = item_right->duration0 / 20; }
            else {
                distance_R = 999999;
            }
            vRingbufferReturnItem(rb_right, (void*) item_right);
            dist_err_right = 0;
        }else {
            ESP_LOGE("ram","skipped right buffer!");
            dist_err_right = 1;
        }


       
        //printf("right distance is %d mm\n",distance_R);        
    
    // Arduino code port starts here

    if (dist_err_right || dist_err_left) {
        distance_L = 999999;
        distance_R = 999999;
    }

    printf("left distance is %d cm \t right distance is %d cm diff is %d cm \n",distance_L, distance_R, distance_L - distance_R);
    distance_average = (distance_R+distance_L)/2; 
  
  if (distance_R < YELLOW_THRESH && distance_L < YELLOW_THRESH){
      if (distance_R > distance_L + 5){
         //delay(500);
         gpio_set_level (led_L, 1);
         gpio_set_level (led_R,0);
         //delay (500);
      }
      else if (distance_L > distance_R + 5){ 
         gpio_set_level (led_L, 0);
         //delay (500);
         gpio_set_level (led_R, 1);
         //delay (500);
      }
      else {
         gpio_set_level (led_R, 0);
         //delay (500);
         gpio_set_level (led_L, 0);
         //delay (500);
        
      }
    
  }
  else{
      gpio_set_level (led_R, 0);
      //delay (500);
      gpio_set_level (led_L, 0);
      //delay (500);
  }


  if (distance_average > YELLOW_THRESH){
      gpio_set_level(led_GREEN,1);
      gpio_set_level(led_RED,0);
      gpio_set_level(led_YELLOW,0);
      //Serial.println("here");

      if (one_shot == 0) {
        one_shot = 1;
        //client.publish("esp32/parking","car_available");
        esp_mqtt_client_publish(client, "esp32/parking",car_available.strbuf,car_available.len, 0, 0);
      }
    } 
  else if (distance_average >=RED_THRESH && distance_average <= YELLOW_THRESH) {
      gpio_set_level(led_GREEN,0 ); //do we really need green and red to show yellow signal ? or buy LED with yellow light?
      gpio_set_level(led_RED,0);
      gpio_set_level(led_YELLOW,1);
      period = 30;
      if (one_shot == 1) {
        one_shot = 0;
        //client.publish("esp32/parking","car_taken");
        esp_mqtt_client_publish(client, "esp32/parking",car_taken.strbuf,car_taken.len, 0, 0);
      }
    }
  else {
      gpio_set_level(led_GREEN,0);
      gpio_set_level(led_RED,1);
      gpio_set_level(led_YELLOW,0);
      period = 10;
      if (one_shot == 1) {
        one_shot = 0;
        //client.publish("esp32/parking","car_taken");
        esp_mqtt_client_publish(client, "esp32/parking",car_taken.strbuf,car_taken.len, 0, 0);
      }
    }

    blinkBuzzer(distance_average);
  
    //and ends here
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

}


//configs



void ultrasonic_config(void){
    //Configuration common to both left and right ultrasonic sensor TX
    //Flush config


    rmt_tx_config_t tx_config;
    tx_config.carrier_en = 0; //DC transmission
    tx_config.idle_level = 0;
    tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    tx_config.idle_output_en = RMT_IDLE_LEVEL_LOW; //make pin not floating
    tx_config.loop_en = 0;

    //Configure left TX channel

    rmt_config_t left_sensor_tx;
    left_sensor_tx.rmt_mode = RMT_MODE_TX;
    left_sensor_tx.channel = LEFT_TRIG_CH;
    left_sensor_tx.clk_div = 80; //1 uS period
    left_sensor_tx.gpio_num = trig_sensor_L;
    left_sensor_tx.mem_block_num = 1;
    left_sensor_tx.tx_config = tx_config;

    //Configure right TX channel

    rmt_config_t right_sensor_tx;
    right_sensor_tx.rmt_mode = RMT_MODE_TX;
    right_sensor_tx.channel = RIGHT_TRIG_CH;
    right_sensor_tx.clk_div = 80; //1 uS period
    right_sensor_tx.gpio_num = trig_sensor_R;
    right_sensor_tx.mem_block_num = 1;
    right_sensor_tx.tx_config = tx_config;

    //Configuration common to both left and right ultrasonic sensor RX
    rmt_rx_config_t rx_config = {
    rx_config.filter_en = 1,
    rx_config.filter_ticks_thresh = 2,
    rx_config.idle_threshold = 10000,
    }; //timeout in 3ms

    //Configure left RX channel
    rmt_config_t left_sensor_rx = {
    .rmt_mode = RMT_MODE_RX,
    .channel = LEFT_ECHO_CH,
    .clk_div = 235, //340 kHz
    .gpio_num = echo_sensor_L,
    .mem_block_num = 1,
    .rx_config = rx_config,
    };

    rmt_config_t right_sensor_rx = 
    {.rmt_mode = RMT_MODE_RX,
    .channel = RIGHT_ECHO_CH,
    .clk_div = 235, //340 kHz
    .gpio_num = echo_sensor_R,
    .mem_block_num = 1,
    .rx_config = rx_config,
    };

    rmt_config(&left_sensor_tx);
    rmt_config(&right_sensor_tx);
    rmt_driver_install(left_sensor_tx.channel,0,0);
    rmt_driver_install(right_sensor_tx.channel,0,0);


    rmt_config(&left_sensor_rx);
    rmt_config(&right_sensor_rx);
    rmt_driver_install(left_sensor_rx.channel,1000,0);
    rmt_driver_install(right_sensor_rx.channel,1000,0);

}



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}



int wifi_init(void){
    esp_err_t err = ESP_OK;
    ESP_LOGI("IOT", "Connecting to WiFi");
    nvs_flash_init();
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    esp_event_loop_init(event_handler, NULL); 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    err = esp_wifi_start();
    if (err == ESP_ERR_WIFI_CONN) {
        ESP_LOGE("IOT","WiFi module failed setup");
    }
    err = esp_wifi_connect();

    if (err == ESP_ERR_WIFI_SSID){
        ESP_LOGE("IOT", "WiFi SSID not found");
    } else if (err == ESP_ERR_WIFI_PASSWORD){
        ESP_LOGE("IOT", "WiFi password is wrong");
    } else if (err == ESP_OK){
        ESP_LOGI("IOT", "Successfully connected to WiFi, waiting for IP address");
    }

    if (err != ESP_OK){
        ESP_LOGI("ParkSense", "Starting ParkSense in offline mode");
        return err;
    }

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);

    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA,&ipInfo);
    //ESP_LOGI("IOT", "IP address: " IPSTR " netmask: " IPSTR " default gateway: " IPSTR , IP2STR(&ipInfo.ip),IP2STR(&ipInfo.netmask),IP2STR(&ipInfo.gw));


    const esp_mqtt_client_config_t mqtt_cfg = {
    .uri = MQTT_URL,
    // .user_context = (void *)your_context
    };



    client = esp_mqtt_client_init(&mqtt_cfg);    


    err = esp_mqtt_client_start(client);

    if (err == ESP_FAIL){
         ESP_LOGE("IOT", "MQTT connection failed");
    }

    vTaskDelay(5000/portTICK_PERIOD_MS);
    ESP_LOGI("IOT", "Sending test connection");

    esp_wifi_get_mac(WIFI_MODE_STA,mac_addr);

    //Initialize MQTT messages
    car_available.len = sprintf(car_available.strbuf,"%x%x%x%x%x%x:car_available",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
    car_taken.len = sprintf(car_taken.strbuf,"%x%x%x%x%x%x:car_taken",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);

    esp_mqtt_client_publish(client, "esp32/parking",car_available.strbuf,car_available.len, 0, 0);

    if (err != ESP_OK){
        ESP_LOGI("ParkSense", "Starting ParkSense in offline mode");
        return err;
    }
    online = true;
    
    return 0;

}

void gpio_setup(){
  pinMode (trig_sensor_R, GPIO_MODE_OUTPUT);
  pinMode (echo_sensor_R, GPIO_MODE_INPUT);
  pinMode (trig_sensor_L, GPIO_MODE_OUTPUT);
  pinMode (echo_sensor_L, GPIO_MODE_INPUT);
  pinMode (led_R, GPIO_MODE_OUTPUT);
  pinMode (led_L, GPIO_MODE_OUTPUT);
  pinMode (led_RED, GPIO_MODE_OUTPUT);
  pinMode (led_YELLOW, GPIO_MODE_OUTPUT);
  pinMode (led_GREEN, GPIO_MODE_OUTPUT);
  pinMode (buzzer, GPIO_MODE_OUTPUT);

}

//helper functions
inline void pinMode(uint8_t pin, uint8_t mode){
    gpio_pad_select_gpio(pin);
    gpio_set_direction(pin, mode);
}

void blinkBuzzer(int dist){
    uint32_t currentMillis =  xTaskGetTickCount();
    //printf("currentMillis = %d prevMillis = %d\n",currentMillis,previousMillis);

    if (dist > YELLOW_THRESH) {
      gpio_set_level(buzzer,0);
    } else {
        if (currentMillis - previousMillis >= period) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        // if the LED is off turn it on and vice-versa:
        if (buzzerState == 0) {
          buzzerState = 1;
        } else {
          buzzerState = 0;
        }
        // set the LED with the ledState of the variable:
        gpio_set_level(buzzer, buzzerState);
      }
    }

}
