#include "wifi_transport.h"

#include "esp_wifi.h" //contain wifi driver
#include "esp_event.h"  //create event as while() in arduino framework
#include "esp_log.h" // for loggin purpose better than printf
#include "nvs_flash.h" //non_volatile_memory--> for storing wifi crediantial 

#define WIFI_SSID "Local"
#define WIFI_PASS "qwerty123456"

static const char *TAG= "WIFI"; //used while logging to know from which module log is coming,  [wifi]: connection....

void wifi_init_sta(void) // start wifi station --> wifi client, another one is AP access point esp32 khud wifi banayega (server)
{
   esp_netif_init(); // network interface : matlab networking system start karo
   esp_event_loop_create_default(); //event system on: ek notice board banao jahan system bataye kya ho rha hai: jab wifi connect ho mujhe batao
   esp_netif_create_default_wifi_sta(); //wifi ke liye ek network card banao
   
   wifi_init_config_t cfg= WIFI_INIT_CONFIG_DEFAULT(); //wifi driver config lao
   esp_wifi_init(&cfg); //wifi driver start hone ke liye ready karo

   wifi_config_t wifi_config= {
        .sta={                       //wifi station (client) ke liye config
            .ssid=WIFI_SSID,
            .password=WIFI_PASS,
        },
   };
   esp_wifi_set_mode(WIFI_MODE_STA); //client ban ke connect kar
   esp_wifi_set_config(WIFI_IF_STA,&wifi_config);  //config apply karo
   esp_wifi_start(); //wifi start karo ab

   ESP_LOGI(TAG,"WiFi initialized");

}

//this file is node needed for this project this one is used for custom network transport