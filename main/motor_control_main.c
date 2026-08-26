#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mecanum_drive.h"
#include "nvs_flash.h"
#include "wifi_credentials.h"

#define UDP_PORT 3333
#define COMMAND_BUFFER_SIZE 96
static const char *TAG = "robot_control";
static const mecanum_wheel_t s_physical_motor_map[] = {
    MECANUM_WHEEL_REAR_RIGHT,  // M1
    MECANUM_WHEEL_REAR_LEFT,   // M2
    MECANUM_WHEEL_FRONT_LEFT,  // M3
    MECANUM_WHEEL_FRONT_RIGHT, // M4
};

static void wifi_events(void *arg, esp_event_base_t base, int32_t event_id, void *data) {
    (void)arg;
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { ESP_LOGW(TAG, "station disconnected; AP remains available"); esp_wifi_connect(); }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { ip_event_got_ip_t *event=data; ESP_LOGI(TAG,"station IP " IPSTR, IP2STR(&event->ip_info.ip)); }
}
static void start_wifi(void) {
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_events, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_events, NULL));
    wifi_config_t ap = {.ap={.ssid=ROBOT_WIFI_AP_SSID,.ssid_len=sizeof(ROBOT_WIFI_AP_SSID)-1,.channel=1,.password=ROBOT_WIFI_AP_PASSWORD,.max_connection=2,.authmode=WIFI_AUTH_WPA2_PSK}};
    wifi_config_t sta = {.sta={.ssid=ROBOT_WIFI_STA_SSID,.password=ROBOT_WIFI_STA_PASSWORD,.threshold.authmode=WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&sta)); ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP ready at 192.168.4.1, UDP/%d", UDP_PORT);
}
static const char *process_command(const char *command, char *reply, size_t reply_size) {
    if (!strcmp(command, "telemetry")) {
        mecanum_drive_telemetry_t telemetry;
        if (mecanum_drive_get_telemetry(&telemetry) != ESP_OK) return "error: telemetry unavailable\n";
        snprintf(reply, reply_size,
                 "target %.2f %.2f %.2f %.2f; rpm %.0f %.0f %.0f %.0f; fg %u %u %u %u; total %llu %llu %llu %llu; duty %u %u %u %u; reversing %u %u %u %u\n",
                 telemetry.commanded[0], telemetry.commanded[1], telemetry.commanded[2], telemetry.commanded[3],
                 telemetry.measured_rpm[0], telemetry.measured_rpm[1], telemetry.measured_rpm[2], telemetry.measured_rpm[3],
                 (unsigned)telemetry.encoder_edges[0], (unsigned)telemetry.encoder_edges[1],
                 (unsigned)telemetry.encoder_edges[2], (unsigned)telemetry.encoder_edges[3],
                 (unsigned long long)telemetry.encoder_total_edges[0],
                 (unsigned long long)telemetry.encoder_total_edges[1],
                 (unsigned long long)telemetry.encoder_total_edges[2],
                 (unsigned long long)telemetry.encoder_total_edges[3],
                 (unsigned)telemetry.duty_percent[0], (unsigned)telemetry.duty_percent[1],
                 (unsigned)telemetry.duty_percent[2], (unsigned)telemetry.duty_percent[3],
                 telemetry.reversing[0], telemetry.reversing[1], telemetry.reversing[2], telemetry.reversing[3]);
        return reply;
    }
    if (!strcmp(command,"stop")) return mecanum_drive_stop()==ESP_OK ? "ok\n" : "error: stop failed\n";
    if (!strcmp(command,"pid")) {
        float kp, ki;
        return mecanum_drive_get_pid_gains(&kp, &ki)==ESP_OK ?
            (snprintf(reply, reply_size, "pid kp %.4f ki %.4f\n", kp, ki), reply) : "error: pid unavailable\n";
    }
    float kp, ki; char pid_extra;
    if (sscanf(command,"pid %f %f %c",&kp,&ki,&pid_extra)==2)
        return mecanum_drive_set_pid_gains(kp,ki)==ESP_OK ?
            (snprintf(reply, reply_size, "pid kp %.4f ki %.4f\n", kp, ki), reply) : "error: invalid pid gains\n";
    unsigned motor; float wheel_speed; char wheel_extra;
    if (sscanf(command,"wheel %u %f %c",&motor,&wheel_speed,&wheel_extra)==2 && motor>=1 && motor<=4 && wheel_speed>=-1 && wheel_speed<=1)
        return mecanum_drive_set_wheel(s_physical_motor_map[motor-1],wheel_speed)==ESP_OK ? "ok\n" : "error: wheel failed\n";
    float f,s,t; char extra;
    if (sscanf(command,"drive %f %f %f %c",&f,&s,&t,&extra)==3 && f>=-1&&f<=1&&s>=-1&&s<=1&&t>=-1&&t<=1) return mecanum_drive_set_twist(f,s,t)==ESP_OK ? "ok\n" : "error: drive failed\n";
    return "error: pid [KP KI], wheel M SPEED, drive F S T, telemetry, or stop\n";
}
static void udp_task(void *unused) {
    (void)unused; int fd=socket(AF_INET,SOCK_DGRAM,IPPROTO_IP); if(fd<0){ESP_LOGE(TAG,"socket errno %d",errno);vTaskDelete(NULL);return;}
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(UDP_PORT),.sin_addr.s_addr=htonl(INADDR_ANY)};
    if(bind(fd,(struct sockaddr *)&address,sizeof(address))<0){ESP_LOGE(TAG,"bind errno %d",errno);vTaskDelete(NULL);return;}
    while(true){ char buffer[COMMAND_BUFFER_SIZE], reply[256]; struct sockaddr_in sender; socklen_t len=sizeof(sender); int received=recvfrom(fd,buffer,sizeof(buffer)-1,0,(struct sockaddr *)&sender,&len); if(received<0)continue; buffer[received]='\0'; const char *response=process_command(buffer,reply,sizeof(reply)); sendto(fd,response,strlen(response),0,(struct sockaddr *)&sender,len); }
}
static void telemetry_task(void *unused) {
    (void)unused; while(true) { mecanum_drive_telemetry_t t; if(mecanum_drive_get_telemetry(&t)==ESP_OK) ESP_LOGI(TAG,"rpm %.0f %.0f %.0f %.0f; duty %u %u %u %u; FG %u %u %u %u",t.measured_rpm[0],t.measured_rpm[1],t.measured_rpm[2],t.measured_rpm[3],(unsigned)t.duty_percent[0],(unsigned)t.duty_percent[1],(unsigned)t.duty_percent[2],(unsigned)t.duty_percent[3],(unsigned)t.encoder_edges[0],(unsigned)t.encoder_edges[1],(unsigned)t.encoder_edges[2],(unsigned)t.encoder_edges[3]); vTaskDelay(pdMS_TO_TICKS(1000)); }
}
void app_main(void) {
    esp_err_t err=nvs_flash_init(); if(err==ESP_ERR_NVS_NO_FREE_PAGES||err==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());err=nvs_flash_init();} ESP_ERROR_CHECK(err);
    mecanum_drive_config_t config={.wheels={
        // Physical connector map: M1 rear-right, M2 rear-left, M3 front-left, M4 front-right.
        [MECANUM_WHEEL_FRONT_LEFT]={GPIO_NUM_26,GPIO_NUM_27,GPIO_NUM_35,false}, // M3
        [MECANUM_WHEEL_FRONT_RIGHT]={GPIO_NUM_19,GPIO_NUM_13,GPIO_NUM_34,true}, // M4
        [MECANUM_WHEEL_REAR_LEFT]={GPIO_NUM_21,GPIO_NUM_25,GPIO_NUM_39,false}, // M2
        [MECANUM_WHEEL_REAR_RIGHT]={GPIO_NUM_23,GPIO_NUM_32,GPIO_NUM_36,true}, // M1
    },
        .encoder_pulses_per_output_rev=86.4f,.no_load_output_rpm=620,.rated_output_rpm=450,
        .control_period_ms=20,.command_timeout_ms=500,.max_duty_percent=100};
    ESP_ERROR_CHECK(mecanum_drive_init(&config)); start_wifi();
    xTaskCreate(udp_task,"udp_control",4096,NULL,5,NULL); xTaskCreate(telemetry_task,"telemetry",3072,NULL,3,NULL);
    ESP_LOGI(TAG,"chassis: 190 mm wheel-center square, 23 mm wheel radius");
}
