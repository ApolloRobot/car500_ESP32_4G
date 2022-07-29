/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"



#define BLINK_LED_PIN GPIO_NUM_10   //车灯连接esp32的io10
#define MOTOR_IN1 GPIO_NUM_0   //7A大电流驱动板，in1接口，连接esp32的io0
#define MOTOR_IN2 GPIO_NUM_1   //7A大电流驱动板，in2接口，连接esp32的io1
#define BLINK_LED_BLINK 500

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_SERVO_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (2) // Define the output GPIO电机调速PWM口ENA，连接ESP32的io2
#define SERVO_OUTPUT_IO          (3) // Define the output GPIO舵机接口，调整方向使用，舵机信号口连接ESP32的io3
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define SERVO_CHANNEL            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_SERVO_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits 分辨率2的13次方
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_SERVO_DUTY               (532) // Set duty to 5%. ((2 ** 13) - 1) * 7.5% = 307.2(理论值，测试时以实际情况为准，该值需要自己调试)  0.5ms-2.5ms舵机调整范围  pwm波 20ms周期，2.5%-12.5%占空比 调整0-180度 中间值为1.5ms，占空比为7.5%  409分辨率
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz  电机pwm频率
#define LEDC_SERVO_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 50Hz  舵机pwm波频率

static const char *TAG = "MQTT_EXAMPLE";
static char mqtt_msg[512];

//mqtt接收消息缓冲区
static float initspeed=0.5;
//小车初始速度系数
static int ledstatus=0;
//车灯状态
static int x=128;
static int y=128;
//获取方向盘油门数值
static int xangle=128;
static int yangle=128;
//方向盘油门中值

static int pulseWithmidle = 532;
// static int freq = 50;      // 频率(20ms周期)
// static int channel = 8;    // 
// static int resolution = 8; // 分辨率
/*
如果 ESP32-C3 的定时器选用了RTCxM_CLK作为其时钟源，驱动会通过内部校准来得知这个时钟源的实际频率。这样确保了输出PWM信号频率的精准性。

ESP32-C3 的所有定时器共用一个时钟源。因此 ESP32-C3 不支持给不同的定时器配置不同的时钟源。
ESP32-C3 LEDC 时钟源特性

时钟名称      时钟频率      时钟功能
APB_CLK      80 MHz       /

RTC20M_CLK   ~20 MHz      支持动态调频（DFS）功能，支持Light-sleep模式

XTAL_CLK     40 MHz       支持动态调频（DFS）功能

*/



static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

char *mystrncpy(const char *string,int n){//要求截取的字符串不可以改变，但指向字符串的指针可以改变
char *p=string;
    if(p==NULL){//如果截取的字符串是空的直接返回
        return NULL;
    }else{
        int i=0;
    while(*p!='\0'){//循环直到达n个字符串终止
    if(i==n){
        break;
    }
    i++;
    p++;
    }
    *(p++)='\0';//赋值结束字符串
    return string;
    }
}
//电机pwm初始化
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
//舵机pwm初始化
static void example_servo_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t servo_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_SERVO_TIMER,
        .duty_resolution  = LEDC_SERVO_DUTY_RES,    //2的13次方分辨率
        .freq_hz          = LEDC_SERVO_FREQUENCY,  // Set output frequency at 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t servo_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = LEDC_SERVO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_OUTPUT_IO ,
        .duty           = LEDC_SERVO_DUTY, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED://mqtt连接事件
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/ESP32_8986XXXXXXXXXXXXXXX", "connected", 0, 1, 0);//将8986XXXXXXXXXXXXXXX替换为你的物联网卡ICCID，19位数字，和方向盘的需要一致
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        // esp_mqtt_client_subscribe(client, "/car_8986062018005772874", 1);
        msg_id = esp_mqtt_client_subscribe(client, "/car_8986XXXXXXXXXXXXXXX", 1);////将8986XXXXXXXXXXXXXXX替换为你的物联网卡ICCID，19位数字，和方向盘的需要一致
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED://mqtt断开事件
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        // msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA://mqtt接收数据事件
        // ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        // printf("DATA=%.*s\r\n", event->data_len, event->data);
        // printf(ty)
        // printf((event->data).toString());
        // char orderstr[event->data_len];
        // strcpy(orderstr,event->data);
        if (event->data_len >= (sizeof(mqtt_msg) - 1))
        {
            ESP_LOGE(TAG, "Received MQTT message size [%d] more than expected [%d]", event->data_len, (sizeof(mqtt_msg) - 1));
            return ESP_FAIL;
        }


        // if(event->data_len>1){
        //     // printf(strlen(event->data));
        //     // printf("DATA=%.*s\r\n",event->data);
        // }
        // char orderstr=event->data;
        // printf(orderstr);
        strcpy(mqtt_msg, mystrncpy(event->data,event->data_len));
        // printf(mqtt_msg);
        // printf("\r\n");
        if(strcmp(mqtt_msg,"start")==0){
            // printf("turn on or off led\r\n");
            if(ledstatus){
                //开灯
                gpio_set_level(BLINK_LED_PIN, 0);
                ledstatus = 0;
                return;
            }else{
                //关灯
                gpio_set_level(BLINK_LED_PIN, 1);
                ledstatus = 1;
                return;
            }
        }else if (strcmp(mqtt_msg,"one")==0)
        {
            initspeed =1;
            return;
        }else if (strcmp(mqtt_msg,"half")==0)
        {
            initspeed=0.5;
            return;
        }else if (strcmp(mqtt_msg,"halfhalf")==0)
        {
            initspeed=0.25;
            return;
        }
        cJSON* cjson = cJSON_Parse(mqtt_msg);//将JSON字符串转换成JSON结构体
        if(cjson == NULL)						//判断转换是否成功
        {
                printf("cjson error...\r\n");
        }
        else
        {
                // printf("%s\n",cJSON_Print(cjson));//打包成功调用cJSON_Print打印输出
        }
            
        // printf("/*********************以下就是提取的数据**********************/\n");
            x = cJSON_GetObjectItem(cjson,"x")->valueint;	//解析字符串
            // printf("x--->>%d\n",x);
            y = cJSON_GetObjectItem(cjson,"y")->valueint;	//解析字符串
            // printf("y--->>>%d\n",y);
            // char *name = cJSON_GetObjectItem(cjson,"name")->valuestring;	//解析字符串
	        // printf("%s\n",name);
            // int age = cJSON_GetObjectItem(cjson,"age")->valueint;			//解析整型
            // printf("%d\n",age);
            // double height = cJSON_GetObjectItem(cjson,"height")->valuedouble;	//解析双浮点型
            // printf("%.1f\n",height);
            // int gender = cJSON_GetObjectItem(cjson,"gender")->type; 	//解析逻辑值---输出逻辑值对应的宏定义数值
            // printf("%d\n",gender);

            cJSON_Delete(cjson);//清除结构体 
            if(xangle-x>0){
                //turn left左转
                // en2.servoWrite(parseInt(pulseWithmidle+(xangle-JSON.parse(message.toString()).x)*3.125))
                    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL,pulseWithmidle+(xangle-x)*3.125));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
                }
                if(x-xangle>0){
                //turn right右转
                // en2.servoWrite(parseInt(pulseWithmidle-(JSON.parse(message.toString()).x-xangle)*3.125))
                 ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL,pulseWithmidle-(x-xangle)*3.125));
                    // Update duty to apply the new value
                    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
                }
                if(x-xangle==0){
                
                // console.log('midllle-->>',JSON.parse(message.toString()).x-xangle)
                // en2.servoWrite(pulseWithmidle)
                // Set duty to 2.5%
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, pulseWithmidle));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
                }
                if(yangle-y>0){
                //油门
                // en1.write((yangle-JSON.parse(message.toString()).y)*initspeed/128)
                // in1.writeSync(1)
                // in2.writeSync(0)
                // in1.digitalWrite(1)
                // in2.digitalWrite(0)
                gpio_set_level(MOTOR_IN1, 1);
                gpio_set_level(MOTOR_IN2, 0);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (yangle-y)*initspeed*64));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                // console.log('y left-->>',yangle-JSON.parse(message.toString()).y)
                return;
                }
                if(y-yangle>0){
                //油门
                // en1.write((JSON.parse(message.toString()).y-yangle)*initspeed/128)
                // in1.writeSync(0)
                // in2.writeSync(1)
                // in1.digitalWrite(0)
                // in2.digitalWrite(1)
                gpio_set_level(MOTOR_IN1, 0);
                gpio_set_level(MOTOR_IN2, 1);
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, (y-yangle)*initspeed*64));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                // console.log('y right-->>',JSON.parse(message.toString()).y-yangle)
                return;
                }
                if(y-yangle==0){
                //停止
                // in1.writeSync(1)
                // in2.writeSync(1)
                // in1.digitalWrite(1)
                // in2.digitalWrite(1)
                // console.log('stop-->>',JSON.parse(message.toString()).y-yangle)
                gpio_set_level(MOTOR_IN1, 1);
                gpio_set_level(MOTOR_IN2, 1);
                return;
            }
            return 0;
        
        
        
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    // static esp_mqtt_client_config_t    mqtt_cfg = {
    //         .host= IOT_CORE_MQTT_BROKER_URL,
    //         .event_handle = mqtt_event_handler,//注册回调函数
    //         .port = 1883,
    //         .username = mqtt_token,
    //         .client_id = my_clinet_id
    //         };
//定义并初始化MQTT Client配置结构体
//client_id 默认使用的是ESP32_%CHIPID%的形式；

#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    //启动客户端，连接服务器
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());


    //配置输出引脚10灯光
    //电机控制io口0,1
        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.21
        io_conf.pin_bit_mask = 1ULL << BLINK_LED_PIN;
        //disable pull-down mode
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        //disable pull-up mode
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        //configure GPIO with the given settings
        gpio_config(&io_conf);
        gpio_config_t io_conf_motor1 = {};
        //disable interrupt
        io_conf_motor1.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf_motor1.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.21
        io_conf_motor1.pin_bit_mask = 1ULL << MOTOR_IN1;
        //disable pull-down mode
        io_conf_motor1.pull_down_en = GPIO_PULLDOWN_DISABLE;
        //disable pull-up mode
        io_conf_motor1.pull_up_en = GPIO_PULLUP_ENABLE;
        //configure GPIO with the given settings
        gpio_config(&io_conf_motor1);
        gpio_config_t io_conf_motor2 = {};
        //disable interrupt
        io_conf_motor2.intr_type = GPIO_INTR_DISABLE;
        //set as output mode
        io_conf_motor2.mode = GPIO_MODE_OUTPUT;
        //bit mask of the pins that you want to set,e.g.21
        io_conf_motor2.pin_bit_mask = 1ULL << MOTOR_IN2;
        //disable pull-down mode
        io_conf_motor2.pull_down_en = GPIO_PULLDOWN_DISABLE;
        //disable pull-up mode
        io_conf_motor2.pull_up_en = GPIO_PULLUP_ENABLE;
        //configure GPIO with the given settings
        gpio_config(&io_conf_motor2);

// Set the LEDC peripheral configuration
    example_ledc_init();
    example_servo_init();
    // Set duty to 50%
    // vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

     // Set duty to 5%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, SERVO_CHANNEL, LEDC_SERVO_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, SERVO_CHANNEL));
    ESP_LOGI(TAG, "[APP] get freq--->>>..%d",ledc_get_freq(LEDC_MODE,LEDC_SERVO_TIMER));
    
    mqtt_app_start();
}
