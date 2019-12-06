//
//  main.c
//  esp-homekit-rf-switches
//
//  Created by maccoylton on 13/05/2019.
//  Copyright © 2019 maccoylton. All rights reserved.
//

#define DEVICE_MANUFACTURER "MacCoylton"
#define DEVICE_NAME "RF Bridge"
#define DEVICE_MODEL "Basic"
#define DEVICE_SERIAL "12345678"
#define FW_VERSION "1.0"


#include <stdio.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


#include <homekit/homekit.h>
#include <homekit/characteristics.h>

//rf 433mhz
#include <rf_433mhz.h>


#include <button.h>
#include <led_codes.h>

// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include <ota-api.h>

homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t name         = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  DEVICE_MANUFACTURER);
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         DEVICE_MODEL);
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  FW_VERSION);



#define TRANSMITTER_PIN 5
#define RECIEVER_PIN 4

// The GPIO pin that is connected to the relay on the Sonoff Basic.
const int relay_gpio = 12;
// The GPIO pin that is connected to the LED on the Sonoff Basic.
const int LED_GPIO = 13;
// The GPIO pin that is oconnected to the button on the Sonoff Basic.
const int button_gpio = 0;

int button_pressed_value=0; /*set to o when botton is connect to gound, 1 when button is providing +3.3V */
int led_off_value=0; /*set to o when botton is connect to gound, 1 when LED is providing +3.3V */



reciever_433mhz* reciever;
QueueHandle_t decode_queue;

void rf433mhz_transmit(void *pvParameters) {
    gpio_enable(TRANSMITTER_PIN, GPIO_OUTPUT);
    message_433mhz msg;
    msg.code_lenght=24;
    msg.repeat=2;
    msg.pulse_length = 312;
    protocol_433mhz proto = protocols_433mhz[PROTO1];
    msg.protocol = &proto;
    
    while(true) {
        msg.data=1364;
        send_message_433mhz(TRANSMITTER_PIN, &msg);
        vTaskDelay(1000);
        msg.data=1361;
        send_message_433mhz(TRANSMITTER_PIN, &msg);
        vTaskDelay(1000);
    }
}

void recieve_interrupt_handler(uint8_t gpio_num) {
    BaseType_t xHigerPriorityTaskTriggerd = pdFALSE;
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    recieve_on_interrupt(reciever);
    if (reciever->repeatCount < 2) {
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    printf("Function recieve_interrupt_handler recieved going enter critical \n");
    reciever_433mhz* old_reciever = reciever;
    reciever = create_reciever_buffer();
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    xQueueSendFromISR(decode_queue, &old_reciever, &xHigerPriorityTaskTriggerd);
}

void rf433mhz_recieve(void *pvParameters) {
    //enable reception on recieve pin
    gpio_enable(RECIEVER_PIN, GPIO_INPUT);
    reciever = create_reciever_buffer();
    gpio_set_interrupt(RECIEVER_PIN, GPIO_INTTYPE_EDGE_ANY, recieve_interrupt_handler);
    
    reciever_433mhz* recv;
    message_433mhz msg;
    while(1) {
        if(xQueueReceive(decode_queue, &recv, 0) == pdTRUE){
            //try to decode recieved data
            bool recieved = false;
            for(int i=0; i < PROTOCOL_COUNT ; i++) {
                if(decode_recieved(recv, &protocols_433mhz[i], &msg)) {
                    recieved = true;
                    printf("Recieved message with data: %d\n", msg.data);
                    break;
                } else {
                    printf("Recieved message no protcol found to decode data\n");
                }
                //no protcol found to decode data
            }
            free(recv);
        }
        vTaskDelay(10);
    }
}




void switch_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context);

void button_callback(uint8_t gpio, button_event_t event);

void relay_write(bool on) {
    //gpio_write(relay_gpio, on ? 1 : 0);
}

void led_write(bool on) {
    gpio_write(LED_GPIO, on ? 0 : 1);
}

void reset_configuration_task() {
    //Flash the LED first before we start the reset
    led_code (LED_GPIO, WIFI_CONFIG_RESET);
    printf("Resetting Wifi Config\n");
    
    //    wifi_config_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Resetting HomeKit Config\n");
    
    homekit_server_reset();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("Restarting\n");
    
    sdk_system_restart();
    
    vTaskDelete(NULL);
}

void reset_configuration() {
    printf("Resetting Sonoff configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

homekit_characteristic_t switch_on = HOMEKIT_CHARACTERISTIC_(
                                                             ON, false, .callback=HOMEKIT_CHARACTERISTIC_CALLBACK(switch_on_callback)
                                                             );

void gpio_init() {
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    led_write(false);
    gpio_enable(relay_gpio, GPIO_OUTPUT);
    relay_write(switch_on.value.bool_value);
}

void switch_on_callback(homekit_characteristic_t *_ch, homekit_value_t on, void *context) {
    relay_write(switch_on.value.bool_value);
    led_write(switch_on.value.bool_value);
}

void button_callback(uint8_t gpio, button_event_t event) {
    switch (event) {
        case button_event_single_press:
            printf("Toggling relay\n");
            switch_on.value.bool_value = !switch_on.value.bool_value;
            relay_write(switch_on.value.bool_value);
            led_write(switch_on.value.bool_value);
            homekit_characteristic_notify(&switch_on, switch_on.value);
            break;
        case button_event_long_press:
            reset_configuration();
            break;
        default:
            printf("Unknown button event: %d\n", event);
    }
}

void switch_identify_task(void *_args) {
    // We identify the Sonoff by Flashing it's LED.
    led_code( LED_GPIO, IDENTIFY_ACCESSORY);
    led_write(switch_on.value.bool_value);
    vTaskDelete(NULL);
}

void switch_identify(homekit_value_t _value) {
    printf("Switch identify\n");
    xTaskCreate(switch_identify_task, "Switch identify", 128, NULL, 2, NULL);
}


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_switch, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, switch_identify),
            NULL
        }),
        HOMEKIT_SERVICE(SWITCH, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Switch"),
            &switch_on,
            &ota_trigger,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};

void create_accessory_name() {
    
    int serialLength = snprintf(NULL, 0, "%d", sdk_system_get_chip_id());
    
    char *serialNumberValue = malloc(serialLength + 1);
    
    snprintf(serialNumberValue, serialLength + 1, "%d", sdk_system_get_chip_id());
    
    int name_len = snprintf(NULL, 0, "%s-%s-%s",
                            DEVICE_NAME,
                            DEVICE_MODEL,
                            serialNumberValue);
    
    if (name_len > 63) {
        name_len = 63;
    }
    
    char *name_value = malloc(name_len + 1);
    
    snprintf(name_value, name_len + 1, "%s-%s-%s",
             DEVICE_NAME, DEVICE_MODEL, serialNumberValue);
    
    
    name.value = HOMEKIT_STRING(name_value);
    serial.value = name.value;
}



void user_init(void) {
    uart_set_baud(0, 115200);
    decode_queue = xQueueCreate(2, sizeof(reciever_433mhz*));
    
    //start tasks
    xTaskCreate(rf433mhz_transmit, "transmitter_rf", 256, NULL, 1, NULL);
    xTaskCreate(rf433mhz_recieve, "reciever_rf", 256, NULL, 1, NULL);
    
    gpio_init();
    
    create_accessory_name();
    
    if (button_create(button_gpio, 0, 4000, button_callback)) {
        printf("Failed to initialize button\n");
    }
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                 &model.value.string_value,&revision.value.string_value);
    if (c_hash==0) c_hash=1;
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);
}
