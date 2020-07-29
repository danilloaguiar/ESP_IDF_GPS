/* UART Events Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"



/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 * 
 *  
Char time [2-7] 
Char Late [12-15 e 17-21]
Char Long [25-29 e 31-35]
Char Alti [49-50]
 * 
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (2)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BYTES_TO_READ (68)
#define BUF_SIZE (512)
#define RD_BUF_SIZE (BUF_SIZE)

#define TIME_SIZE (6)
#define TIME_POS (2)
#define LAT0_SIZE (4)
#define LAT0_POS (12)
#define LAT1_SIZE (5)
#define LAT1_POS (17)
#define LON0_SIZE (5)
#define LON0_POS (25)
#define LON1_SIZE (5)
#define LON1_POS (31)
#define ALT_SIZE (2)
#define ALT_POS (49)


typedef struct {
	uint32_t time;
	int longitude0;
	int latitude0;
    int longitude1;
	int latitude1;
	int n_satellites;
	int altitude;
	char altitude_unit;
} nmea_gpgga_s;

static QueueHandle_t uart0_queue;
static const char *TAG = "uart_events";
static const char *TAG1 = "GPS_infor";
static nmea_gpgga_s exemplo;

void init_load(void){
    exemplo.time=0;
    exemplo.longitude0=0;
    exemplo.longitude1=0;
    exemplo.latitude0=0;
    exemplo.latitude1=0;
    exemplo.altitude=0;
}

int converter(int size, int pos, uint8_t* data){
    int base;
    int aux0[size];
    int aux1=0;

    for(int i = 0;i < size;i++){
        aux0[i] = data[pos+i] - '0';
    }
    for(int j = 0; j < size; j++){
       base = pow(10, size - j -1);
       aux1 += aux0[j]*base;
    }
     
    return aux1;
}

void load_value(uint8_t* data){

    exemplo.time = converter(TIME_SIZE, TIME_POS, data);
    ESP_LOGI(TAG1, "UTC Time: %d ", exemplo.time);

    exemplo.latitude0 = converter(LAT0_SIZE, LAT0_POS, data);
    exemplo.latitude1 = converter(LAT1_SIZE, LAT1_POS, data);
    ESP_LOGI(TAG1, "Latitude: %d.%d", exemplo.latitude0, exemplo.latitude1);      
    
    exemplo.longitude0 = converter(LON0_SIZE, LON0_POS, data);
    exemplo.longitude1 = converter(LON1_SIZE, LON1_POS, data);
    ESP_LOGI(TAG1, "Longitude: %d.%d", exemplo.longitude0, exemplo.longitude1); 

    exemplo.altitude = converter(ALT_SIZE, ALT_POS, data);
    ESP_LOGI(TAG1, "Altitude: %d \n\n\n", exemplo.altitude);     

    init_load();
}



void get_data(uint8_t* data) {
    int n = uart_read_bytes(EX_UART_NUM, data, BYTES_TO_READ, (BYTES_TO_READ + 20) / portTICK_PERIOD_MS);
    if (n >= 0) {
        data[n] = 0;
        ESP_LOGI(TAG, "read DATA: %s \n", data);
        load_value(data);
    } else {
        ESP_LOGI(TAG, "read DATA: FAILED");
    }

}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    uint8_t* data = (uint8_t*) malloc(BYTES_TO_READ);

    

    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "[DATA EVT]:");
                    /*
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    */
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    //ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        /*
                        There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        record the position. We should set a larger queue size.
                        As an example, we directly flush the rx buffer here.
                        */
                        ESP_LOGE(TAG, "read pat failed");
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 1 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 10 / portTICK_PERIOD_MS);
                        /*
                        ESP_LOGI(TAG, "read pat : %s \n", pat);
                        ESP_LOGI(TAG, "read dtmp: %s \n", dtmp);
                        */
                        vTaskDelay(80 / portTICK_RATE_MS);
                        get_data(data);
                    } 
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    free(data);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 64, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr(EX_UART_NUM, 'G', PATTERN_CHR_NUM, 60000, 0, 0);
    //Reset the pattern queue length to record at most 16 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 16);

    init_load();

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 8192, NULL, 6, NULL);
}
