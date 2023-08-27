#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lora.h"
#include "driver/gpio.h"
// #include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"
#include "cmd_nvs.h"
#include "cmd_wifi.h"

#include "nmea_parser.h"


// #include "nmea_example.h"
// #include "nmea.h"
// #include "gpgll.h"
// #include "gpgga.h"
// #include "gprmc.h"
// #include "gpgsa.h"
// #include "gpvtg.h"
// #include "gptxt.h"
// #include "gpgsv.h"

#define DISP_POWER_PIN GPIO_NUM_18

static const char* tag = "OGC";
static const char* gpsTag = "GPS";
static const char* loraTag = "LoRa";

#define TIME_ZONE (+3)   //EEST
#define YEAR_BASE (2000) //date in GPS starts from 2000
nmea_parser_handle_t nmea_hdl;
gps_t gpsData;
uint8_t gpsState=0;
#define PROMPT_STR "OGC"
#define MESSAGE_MAX_LENGTH 255
#define UUID_LENGTH 7
uint8_t buf[MESSAGE_MAX_LENGTH]={"salut\0"};
uint8_t prevBuf[MESSAGE_MAX_LENGTH]={0};
uint8_t receiveState=1;
uint8_t baseMac[6];
char hexMac[sizeof(baseMac)*2 + 1];
char HexLookUp[] = "0123456789ABCDEF";
char UUIDstr[UUID_LENGTH];
/* Console command history can be stored to and loaded from a file.
 * The easiest way to do this is to use FATFS filesystem on top of
 * wear_levelling library.
 */
#if CONFIG_EXAMPLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"
static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void IRAM_ATTR DIO0_isr_handler(void* arg)
{
   RxDoneFlag=1;
   
   //printf("\nLoRa DIO0 triggered\n");
}

/**
 * Compares 2 uint8_t arrays and returns 1 if they are the same and 0 if they are different
*/
int arraysAreIdentical(uint8_t *array1,uint8_t *array2,int length){
	for(int i=0;i<length;i++){
		if(array1[i]!=array2[i])
		return 0;
	}
	return 1;
}

void bytes2hex (unsigned char *src, char *out, int len)
{
    while(len--)
    {
        *out++ = HexLookUp[*src >> 4];
        *out++ = HexLookUp[*src & 0x0F];
        src++;
    }
    *out = 0;
}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	
    gps_t *gpsP = NULL;
    switch (event_id) {
    case GPS_UPDATE:
        gpsP = (gps_t *)event_data;
        /* print information parsed from GPS statements */
		// ESP_LOGI(gpsTag, "%d/%d/%d %d:%d:%d => \r\n"
        //          "latitude   = %.05f°N\r\n"
        //          "longitude = %.05f°E\r\n"
        //          "altitude   = %.02fm\r\n"
        //          "speed      = %fm/s\n"
		// 		 "validity=%d fix=%d",
        //          gpsP->date.year + YEAR_BASE, gpsP->date.month, gpsP->date.day,
        //          gpsP->tim.hour + TIME_ZONE, gpsP->tim.minute, gpsP->tim.second,
        //          gpsP->latitude, gpsP->longitude, gpsP->altitude, gpsP->speed,
		// 		 gpsP->valid, gpsP->fix);
		if(gpsP->valid){
			gpsData=*gpsP;
		}else
			gpsData.valid=false;
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(gpsTag, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}
void loraRead(){
	uint8_t buf[512];
	lora_idle();
	ESP_LOGI(tag,"lora receive mode");
	lora_receive();
	while(true){
		if(lora_received() != 0)
		{
			lora_receive_packet(buf,512);
			printf((char*)buf);
			printf("\n");
		}else{
			ESP_LOGI(tag,"nothing received");	
		}
		vTaskDelay(pdMS_TO_TICKS(1500));
	}
}

void loraTransmitMessage(char* message){
	// uint8_t prevReceiveState=ReceiveState;
	receiveState=0;
	lora_idle();
	//ESP_LOGI(tag,"message before truncating is %s",message);
	int len=strlen(message);
	if (len>MESSAGE_MAX_LENGTH){
		len=MESSAGE_MAX_LENGTH;
		message[MESSAGE_MAX_LENGTH]='\0';
		ESP_LOGI(tag,"message after truncating is %s",message);
	}
	
	lora_send_packet((uint8_t*) message,len);
	ESP_LOGI(loraTag,"message %s sent",message);
	memcpy(prevBuf,message,len);
	lora_receive();
	receiveState=1;
}

int LoRa_send(int argc, char **argv){
    //ESP_LOGI(tag,"your message is: %s,length of the message is %d",argv[1],strlen(argv[1]));
	char message[MESSAGE_MAX_LENGTH]={0};
	strncat(message,UUIDstr,UUID_LENGTH);
	//ESP_LOGI("tag","UUIDstr copied into message=%s",message);
	strncat(message,argv[1],UUID_LENGTH);
	strncat(message,argv[2],255-(2*UUID_LENGTH));
	loraTransmitMessage(message);
	ESP_LOGI(gpsTag, "%d/%d/%d %d:%d:%d => \r\n"
                 "latitude   = %.05f°N\r\n"
                 "longitude = %.05f°E\r\n"
                 "altitude   = %.02fm\r\n"
                 "speed      = %fm/s\n"
				 "validity=%d fix=%d",
                 gpsData.date.year + YEAR_BASE, gpsData.date.month, gpsData.date.day,
                 gpsData.tim.hour + TIME_ZONE, gpsData.tim.minute, gpsData.tim.second,
                 gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.speed,
				 gpsData.valid, gpsData.fix);
    return 0;

}

esp_err_t esp_console_register_LoRa_send(void){
    esp_console_cmd_t command = {
        .command = "send",
        .help = "Broadcast the message and specify destination",
        .hint = "[destination UUIDstr] [message]",
        .func = &LoRa_send
    };
    return esp_console_cmd_register(&command);
}

int RxDoneCheck(int argc, char **argv){
	//int RxDoneFlag=isRxDone();
	ESP_LOGI(loraTag,"RxDoneFlag=%d",RxDoneFlag);
	ESP_LOGI(loraTag,"ReceiveState=%d",receiveState);
	return RxDoneFlag;
}

esp_err_t register_LoRa_RxDoneCheck(){
	esp_console_cmd_t command = {
        .command = "checkRx",
        .help = "Check the RxDoneFlag",
        .hint = "",
        .func = &RxDoneCheck
    };
    return esp_console_cmd_register(&command);
}

int setReceive(int argc, char **argv){
	ESP_LOGI(loraTag,"ReceiveState was: %d",receiveState);
	switch(argv[1][0])
	{
		case '0':
			receiveState=0;
			ESP_LOGI(loraTag,"Receive state set to 0(off)");
			return 0;
		case '1':
			receiveState=1;
			ESP_LOGI(loraTag,"Receive state set to 1(on)");
			return 0;
		default:
			ESP_LOGI(loraTag,"Invalid state descriptor, state unchanged");
			return -1;
	}
}

esp_err_t register_LoRa_SetReceiveState(){
	esp_console_cmd_t command = {
        .command = "set_receive",
        .help = "Set the receive state on or off",
        .hint = "[0|1]",
        .func = &setReceive
    };
    return esp_console_cmd_register(&command);
}

int setGPS(int argc, char **argv){
	ESP_LOGI(gpsTag,"GpsState was: %d",gpsState);
	switch(argv[1][0])
	{
		case '0':
			if(0!=gpsState){
				nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
				gpsState=0;
				ESP_LOGI(gpsTag,"gpsState set to 0(off)");
			}else{
				ESP_LOGW(gpsTag,"GPS was already off(0)");
			}
			return 0;
		case '1':
			if(1!=gpsState){
				nmea_parser_add_handler(nmea_hdl, gps_event_handler,NULL);
				gpsState=1;
				ESP_LOGI(gpsTag,"gpsState set to 1(on)");
			}else{
				ESP_LOGW(gpsTag,"GPS was already on(1)");	
			}
			return 0;
		default:
			ESP_LOGI(gpsTag,"Invalid state descriptor, state unchanged");
			return -1;
	}
}
esp_err_t register_GPS_SetGpsState(){
	esp_console_cmd_t command = {
        .command = "gps_state",
        .help = "Set the GPS state on or off",
        .hint = "[0|1]",
        .func = &setGPS
    };
    return esp_console_cmd_register(&command);
}

void RxTask(void *p){
	int packetLength;
	char* responseText="Received your message";
	uint8_t responseLen=strlen(responseText);
	char response[responseLen+UUID_LENGTH];
	strncpy(response,UUIDstr,UUID_LENGTH);
	strncat(response,responseText,responseLen+1);
	responseLen+= UUID_LENGTH;
	lora_receive();   
	for(;;) {
	while(1==receiveState){
		
	//ESP_LOGI(loraTag,"checking for LoRa packet");
		while(isRxDone()) {
			packetLength = lora_receive_packet(buf, sizeof(buf));
			resetRxDone();
			if(packetLength==0){
				ESP_LOGI(loraTag,"CRC Error with %.2f SNR and %d RSSI\n",lora_packet_snr(),lora_packet_rssi());
				lora_receive();
				break;
			}
			//msgReady=1;
			if(packetLength>254)
			packetLength=254;
			buf[packetLength] = 0;//string terminator
			ESP_LOGI(loraTag,"Received:\"%s\" SNR:%.2f RSSI:%d", buf,lora_packet_snr(),lora_packet_rssi());
			// ESP_LOGI(loraTag,"buf= \"%s\" prevbuf= \"%s\"",buf,prevBuf);
			if(1 == arraysAreIdentical(buf+UUID_LENGTH,(uint8_t*)UUIDstr,UUID_LENGTH))
			{
				ESP_LOGI(loraTag,"Received a message destined for this device\nRelaying a response");
				lora_send_packet((uint8_t*)response,responseLen);
			}else if(0 == arraysAreIdentical(buf,prevBuf,packetLength) && buf[6]=='>'){
				ESP_LOGI(loraTag,"Relaying  \"%s\" , length %d",buf,packetLength);
				lora_send_packet(buf,packetLength);
				memcpy(prevBuf,buf,sizeof(buf));
			}
			lora_receive();
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
	vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


void gpsTask(){
	
}

void app_main(void)
{
	


	uint8_t loraVersion=lora_init();
	ESP_LOGI(loraTag,"LoRa Version is 0x%02x",loraVersion);
	lora_set_bandwidth(125E3);
	//lora_set_frequency(8683e5);//868.3Mhz, bandwidth 125Khz ,banda libera up to 1% duty cycle
	lora_set_frequency(869525e3);//869.4-869.65 BW 125kHz banda de downlink max 27dB si 10% DC
	lora_set_tx_power(17);//ONLY USE 17 IF IN THE 868.525 BAND!!! 14 otherwise
	lora_enable_crc();
	lora_set_spreading_factor(7);
	// lora_set_sync_word(35);
	
	gpio_isr_handler_add(CONFIG_LORA_DIO0_GPIO, DIO0_isr_handler, (void*) CONFIG_LORA_DIO0_GPIO);

	gpio_set_direction(DISP_POWER_PIN,GPIO_MODE_OUTPUT);
	gpio_set_level(DISP_POWER_PIN,1);
	esp_base_mac_addr_get(baseMac);
	ESP_LOGI(tag,"MAC address is %02x:%02x:%02x:%02x:%02x:%02x",baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5]); 
	char hexMac[sizeof(baseMac)*2 + 1];
    bytes2hex(baseMac, hexMac, sizeof(baseMac));
    ESP_LOGI(tag,"Mac in hex is %s\n",hexMac);
	
	for(int i=0;i<3;i++){
		UUIDstr[2*i]=hexMac[2*i+6];
		UUIDstr[2*i+1]=hexMac[2*i+7];
	}
	UUIDstr[6]='>';
	//UUIDstr[7]='\0';

	esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = UUIDstr;
    repl_config.max_cmdline_length = CONFIG_EXAMPLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

#if CONFIG_EXAMPLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(tag, "Command history enabled");
#else
    ESP_LOGI(tag, "Command history disabled");
#endif

    /* Register commands */
    esp_console_register_help_command();
    esp_console_register_LoRa_send();
	register_LoRa_RxDoneCheck();
	register_LoRa_SetReceiveState();
	register_GPS_SetGpsState();
    register_system();
    register_wifi();
    register_nvs();

	//vTaskDelay(100/portTICK_PERIOD_MS);
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif
	//gps defaults
	// gpsData.latitude=(float)46.77129;
	// gpsData.longitude=(float)23.6239;
	// gpsData.fix=GPS_FIX_INVALID;
	// gpsData.date.year=23;
	// gpsData.date.month=9;
	// gpsData.date.day=33;
	// gpsData.valid=false;
	ESP_LOGI(gpsTag, "%d/%d/%d %d:%d:%d => \r\n"
                 "latitude   = %.05f°N\r\n"
                 "longitude = %.05f°E\r\n"
                 "altitude   = %.02fm\r\n"
                 "speed      = %fm/s\n"
				 "validity=%d fix=%d\n ",
                 gpsData.date.year + YEAR_BASE, gpsData.date.month, gpsData.date.day,
                 gpsData.tim.hour + TIME_ZONE, gpsData.tim.minute, gpsData.tim.second,
                 gpsData.latitude, gpsData.longitude, gpsData.altitude, gpsData.speed,
				 gpsData.valid, gpsData.fix);
	

	
	 /* NMEA parser configuration */
    nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
    /* init NMEA parser library */
    nmea_hdl = nmea_parser_init(&config);
    /* register event handler for NMEA parser library */
	if(1==gpsState){
    	nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);
	}
	xTaskCreate(&RxTask, "task_rx", 4096, NULL,2, NULL);
	ESP_ERROR_CHECK(esp_console_start_repl(repl));
    // xTaskCreate(&gpsTask,"GPS Task",2048,NULL,2,NULL);
	//xTaskCreate(&consoleTask,"Console Task",2048,repl,2,NULL);
	
}
