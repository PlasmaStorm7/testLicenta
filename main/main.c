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

// #include ""
#include "nmea_example.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

#define DISP_POWER_PIN GPIO_NUM_18

static const char* tag = "OGC";
#define PROMPT_STR "OGC"
#define MESSAGE_MAX_LENGTH 255
#define UUID_LENGTH 10
uint8_t buf[MESSAGE_MAX_LENGTH]={"salut\0"};
uint8_t prevBuf[MESSAGE_MAX_LENGTH]={0};
uint8_t ReceiveState=0;
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
	ReceiveState=0;
	lora_idle();
	//ESP_LOGI(tag,"message before truncating is %s",message);
	int len=strlen(message);
	if (len>MESSAGE_MAX_LENGTH){
		len=MESSAGE_MAX_LENGTH;
		message[MESSAGE_MAX_LENGTH]='\0';
		ESP_LOGI(tag,"message after truncating is %s",message);
	}
	
	lora_send_packet((uint8_t*) message,len);
	ESP_LOGI("LoRa","message %s sent",message);
	memcpy(prevBuf,message,len);
	lora_receive();
	ReceiveState=1;
}

int LoRa_send(int argc, char **argv){
    //ESP_LOGI(tag,"your message is: %s,length of the message is %d",argv[1],strlen(argv[1]));
	char message[MESSAGE_MAX_LENGTH]={0};
	strncat(message,UUIDstr,UUID_LENGTH);
	//ESP_LOGI("tag","UUIDstr copied into message=%s",message);
	strncat(message,argv[1],255-UUID_LENGTH);
	loraTransmitMessage(message);
    return 0;

}

esp_err_t esp_console_register_LoRa_send(void){
    esp_console_cmd_t command = {
        .command = "send",
        .help = "Transmit the specified message",
        .hint = "message",
        .func = &LoRa_send
    };
    return esp_console_cmd_register(&command);
}

int RxDoneCheck(int argc, char **argv){
	int RxDoneFlag=isRxDone();
	ESP_LOGI("LoRa","RxDoneFlag=%d",RxDoneFlag);
	ESP_LOGI("LoRa","ReceiveState=%d",ReceiveState);
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
	ESP_LOGI("LoRa","ReceiveState was: %d",ReceiveState);
	switch(argv[1][0])
	{
		case '0':
			ReceiveState=0;
			ESP_LOGI("LoRa","Receive state set to 0(off)");
			return 0;
		case '1':
			ReceiveState=1;
			ESP_LOGI("LoRa","Receive state set to 1(on)");
			return 0;
		default:
			ESP_LOGI("LoRa","Invalid state descriptor, state unchanged");
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

void RxTask(void *p){
	int packetLength;
	
	lora_receive();   
	for(;;) {
	while(1==ReceiveState){
		
	//ESP_LOGI("LoRa","checking for LoRa packet");
		while(isRxDone()) {
			packetLength = lora_receive_packet(buf, sizeof(buf));
			resetRxDone();
			if(packetLength==0){
				ESP_LOGI("LoRa","CRC Error with %.2f SNR and %d RSSI\n",lora_packet_snr(),lora_packet_rssi());
				lora_receive();
				break;
			}
			//msgReady=1;
			if(packetLength>254)
			packetLength=254;
			buf[packetLength+1] = 0;//string terminator
			ESP_LOGI("LoRa","Received:\"%s\" SNR:%.2f RSSI:%d", buf,lora_packet_snr(),lora_packet_rssi());
			// ESP_LOGI("LoRa","buf= \"%s\" prevbuf= \"%s\"",buf,prevBuf);
			if(0 == arraysAreIdentical(buf,prevBuf,packetLength)){
				ESP_LOGI("LoRa","Relaying  \"%s\" , length %d",buf,packetLength);
				lora_send_packet(buf,packetLength);
				memcpy(prevBuf,buf,sizeof(buf));
			}
			lora_receive();
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
	vTaskDelay(100/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

int gpsRead(){
    // Sentence string to be parsed
	// char sentence[] = "$GPGLL,4916.45,N,12311.12,W,225444,A,*1D\r\n";
	// char sentence[] = "$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30\r\n";
	// char sentence[] = "$GPTXT,01,03,02,u-blox ag - www.u-blox.com*50\r\n";
	// char sentence[] = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K\r\n";
	char sentence[] = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n";

	printf("Parsing NMEA sentence: %s", sentence);

	// Pointer to struct containing the parsed data. Should be freed manually.
	nmea_s *data;

	// Parse...
	data = nmea_parse(sentence, strlen(sentence), 0);

	if(NULL == data) {
		printf("Failed to parse sentence!\n");
		return -1;
	}

	if (NMEA_GPGLL == data->type) {
		nmea_gpgll_s *gpgll = (nmea_gpgll_s *) data;

		printf("GPGLL Sentence\n");
		printf("Longitude:\n");
		printf("  Degrees: %d\n", gpgll->longitude.degrees);
		printf("  Minutes: %f\n", gpgll->longitude.minutes);
		printf("  Cardinal: %c\n", (char) gpgll->longitude.cardinal);
		printf("Latitude:\n");
		printf("  Degrees: %d\n", gpgll->latitude.degrees);
		printf("  Minutes: %f\n", gpgll->latitude.minutes);
		printf("  Cardinal: %c\n", (char) gpgll->latitude.cardinal);
	}

	if (NMEA_GPGSA == data->type) {
		nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

		printf("GPGSA Sentence:\n");
		printf("\tMode: %c\n", gpgsa->mode);
		printf("\tFix:  %d\n", gpgsa->fixtype);
		printf("\tPDOP: %.2lf\n", gpgsa->pdop);
		printf("\tHDOP: %.2lf\n", gpgsa->hdop);
		printf("\tVDOP: %.2lf\n", gpgsa->vdop);
	}

	if (NMEA_GPVTG == data->type) {
		nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

		printf("GPVTG Sentence:\n");
		printf("\tTrack [deg]:   %.2lf\n", gpvtg->track_deg);
		printf("\tSpeed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
		printf("\tSpeed [knots]: %.2lf\n", gpvtg->gndspd_knots);
	}

	if (NMEA_GPTXT == data->type) {
		nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

		printf("GPTXT Sentence:\n");
		printf("\tID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
		printf("\t%s\n", gptxt->text);
	}

	if (NMEA_GPGSV == data->type) {
		nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

		printf("GPGSV Sentence:\n");
		printf("\tNum: %d\n", gpgsv->sentences);
		printf("\tID:  %d\n", gpgsv->sentence_number);
		printf("\tSV:  %d\n", gpgsv->satellites);
		printf("\t#1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
		printf("\t#2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
		printf("\t#3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
		printf("\t#4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
	}

	nmea_free(data);

	return 0;
}

void gpsTask(){
	while(1){
		vTaskDelay(1000/portTICK_PERIOD_MS);
		// ESP_LOGI("GPS","GPS Tasking");
	char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        nmea_example_read_line(&start, &length, 1000 /* ms */);
        if (length == 0) {
			ESP_LOGI("GPS","no read");
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }

            if (NMEA_GPGGA == data->type) {
                printf("GPGGA sentence\n");
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
                printf("Number of satellites: %d\n", gpgga->n_satellites);
                printf("Altitude: %f %c\n", gpgga->altitude,
                       gpgga->altitude_unit);
            }

            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
                printf("Time: %s\n", fmt_buf);
            }

            if (NMEA_GPRMC == data->type) {
                printf("GPRMC sentence\n");
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &pos->date_time);
                printf("Date & Time: %s\n", fmt_buf);
                printf("Speed, in Knots: %f\n", pos->gndspd_knots);
                printf("Track, in degrees: %f\n", pos->track_deg);
                printf("Magnetic Variation:\n");
                printf("  Degrees: %f\n", pos->magvar_deg);
                printf("  Cardinal: %c\n", (char) pos->magvar_cardinal);
                double adjusted_course = pos->track_deg;
                if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                    adjusted_course -= pos->magvar_deg;
                } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                    adjusted_course += pos->magvar_deg;
                } else {
                    printf("Invalid Magnetic Variation Direction!\n");
                }

                printf("Adjusted Track (heading): %f\n", adjusted_course);
            }

            if (NMEA_GPGSA == data->type) {
                nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

                printf("GPGSA Sentence:\n");
                printf("  Mode: %c\n", gpgsa->mode);
                printf("  Fix:  %d\n", gpgsa->fixtype);
                printf("  PDOP: %.2lf\n", gpgsa->pdop);
                printf("  HDOP: %.2lf\n", gpgsa->hdop);
                printf("  VDOP: %.2lf\n", gpgsa->vdop);
            }

            if (NMEA_GPGSV == data->type) {
                nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

                printf("GPGSV Sentence:\n");
                printf("  Num: %d\n", gpgsv->sentences);
                printf("  ID:  %d\n", gpgsv->sentence_number);
                printf("  SV:  %d\n", gpgsv->satellites);
                printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
                printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
                printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
                printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
            }

            if (NMEA_GPTXT == data->type) {
                nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

                printf("GPTXT Sentence:\n");
                printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
                printf("  %s\n", gptxt->text);
            }

            if (NMEA_GPVTG == data->type) {
                nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

                printf("GPVTG Sentence:\n");
                printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
                printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
                printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
            }

            nmea_free(data);
        }
		ESP_LOGI("GPS","we bee sleepy");
	vTaskDelay(1000/portTICK_PERIOD_MS);	
	}
	
}

void app_main(void)
{
	nmea_example_init_interface();


	uint8_t loraVersion=lora_init();
	ESP_LOGI("LoRa","LoRa Version is 0x%02x",loraVersion);
	lora_set_bandwidth(125E3);
	lora_set_frequency(8683e5);//868.3Mhz, bandwidth 125Khz ,banda libera up to 1% duty cycle
	lora_enable_crc();
	ReceiveState=0;

	gpio_set_direction(DISP_POWER_PIN,GPIO_MODE_OUTPUT);

	esp_base_mac_addr_get(baseMac);
	ESP_LOGI(tag,"MAC address is %02x:%02x:%02x:%02x:%02x:%02x",baseMac[0],baseMac[1],baseMac[2],baseMac[3],baseMac[4],baseMac[5]); 
	char hexMac[sizeof(baseMac)*2 + 1];
    bytes2hex(baseMac, hexMac, sizeof(baseMac));
    ESP_LOGI(tag,"Mac in hex is %s\n",hexMac);
	
	for(int i=0;i<3;i++){
		UUIDstr[3*i]=hexMac[2*i+6];
		UUIDstr[3*i+1]=hexMac[2*i+7];
		UUIDstr[3*i+2]=':';
	}
	UUIDstr[8]='>';
	UUIDstr[9]='\0';

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

	
	

	ESP_ERROR_CHECK(esp_console_start_repl(repl));
	xTaskCreate(&gpsTask, "task_gps", 4096, NULL,1, NULL);
	xTaskCreate(&RxTask, "task_rx", 4096, NULL,2, NULL);
    // xTaskCreate(&gpsTask,"GPS Task",2048,NULL,2,NULL);
	//xTaskCreate(&consoleTask,"Console Task",2048,repl,2,NULL);
	
}
