#include <stdio.h>
#include "lora.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "log.h"

#include <string.h>
#include <stdlib.h>
#include "nmea.h"
#include "gpgll.h"
#include <gpgsa.h>
#include <gpvtg.h>
#include <gptxt.h>
#include <gpgsv.h>

int gps(){
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

void initLoRa(){
    int current_spi_host=SPI2_HOST;
    /**
    * Configure SPI bus and device
    */
    spi_bus_config_t bus = {
        .miso_io_num = CONFIG_LORA_MISO_GPIO,
        .mosi_io_num = CONFIG_LORA_MOSI_GPIO,
        .sclk_io_num = CONFIG_LORA_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    esp_err_t ret = spi_bus_initialize(current_spi_host, &bus, 0);
    assert(ret == ESP_OK);
    lora_init(current_spi_host);
    ESP_LOGI("LoRa","LoRa initialization succesful");
    setDIO(DIO0RxDone,DIO1RxTimeout,DIO2Fhss,DIO3ValidHeader,DIO4CadDetected,DIO5ModeReady);
}

void app_main(void)
{

    // gps();
    initLoRa();
}
