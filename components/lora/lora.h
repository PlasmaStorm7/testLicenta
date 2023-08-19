
#ifndef __LORA_H__
#define __LORA_H__

// #include <stdbool.h>

//DIO mapping enums

enum DIO0modeEnum {DIO0RxDone=0,DIO0Txdone=1,DIO0CadDone=2};
enum DIO1modeEnum {DIO1RxTimeout=0,DIO1Fhss=1,DIO1CadDetected=2};
enum DIO2modeEnum {DIO2Fhss=0};
enum DIO3modeEnum {DIO3CadDone=0,DIO3ValidHeader=1,DIO3CrcError=2};
enum DIO4modeEnum {DIO4CadDetected=0,DIO4PllLock=1};
enum DIO5modeEnum {DIO5ModeReady=0,DIO5ClkOut=1}; 


void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(int size);
void lora_idle(void);
void lora_sleep(void); 
void lora_receive(void);
void lora_transmit(void);
void lora_receive_single(void);
void lora_CAD(void);
void lora_set_tx_power(int level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(int sf);
void lora_set_bandwidth(long sbw);
void lora_set_coding_rate(int denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
uint8_t lora_init();
void lora_send_packet(uint8_t *buf, int size);
int lora_receive_packet(uint8_t *buf, int size);
int lora_received(void);
int lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);
void lora_dump_registers(void);
uint8_t isRxDone(void);
void resetRxDone(void);
void setDIO(enum DIO0modeEnum DIO0mode,enum DIO1modeEnum DIO1mode,enum DIO2modeEnum DIO2mode,enum DIO3modeEnum DIO3mode,enum DIO4modeEnum DIO4mode,enum DIO5modeEnum DIO5mode);


#endif
