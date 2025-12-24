/* Tools / Board / Heltec ESP32 Series Dev-boards --> Wireless Tracker
 *
 * follow functions:
 *
 * - Only transmit data to LoRa device
 *
 *
 * by Perfecto Martís Flórez
 * http://es.linkedin.com/in/perfectomartisflorez/
 * https://www.experiweb.net
 *
 * this project also realess in GitHub:
 * https://github.com/pmartis/IoT
*/


#include "Arduino.h"
#include "WiFi.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#include <../../../include/secrets.h>

typedef enum
{
	WIFI_CONNECT_TEST_INIT,
	WIFI_CONNECT_TEST,
	WIFI_SCAN_TEST,
	LORA_TEST_INIT,
	LORA_COMMUNICATION_TEST,
	DEEPSLEEP_TEST,
	GPS_TEST,
}test_status_t;

HT_st7735 st7735;
TinyGPSPlus gps;

test_status_t  test_status;
uint16_t wifi_connect_try_num = 15;
bool resendflag=false;
bool deepsleepflag=false;
bool interrupt_flag = false;
/********************************* lora  *********************************************/
#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             10        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 100 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX,
    STATE_TX_DONE
}States_t;

int16_t txNumber = 0;
int16_t rxNumber = 0;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String packet;
String send_num;

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;
char chipidTxt[13];
int16_t RssiDetection = 0;


void OnTxDone( void )
{
	Serial.print("TX done......");
	state=STATE_TX_DONE;
	//state=STATE_RX;
}

void OnTxTimeout( void )
{
	Radio.Sleep( );
	Serial.print("TX Timeout......");
	state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	rxNumber++;
	Rssi=rssi;
	rxSize=size;
	memcpy(rxpacket, payload, size );
	rxpacket[size]='\0';
	Radio.Sleep( );
	Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
	Serial.println("wait to send next packet");
	receiveflag = true;
	state=STATE_TX;
}


void lora_init(void)
{
	txNumber=0;
	Rssi=0;
	rxNumber = 0;
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxDone = OnRxDone;

	Radio.Init( &RadioEvents );
	Radio.SetChannel( RF_FREQUENCY );
	Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
									LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
									LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
									0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
	state=STATE_TX;
	st7735.st7735_fill_screen(ST7735_BLACK);
	//packet ="waiting lora data!";
	//st7735.st7735_write_str(0, 10, packet, Font_7x10);
}


/********************************* lora  *********************************************/

void custom_delay(uint32_t time_ms)
{
#if 1
	uint32_t conut = time_ms/10;
	while (conut > 0) {
		conut --;
		delay(10);
		if(	interrupt_flag == true) {
			delay(200);
			if(digitalRead(0)==0) {
				break;
			}
		}
	}
#else
		delay(time_ms);
#endif
}

void wifi_connect_init(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	custom_delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_USER, WIFI_PASS);//fill in "Your WiFi SSID","Your Password"
	st7735.st7735_write_str(0, 20, "WIFI Setup done", Font_7x10);
}

bool wifi_connect_try(uint8_t try_num)
{
	uint8_t count;
	while(WiFi.status() != WL_CONNECTED && count < try_num) {
		count ++;
		st7735.st7735_fill_screen(ST7735_BLACK);
		st7735.st7735_write_str(0, 0, "wifi connecting...", Font_7x10);
		custom_delay(500);
	}
	if(WiFi.status() == WL_CONNECTED) {
		st7735.st7735_fill_screen(ST7735_BLACK);
		st7735.st7735_write_str(0, 0, "wifi connect OK", Font_7x10);
		custom_delay(2500);
		return true;
	} else {
		st7735.st7735_fill_screen(ST7735_BLACK);
		st7735.st7735_write_str(0, 0, "wifi connect failed", Font_7x10);
		custom_delay(1000);
		return false;
	}
}

void wifi_scan(unsigned int value)
{
	unsigned int i;
	WiFi.disconnect(); //
  WiFi.mode(WIFI_STA);
	st7735.st7735_fill_screen(ST7735_BLACK);
	for(i=0;i<value;i++) {
		st7735.st7735_write_str(0, 0, "Scan start...", Font_7x10);
		int n = WiFi.scanNetworks();
		st7735.st7735_write_str(0, 30, "Scan done", Font_7x10);

		if (n == 0) {
			st7735.st7735_fill_screen(ST7735_BLACK);
			st7735.st7735_write_str(0, 0, "no network found", Font_7x10);
			custom_delay(2000);
		} else {
			st7735.st7735_fill_screen(ST7735_BLACK);
			st7735.st7735_write_str(0, 0, (String)n, Font_7x10);
			st7735.st7735_write_str(30, 0, "networks found:", Font_7x10);

			custom_delay(2000);

			for (int i = 0; (i < n) && (i < 0); ++i) {
				String str = (String)(i + 1) +":"+(String)(WiFi.SSID(i))+" ("+(String)(WiFi.RSSI(i))+")";
				st7735.st7735_write_str(0, 0, str, Font_7x10);
				custom_delay(100);
				st7735.st7735_fill_screen(ST7735_BLACK);
			}
		}
	}
}

void interrupt_GPIO0(void)
{
	interrupt_flag = true;
}

void interrupt_handle(void)
{
	if(interrupt_flag) {
		interrupt_flag = false;
		if(digitalRead(0)==0) {
			if(rxNumber < 2) {
				delay(500);
				if(digitalRead(0)==0) {
					test_status = GPS_TEST;
				} else {
					resendflag=true;
				}
			} else {
				test_status = DEEPSLEEP_TEST;
				// deepsleepflag=true;
			}
		}
	}
}

void enter_deepsleep(void)
{
	Radio.Sleep();
	SPI.end();
	pinMode(RADIO_DIO_1,ANALOG);
	pinMode(RADIO_NSS,ANALOG);
	pinMode(RADIO_RESET,ANALOG);
	pinMode(RADIO_BUSY,ANALOG);
	pinMode(LORA_CLK,ANALOG);
	pinMode(LORA_MISO,ANALOG);
	pinMode(LORA_MOSI,ANALOG);
	esp_sleep_enable_timer_wakeup(600*1000*(uint64_t)1000);
	esp_deep_sleep_start();
}

void lora_status_handle(void)
{
	if(resendflag) {
		state = STATE_TX;
		resendflag = false;
	}

	if(receiveflag && (state==LOWPOWER) ) {
		receiveflag = false;
		packet ="Rdata:";
		int i = 0;
		while(i < rxSize) {
			packet += rxpacket[i];
			i++;
		}
		// packSize = "R_Size:";
		// packSize += String(rxSize,DEC);
		String packSize = "R_rssi:";
		packSize += String(Rssi,DEC);
		send_num = "send num:";
		send_num += String(txNumber,DEC);
		st7735.st7735_fill_screen(ST7735_BLACK);
		delay(100);
		st7735.st7735_write_str(0, 0, packet, Font_7x10);
		st7735.st7735_write_str(0, 40, packSize, Font_7x10);
		st7735.st7735_write_str(0, 60, send_num, Font_7x10);
		if((rxNumber%2)==0) {
			digitalWrite(LED, HIGH);
		}
	}
	switch(state) {
		case STATE_TX:
			delay(1000);
			txNumber++;
			sprintf(txpacket,"%s@%d/%02d/%02d %02d:%02d:%02d@%f,%f #Hello %d,Rssi:%d",
				chipidTxt,
				gps.date.year(),gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),gps.time.second(),
				gps.location.lat(),gps.location.lng(),
				txNumber,Rssi);
			Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
			Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
			state=LOWPOWER;
			break;
		case STATE_TX_DONE:
			delay(10000);
			state=STATE_TX;
			break;
		case STATE_RX:
			Serial.println("into RX mode");
			Radio.Rx( 0 );
			state=LOWPOWER;
			break;
		case LOWPOWER:
			Radio.IrqProcess( );
			break;
		default:
			break;
	}
}

//Vext --> GNSS
void Vext_ON()
{
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, HIGH);
}

void Vext_OFF()
{
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, LOW);
}

void gps_read(void)
{
	if(Serial2.available()>0) {
		if(Serial2.peek()!='\n') {
			gps.encode(Serial2.read());
		} else {
			Serial2.read();
			if(gps.time.second()==0) {
				return;
			}
			st7735.st7735_fill_screen(ST7735_BLACK);
			st7735.st7735_write_str(0, 0, (String)"gps_test", Font_7x10);
			String time_str = (String)gps.time.hour() + ":" + (String)gps.time.minute() + ":" + (String)gps.time.second()+ ":"+(String)gps.time.centisecond();
			st7735.st7735_write_str(0, 10, time_str, Font_7x10);
			String latitude = "LAT: " + (String)gps.location.lat();
			st7735.st7735_write_str(0, 20, latitude, Font_7x10);
			String longitude  = "LON: "+  (String)gps.location.lng();
			st7735.st7735_write_str(0, 30, longitude, Font_7x10);

			Serial.printf(" %02d:%02d:%02d.%02d ",gps.time.hour(),gps.time.minute(),gps.time.second(),gps.time.centisecond());
			Serial.print("LAT: ");
			Serial.print(gps.location.lat(),6);
			Serial.print(", LON: ");
			Serial.print(gps.location.lng(),6);
			Serial.println();
			delay(5000);
			while(Serial2.read()>0);
		}
	}
}

void setup()
{
	Serial.begin(115200);
	Vext_ON();
	Serial2.begin(115200,SERIAL_8N1,33,34);
	delay(100);
	Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
	st7735.st7735_init();
	st7735.st7735_fill_screen(ST7735_BLACK);

	attachInterrupt(0,interrupt_GPIO0,FALLING);
	resendflag=false;
	deepsleepflag=false;
	interrupt_flag = false;

	chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	sprintf(chipidTxt,"%04X%08X",(uint16_t)(chipid>>32),(uint32_t)chipid);
	chipidTxt[13]=0;
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);
	test_status = WIFI_CONNECT_TEST_INIT;
}

void loop()
{
	interrupt_handle();
	switch (test_status) {
		case WIFI_CONNECT_TEST_INIT:
			wifi_connect_init();
			test_status = WIFI_CONNECT_TEST;
		case WIFI_CONNECT_TEST:
			if(wifi_connect_try(1)==true)	{
				test_status = WIFI_SCAN_TEST;
			}
			wifi_connect_try_num--;
			break;
		case WIFI_SCAN_TEST:
			wifi_scan(1);
			test_status = LORA_TEST_INIT;
			break;
		case LORA_TEST_INIT:
			lora_init();
			test_status = LORA_COMMUNICATION_TEST;
			break;
		case LORA_COMMUNICATION_TEST:
			lora_status_handle();
			break;
		case DEEPSLEEP_TEST:
			enter_deepsleep();
			break;
		case GPS_TEST:
			gps_read();
			break;
		default:
			break;
	}
	gps_read();
}
