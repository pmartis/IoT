/* Tools / Board / Heltec ESP32 Series Dev-boards --> Wireless Tracker
 * https://heltec.org/project/wireless-tracker/
 * ESP32S3 + SX1262 + UC6580
 * 
 * - Only transmit data to LoRa device
 * 
 * by Perfecto Martís Flórez
 * http://es.linkedin.com/in/perfectomartisflorez/
 * https://www.experiweb.net
 *
 * this project also realess in GitHub:
 * https://github.com/pmartis/IoT
*/

#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>

#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#define GNSS_BUFFER	512

#include <HX711.h>		//Bogde’s HX711
#define HX711_DOUT 45
#define HX711_SCK  46
//Valores para linealizar el peso
#define RAWPESO_0	3377.0	// round(337707.285714286/100.0;0)
#define KGPESO_0	0.0
#define RAWPESO_1	4380.0	// round(438004,285714286/100.0;0)
#define KGPESO_1	1.03

#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

#define I2C_SDA 16
#define I2C_SCL 15

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

#define TX_INTERVALTIME	900000	//15' * 60 * 1000 en ms
#define BUFFER_SIZE                                 100 // Tamaño de los paquetes
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
int16_t rxRssi, rxSize;

typedef enum {
	LOWPOWER,
	STATE_RX,
	STATE_TX,
	STATE_TX_DONE
} States_t;
States_t state;

typedef enum {
	LORA_TEST_INIT,
	LORA_COMMUNICATION_TEST,
	DEEPSLEEP_TEST
} test_status_t;
test_status_t  test_status;

HT_st7735 st7735;
TinyGPSPlus gps;
Adafruit_BME280 bme;
HX711 LoadCell;
float peso = 99999.9;

bool resendflag=false;
bool interrupt_flag = false;

union macID {
	uint8_t macbytes[6];
	uint64_t mac64;		//Los 6 bytes más bajos deL chip ID es la dirección MAC. Los 16 bits más altos del uint64_t devuelto están a cero.
} chipid;

struct __attribute__((packed)) Mensajes {
	uint8_t node_id[6];

	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minutes;
	uint8_t seconds;

	int32_t latitude;		// lat × 1e7
	int32_t longitude;	// lon × 1e7

	int16_t  temperature_x100;  // °C × 100
  uint16_t humidity_x100;     // % × 100
  uint32_t pressure;      		// Pa
} mensaje;

int16_t txNumber = 0;
int16_t rxNumber = 0;
bool sleepMode = false;

bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.

char txt[100];

static RadioEvents_t RadioEvents;

void OnTxDone( void )
{
	Serial.println("TX done......");
	state=STATE_TX_DONE;
	//state=STATE_RX;
}

void OnTxTimeout( void )
{
	Radio.Sleep( );
	Serial.println("TX Timeout......");
	state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	rxNumber++;
	rxRssi=rssi;
	rxSize=size;
	memcpy(rxpacket, payload, size );
	rxpacket[size]='\0';
	receiveflag = true;
	state=STATE_TX;

	Radio.Sleep( );
	Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,rxRssi,rxSize);
	Serial.println("wait to send next packet");
}

void lora_init(void)
{
	txNumber=0;
	rxRssi=0;
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
			if(digitalRead(0)==0) break;
		}
	}
#else
		delay(time_ms);
#endif
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
				/*if(digitalRead(0)==0) test_status = GPS_TEST;
				else resendflag=true;*/
				resendflag=true;
			} else test_status = DEEPSLEEP_TEST;
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

void SerialPrintMAC(void)
{
	Serial.println();
	Serial.print("ESP32ChipID = ");
	for(int n=5; n>=0; n--){
		Serial.printf("%02X",chipid.macbytes[n]);
		if (n > 0) Serial.print(":");
	}
	Serial.println();
}

float raw_a_kg(long adcRaw)
{
	return((KGPESO_1-KGPESO_0)/(RAWPESO_1-RAWPESO_0)*((float)adcRaw/100.0-RAWPESO_0));
}

void printScreen(void)
{
	//st7735.st7735_fill_screen(ST7735_BLACK);
	snprintf(txt, sizeof(txt),"%d/%02d/%02d %02d:%02d:%02d",
								gps.date.year(), gps.date.month(), gps.date.day(),
								gps.time.hour(), gps.time.minute(), gps.time.second());
	st7735.st7735_write_str(0, 0, txt, Font_7x10);
	snprintf(txt, sizeof(txt),"%f, %f  ",	gps.location.lat(), gps.location.lng());
	st7735.st7735_write_str(0, 10, txt, Font_7x10);
	snprintf(txt, sizeof(txt),"Temp.: %.2f, %.2f", bme.readTemperature(), bme.readHumidity());
	st7735.st7735_write_str(0, 20, txt, Font_7x10);
	st7735.st7735_write_str(strlen(txt)*7, 20, "%  ", Font_7x10);
	snprintf(txt, sizeof(txt),"%.2f hPa  ", bme.readPressure() / 100.0f);
	st7735.st7735_write_str(0, 30, txt, Font_7x10);
	snprintf(txt, sizeof(txt),"Peso: %.2f kg    ", peso);
	st7735.st7735_write_str(0, 40, txt, Font_7x10);
	}

void data_tx(void)
{
	static unsigned long nextMsgTime = 0ul;
	unsigned long actualTime;
	
	actualTime = millis();
	if (actualTime >= nextMsgTime) {
		nextMsgTime = actualTime + TX_INTERVALTIME;
		txNumber++;
		//Construcción del mensaje LoRa
		mensaje.year = gps.date.year();
		mensaje.month = gps.date.month();
		mensaje.day = gps.date.day();
		mensaje.hour = gps.time.hour();
		mensaje.minutes = gps.time.minute();
		mensaje.seconds = gps.time.second();

		mensaje.latitude = (int32_t)round(gps.location.lat() * 1e7);
		mensaje.longitude = (int32_t)round(gps.location.lng() * 1e7);

		mensaje.temperature_x100 = (int16_t)round(bme.readTemperature() * 100.0f);
		mensaje.humidity_x100 = (uint16_t)round(bme.readHumidity() * 100.0f);
		mensaje.pressure = (uint32_t)round(bme.readPressure());

		Radio.Send( (uint8_t *) &mensaje, sizeof(mensaje) );
		state = LOWPOWER;

		SerialPrintMAC();
		Serial.printf(" --- Enviando mensaje (%d bytes): \r\n", sizeof(mensaje));
		Serial.printf("     %d/%02d/%02d %02d:%02d:%02d@%f,%f#%d,%d,%d\r\n",
									mensaje.year,mensaje.month,mensaje.day,mensaje.hour,mensaje.minutes,mensaje.seconds,
									gps.location.lat(),gps.location.lng(),
									mensaje.temperature_x100,mensaje.humidity_x100,mensaje.pressure);
		Serial.printf("Peso: %f\r\n",peso);
	}
}

void lora_status_handle(void)
{
	int n;

	if(resendflag) {
		state = STATE_TX;
		resendflag = false;
	}

	if(receiveflag && (state==LOWPOWER) ) {
		st7735.st7735_fill_screen(ST7735_BLACK);
		receiveflag = false;
		st7735.st7735_write_str(0, 0, "Rdata:", Font_7x10);
		n = rxSize;
		if (sizeof(txt) < rxSize) n = sizeof(txt);
		snprintf(txt, n, "%s", rxpacket);
		st7735.st7735_write_str(8, 0, txt, Font_7x10);
		snprintf(txt, sizeof(txt),"RX_rssi: %d", rxRssi);
		st7735.st7735_write_str(0, 40, txt, Font_7x10);
		snprintf(txt, sizeof(txt),"RX num: %d", txNumber);
		st7735.st7735_write_str(0, 60, txt, Font_7x10);
		if((rxNumber%2)==0) digitalWrite(LED, HIGH);
	}
	
	switch(state) {
		case STATE_TX:
			data_tx();
			break;
		case STATE_TX_DONE:
			state = STATE_TX;
			break;
		case STATE_RX:
			Serial.println("into RX mode");
			Radio.Rx( 0 );
			state = LOWPOWER;
			break;
		case LOWPOWER:
			Radio.IrqProcess( );
			break;
		default:
			break;
	}
}

//Vext --> GNSS = VGNSS_CTRL = 3
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

void setup()
{
	int n;
	bool statusBME;

	Serial.begin(115200);
	delay(100);
	Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

	chipid.mac64 = ESP.getEfuseMac();
	memcpy(mensaje.node_id, chipid.macbytes, 6);	//Cabecera del mensaje LoRa
	SerialPrintMAC();
	
	Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);

	st7735.st7735_init();
	st7735.st7735_fill_screen(ST7735_BLACK);

	if (!bme.begin(0x76)) {
    Serial.println("BME280 no encontrado");
    while (1);
  }
  Serial.println("BME280 inicializado");

	Serial.println("Inicializando HX711...");
	LoadCell.begin(HX711_DOUT, HX711_SCK);

	Serial2.setRxBufferSize(GNSS_BUFFER);
	Serial2.begin(115200,SERIAL_8N1,33,34);	//GNSS
	Vext_ON();	//Vext --> GNSS
	Serial.println("GNSS inicializado.");

	attachInterrupt(0,interrupt_GPIO0,FALLING);
	resendflag=false;
	interrupt_flag = false;

	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);
	test_status = LORA_TEST_INIT;
}

void loop()
{
	int nCh;

	interrupt_handle();

	nCh = Serial2.available();
	if( nCh > 0) {
		while (Serial2.available()) gps.encode(Serial2.read());
		if( nCh > GNSS_BUFFER) Serial.printf("***** Serial GNSS overflow - bytes recibidos > %d : %d\r\n", GNSS_BUFFER, nCh);
	}
	if (LoadCell.is_ready()) peso = raw_a_kg(LoadCell.read());
	printScreen();

	switch (test_status) {
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
		default:
			break;
	}
}