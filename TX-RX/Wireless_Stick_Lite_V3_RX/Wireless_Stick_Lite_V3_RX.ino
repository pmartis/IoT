/* Tools / Board / Heltec ESP32 Series Dev-boards --> Wireless Stick Lite(V3)
 * 
 * follow functions:
 * 
 * - Only receive data from LoRa device
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
#include <../../../include/secrets.h>
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
#define BUFFER_SIZE                                 30 // Define the payload size here

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
    STATE_TX
}States_t;

int16_t txNumber;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

String rssi = "RSSI --";
String packSize = "--";
String packet;
String send_num;
String show_lora = "lora data show";

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
uint64_t chipid;
int16_t RssiDetection = 0;


void OnTxDone( void )
{
	Serial.print("TX done......");
	state=STATE_RX;

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
	SerialPrintMsgRX("OK");
  //Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
  //Serial.println("wait to send next packet");
	receiveflag = true;
  //state=STATE_TX;
}


void lora_init(void)
{
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
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
	//state=STATE_TX;
	state=STATE_RX;
}


/********************************* lora  *********************************************/

void WIFISetUp(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_USER, WIFI_PASS);//fill in "Your WiFi SSID","Your Password"
	delay(100);
  Serial.print("Connecting...");
	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10)
	{
		count ++;
		delay(500);
	}

	if(WiFi.status() == WL_CONNECTED)
	{
		Serial.println("OK");
	}
	else
	{
		Serial.println("Failed");
	}
	Serial.println("WIFI Setup done");
	delay(500);
}

bool resendflag=false;
bool deepsleepflag=false;
bool interrupt_flag = false;

void interrupt_GPIO0()
{
	interrupt_flag = true;
}

void interrupt_handle(void)
{
	if(interrupt_flag) {
		interrupt_flag = false;
		if(digitalRead(0)==0) {
			if(rxNumber <=2) {
				resendflag=true;
			} else {
				deepsleepflag=true;
			}
		}
	}
}

/*****************************
PMF - FUNCIONES
******************************/
void SerialPrintMsgRX(const char *msg)
{
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\r\n",(uint32_t)chipid);//print Low 4bytes.
	Serial.print("WiFi status: ");  Serial.println(WiFi.status());
	Serial.printf("IP Address: %s", WiFi.localIP().toString());
	//Serial.print("IP Address: ");  Serial.println(WiFi.localIP());
	Serial.printf("\r\nMensaje recibido: \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
	//Serial.printf("Mensaje recibido: %s", msg);
}

void setup()
{
	Serial.begin(115200);
	delay(100);
	WIFISetUp();

	chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\r\n",(uint32_t)chipid);//print Low 4bytes.

	attachInterrupt(0,interrupt_GPIO0,FALLING);
	lora_init();
	packet ="waiting lora data!";
  Serial.println(packet);
  delay(100);
	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);  
}

void loop()
{
	interrupt_handle();
	if(deepsleepflag) {
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

	if(resendflag) {
		state = STATE_TX;
		resendflag = false;
	}

	if(receiveflag && (state==LOWPOWER) ) {
		receiveflag = false;
		packet ="R_data:";
		if((rxNumber%2)==0) {
			digitalWrite(LED, HIGH);
		} else {
			digitalWrite(LED, LOW);
		}
		state=STATE_RX;
	}
	switch(state) {
    case STATE_TX:
      delay(1000);
      txNumber++;
      sprintf(txpacket,"hello %d,Rssi:%d",txNumber,Rssi);
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      state=LOWPOWER;
      break;
    case STATE_RX:
      //Serial.println("into RX mode");
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