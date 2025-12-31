/* Tools / Board / Heltec ESP32 Series Dev-boards --> Wireless Stick Lite(V3)
 * 
 * follow functions:
 * 
 * - Only receive data from LoRa device
 * 
 *
 * by Perfecto Martís Flórez
 * https://www.experiweb.net
 * http://es.linkedin.com/in/perfectomartisflorez/
 *
 * this project also realess in GitHub:
 * https://github.com/pmartis/IoT
*/

#include "Arduino.h"
#include "WiFi.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include <../../../include/secrets.h>	//WIFI_USER, WIFI_PASS

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

#define NUMNODOS 5
#define NUMFILAS 10
uint8_t nodosRegistrados[NUMNODOS][6] =  {{0x10, 0x20, 0xBA, 0x69, 0xC0, 0xE0},
                                          {0x54, 0x32, 0x04, 0xFF, 0xFE, 0x3D},
                                          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                                          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

struct __attribute__((packed)) DatosNodos {
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
} datosNodo[NUMNODOS][NUMFILAS];

struct __attribute__((packed)) Mensajes {
	uint8_t node_id[6];
	DatosNodos datos;
} mensaje;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
} States_t;

int16_t txNumber;
int16_t rxNumber;
States_t state;
bool sleepMode = false;
int16_t rxRssi, rxSize;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.

uint64_t chipid;

WiFiServer server(80);  // Set web server port number to 80
struct tm timeinfo;

void ShowWebPage(void)
{
  static const char diaSem[7][17] = {"Lunes","Martes","Mi&eacute;rcoles","Jueves","Viernes","S&aacute;bado","Domingo"};
	const long timeoutTime = 2000;        // Define timeout time in milliseconds (example: 2000ms = 2s)
	unsigned long previousTime = 0;       // Previous time
  unsigned long currentTime = millis(); // Current time

	WiFiClient client = server.available();   // Listen for incoming clients
  if (client) {    
    Serial.println();                         // If a new client connects,
		Serial.println("Nuevo cliente.");
    currentTime = millis();
    previousTime = currentTime;

    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println("Connection: close");
          client.println();

          getLocalTime(&timeinfo);
          Serial.printf("%d-%d %d/%02d/%02d %02d:%02d:%02d\r\n",
                        timeinfo.tm_wday,timeinfo.tm_yday,
                        1900+timeinfo.tm_year,timeinfo.tm_mon+1,timeinfo.tm_mday,
                        timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);

          // Display the HTML web page
          client.println("<!DOCTYPE html><html>");
          client.println("<head>");
          client.println("<style>");
          client.println("html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
          client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
          client.println(".button2 {background-color: #555555;}");
          client.println(".center { margin-left: auto; margin-right: auto;}");
          client.println("</style>");
          client.println("</head>");

          client.println("<body><h1>ESP32 - Perfe Web Server</h1>");
          client.printf("<p>%s, d&iacute;a %d del a&ntilde;o - %d/%02d/%02d %02d:%02d:%02d</p>",
                        diaSem[timeinfo.tm_wday],timeinfo.tm_yday,
                        1900+timeinfo.tm_year,timeinfo.tm_mon+1,timeinfo.tm_mday,
                        timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
          client.println("<table class=\"center\">");
          client.println("<tr><th>ID Nodo</th><th>Fecha y hora</th><th>Latitud</th><th>Longitud</th><th>Temperatura</th><th>Humedad</th><th>Presi&oacute;n</th></tr>");
          for (int idx=0;idx<NUMNODOS;idx++) {
            for (int fil=0;fil<NUMFILAS;fil++) {
              client.print("<tr>");
              client.print("<td>");
              for(int n=5;n>=0;n--) {
                client.printf("%02X",nodosRegistrados[idx][n]);
                if (n>0) client.print(":");
              }
              client.print("</td>");
              client.printf("<td>%d/%02d/%02d %02d:%02d:%02d</td>",
                            datosNodo[idx][fil].year,datosNodo[idx][fil].month,datosNodo[idx][fil].day,
                            datosNodo[idx][fil].hour,datosNodo[idx][fil].minutes,datosNodo[idx][fil].seconds);
              client.printf("<td>%f</td>",(float)datosNodo[idx][fil].latitude / 1e7f);
              client.printf("<td>%f</td>",(float)datosNodo[idx][fil].longitude / 1e7f);
              client.printf("<td>%.2f &deg;C</td>",(float)datosNodo[idx][fil].temperature_x100 / 100.0f);
              client.printf("<td>%.2f &#37;</td>",(float)datosNodo[idx][fil].humidity_x100 / 100.0f);
              client.printf("<td>%.2f hPa</td>",(float)datosNodo[idx][fil].pressure / 100.0f);
            }
            client.println("</tr>");
          }
          client.println("</table>");
          
          client.println("</body></html>");
          client.println(); // The HTTP response ends with another blank line
          break;  // Break out of the while loop
        }
      }
    }
    // Close the connection
    client.stop();
    Serial.println("Cliente desconectado.");
  }
}

void SerialPrintMsgRX(void)
{
	Serial.println();
  Serial.print("WiFi status: ");  Serial.print(WiFi.status());
	Serial.printf(" --- Sitio web 'no seguro': http://%s\r\n", WiFi.localIP().toString());  //Serial.println(WiFi.localIP());
	Serial.printf("Mensaje %d recibido con %d bytes y Rssi: %d desde el nodo con ID: ", rxNumber, rxSize, rxRssi);
  for(int n=5;n>=0;n--) {
    Serial.printf("%02X",mensaje.node_id[n]);
    if (n>0) Serial.print(":");
  }
	Serial.println();
  Serial.printf("%d/%02d/%02d %02d:%02d:%02d@%f,%f#%d,%d,%d\r\n",
				mensaje.datos.year,mensaje.datos.month,mensaje.datos.day,mensaje.datos.hour,mensaje.datos.minutes,mensaje.datos.seconds,
				(float)mensaje.datos.latitude / 1e7,(float)mensaje.datos.longitude / 1e7,
				mensaje.datos.temperature_x100,mensaje.datos.humidity_x100,mensaje.datos.pressure);
}

int8_t BuscaNodo(const uint8_t node_id[6])
{
  static const uint8_t ID_VACIO[6] = {0};

  if (memcmp(mensaje.node_id, ID_VACIO, 6) == 0) return -1;  // ID inválido → descartar
  for (uint8_t i = 0; i < NUMNODOS; i++) {
      if (memcmp(nodosRegistrados[i], node_id, 6) == 0) return i;   // nodo encontrado
  }
  return -1;          // nodo no encontrado
}

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
	int idx;

  Radio.Sleep();
  rxNumber++;
  rxRssi=rssi;
  rxSize=size;
  if (rxSize > sizeof(mensaje)) rxSize = sizeof(mensaje);

  memcpy(&mensaje, payload, rxSize);
  idx = BuscaNodo(mensaje.node_id);
  if (idx >= 0) {
    if (size != sizeof(mensaje)) Serial.printf("*** Error en RX - Recibidos %d bytes, esperados %d bytes.", size, sizeof(mensaje));
    memmove(&datosNodo[idx][1], &datosNodo[idx][0], (NUMFILAS - 1) * sizeof(DatosNodos));
    memcpy(&datosNodo[idx][0], &mensaje.datos.year, sizeof(DatosNodos));
  }
  SerialPrintMsgRX();

	receiveflag = true;
  //state=STATE_TX;
}

void lora_init(void)
{
  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
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
	//state=STATE_TX;
	state=STATE_RX;
  Serial.println("Esperando datos LoRa...");
}


/********************************* lora  *********************************************/

void WIFISetUp(void)
{
	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.disconnect(true);
	delay(100);
	WiFi.mode(WIFI_STA);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_USER, WIFI_PASS); //fill in "Your WiFi SSID","Your Password"
	delay(100);
  Serial.print("Connecting...");
	byte count = 0;
	while(WiFi.status() != WL_CONNECTED && count < 10) {
		count ++;
		delay(500);
	}
	if(WiFi.status() == WL_CONNECTED)	{
		Serial.println("OK");
		server.begin();
    Serial.println("WIFI Setup completado. Adquiriendo la hora de internet: ");
    configTime(0, 0, "pool.ntp.org");
    getLocalTime(&timeinfo);
    Serial.printf("%d-%d %d/%02d/%02d %02d:%02d:%02d\r\n",
                  timeinfo.tm_wday,timeinfo.tm_yday,
                  1900+timeinfo.tm_year,timeinfo.tm_mon+1,timeinfo.tm_mday,
                  timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
    Serial.print("WiFi status: ");  Serial.print(WiFi.status());
    Serial.printf(" --- Sitio web 'no seguro': http://%s\r\n", WiFi.localIP().toString());
  }	else Serial.println("ERROR de conexión a la WIFI.");
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

void setup()
{
	chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID: %04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\r\n",(uint32_t)chipid);//print Low 4bytes.
  memset(datosNodo, 0, sizeof(datosNodo));  //Inicializa datos de los nodos.

  Serial.begin(115200);
	delay(100);
	WIFISetUp();
	lora_init();

	pinMode(LED ,OUTPUT);
	digitalWrite(LED, LOW);  
	attachInterrupt(0,interrupt_GPIO0,FALLING);
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
		if((rxNumber%2)==0) digitalWrite(LED, HIGH);
		  else digitalWrite(LED, LOW);
		state=STATE_RX;
	}

	switch(state) {
    case STATE_TX:
      delay(1000);
      txNumber++;
      sprintf(txpacket,"Hola %d, último rxRssi: %d",txNumber, rxRssi);
      Serial.printf("\r\nEnviando datos \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
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
	ShowWebPage();
}