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

WiFiServer server(80);  // Set web server port number to 80

void ShowWebPage(void)
{
	String header;  // Variable to store the HTTP request
	unsigned long currentTime = millis(); // Current time
	unsigned long previousTime = 0;       // Previous time
	const long timeoutTime = 2000;        // Define timeout time in milliseconds (example: 2000ms = 2s)

	  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
		Serial.println("Nuevo cliente");
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            /*
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }*/

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP32 - Perfe Web Server</h1>");
            client.println("<p>Son las</p>");
            /*
            // Display current state, and ON/OFF buttons for GPIO 26
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for GPIO 27
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }*/
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
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
		server.begin();
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

void SerialPrintMsgRX(const char *msg)
{
	Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\r\n",(uint32_t)chipid);//print Low 4bytes.
	Serial.print("WiFi status: ");  Serial.print(WiFi.status());
	Serial.printf(" --- IP Address: %s\r\n", WiFi.localIP().toString());
	Serial.printf("Sitio web 'no seguro': http://%ste", WiFi.localIP().toString());
	//Serial.print("IP Address: ");  Serial.println(WiFi.localIP());
	Serial.printf("\r\nMensaje recibido: \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
	//Serial.printf("Mensaje recibido: %s", msg);
	Serial.println();
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
	ShowWebPage();
}