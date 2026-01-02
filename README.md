# IoT
Testing Internet of Things

El conjunto de ejemplos contenidos en este repositorio están basados en la programación de las placas de desarrollo de Heltec (https://heltec.org/).

Para probar dos dispositivos es conveniente empezar con los ejemplos ping-pong de Heltec. Dos dispositivos se enviaran un texto de saludo sucesivamente actuando tanto de emisor como de receptor. Tras esta prueba se puede empezar a modificar el código para adaptarlo poco a poco a las funcionalidades que queremos conseguir.

TX-RX-ASCII
===========
Una segunda prueba de conexión punto a punto está en la carpeta TX-RX-ASCII.
Estos ejemplos se han desarrollado con pequeñas modificaciones a partir de los ejemplos ping-pong de Heltec.
Se trata de una conexión unidireccional, en la que el receptor recibe y muestra en un terminal todo lo que oiga.

https://youtu.be/czkeUJXSHo8?si=uCrelNlibV4LzQ0K

TX-RX
=====
Ejemplos de código en el que los datos se emiten en formato binario y además de han añadido funcionalidades de filtrado para solo tratar los datos de los dispostivos cuya MAC coincida con alguna de las almacenadas en el código.

TX-RX / Wireless_Stick_Lite_V3_RX
=================================
https://heltec.org/project/wireless-stick-lite-v2/

Este dispositivo hace las funciones de receptor de todos los datos que almacena en una matriz y muestra en una página web.
Las variables WIFI_USER y WIFI_PASS contienen el nombre de la red WIFI y la contraseña. Para evitar compartir los datos de acceso de forma publica y que no aparezcan en el código, estas variables se almacenan en un archivo "header" (#include <../../../include/secrets.h>) cuyo contenido podría ser algo como lo siguiente:

//Ver .gitignore
#pragma once

#define WIFI_USER "NOMBREDELAWIFI"
#define WIFI_PASS "CONTRASEÑAWIFI"


TX-RX / Wireless_Tracker_V1_1_TX
================================
https://heltec.org/project/wireless-tracker/

Este dispositivo envía datos en formato binario de posicionamiento GPS, temperatura, humedad y presión barométrica junto con su identificación MAC. Para los datos ambientales utiliza un BME280.

I2C-Scanner
===========
Programa muy simple para buscar verificar que las conexiones del bus I2C, las resistencias de pull-up y la velocidad seleccionada por software están bien configuradas y son compatibles con los dispositivos conectados al bus I2C. Si todo es correcto, se mostrarán las direcciones en hexadecimal de todos los dipositivos conectados.

BME280
======
Este es un buen ejemplo de algunas de las complejidades que rodean a las conexiones I2C, ya que de este sensor se pueden encontrar dos versiones: BME280 (Adafruit_BME280.h) y BMP280 (Adafruit_BMP280.h) aparentemente iguales. Una vez detecta la dirección hexadecimal (0x76 o 0x77), si no se está seguro de cual de las dos versiones se dispone, se prueba con una librería, y si no funciona, con la otra. Una de las dos deberá responder sí o sí. A no ser que el dispositivo esté dañado. Los BMP280 son fabricados por Bosch y son muy fiables, pero en el caso de los BME280 son clones normalmente disponibles en plataformas como Ali-Express a bajo precio y es recomendable comprar 2 o 3 porque 1 de cada 3 o 4 puede fallar.
