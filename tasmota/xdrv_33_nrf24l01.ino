/*
  xdrv_33_nrf24l01.ino - nrf24l01 support for Tasmota

  Copyright (C) 2020  Christian Baars and Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


  --------------------------------------------------------------------------------------------
  Version yyyymmdd  Action    Description
  --------------------------------------------------------------------------------------------
  0.9.1.0 20210107  change  - added rf24gw
  ---
  0.9.0.1 20200624  changes - removed unused legacy code
  ---
  0.9.0.0 20191127  started - further development by Christian Baars
                    forked  - from arendst/tasmota            - https://github.com/arendst/Tasmota

*/

#ifdef USE_SPI
#ifdef USE_NRF24

/*********************************************************************************************\
* NRF24l01(+)
*
* Usage: 5 SPI-data-wires plus VVC/ground, use hardware SPI, select GPIO_NRF24_CS/GPIO_NRF24_DC
\*********************************************************************************************/

#define XDRV_33             33

#include <RF24.h>

#ifdef USE_RF24GW
#define RF24GW_VERBOSECRITICAL          0b0000000000000001
#define RF24GW_VERBOSESTARTUP           0b0000000000000010
#define RF24GW_VERBOSERF24              0b0000000010000000

#define RF24GW_STARTUPVERBOSELEVEL      0b0000000000000011 

typedef struct {   // Our payload can be 32 byte max.
    uint8_t     node_id;         
    uint8_t     msg_id;          
    uint8_t     msg_type;        
    uint8_t     msg_flags;   
    uint8_t     orderno;         
    uint8_t     heartbeatno;      
    uint8_t     reserved2;      
    uint8_t     reserved3;      
    uint32_t    data1;         
    uint32_t    data2;         
    uint32_t    data3;         
    uint32_t    data4;         
    uint32_t    data5;         
    uint32_t    data6;         
} payload_t;

payload_t payload;

// structure of UDP data
typedef struct {
  uint16_t      gw_no;         // the number of the sending gateway
  payload_t     payload;      // the payload to send forward
} udpdata_t;

udpdata_t udpdata;

WiFiUDP udpServer;
WiFiServer tcpServer(RF24GW_TCP_PORTNO);
WiFiClient clients[RF24GW_MAX_TCP_CONNECTIONS];

uint8_t  rf24gw_node2hub[] = RF24GW_NODE2HUB;
uint8_t  rf24gw_hub2node[] = RF24GW_HUB2NODE;
char rf24gw_tcp_buffer[RF24GW_MAX_TCP_CONNECTIONS][30];
uint16_t rf24_verboselevel = RF24GW_STARTUPVERBOSELEVEL;
char webmsg[50];
#endif

const char NRF24type[] PROGMEM = "NRF24";

struct {
  uint8_t chipType = 0; // NRF24l01 active: 32 - NRF24L01 , 43- NRF24L01+  ... we mis-use ascii-codes
} NRF24;

/********************************************************************************************/

RF24 NRF24radio;

bool NRF24initRadio() {
  NRF24radio.begin(Pin(GPIO_NRF24_CS), Pin(GPIO_NRF24_DC));
  NRF24radio.powerUp();
  if (NRF24radio.isChipConnected()) {
    return true;
  } else {
    DEBUG_DRIVER_LOG(PSTR("NRF: Chip NOT !!!! connected"));
    return false;
  }
}

void NRF24Detect(void) {
  if (PinUsed(GPIO_NRF24_CS) && PinUsed(GPIO_NRF24_DC) && TasmotaGlobal.spi_enabled) {
    if (NRF24initRadio()) {
      NRF24.chipType = 32; // SPACE
      AddLog_P(LOG_LEVEL_INFO, PSTR("NRF: Model 24L01 initialized"));
      if (NRF24radio.isPVariant()) {
        NRF24.chipType = 43; // +
        AddLog_P(LOG_LEVEL_INFO, PSTR("NRF: Model 24L01+ detected"));
      }
    }
  }
}

#ifdef USE_RF24GW
void rf24gw_init(void) {
  if (NRF24radio.isChipConnected()) {
    NRF24radio.setChannel(RF24GW_CHANNEL);
    NRF24radio.setDataRate(RF24GW_SPEED);
    NRF24radio.setPALevel(RF24_PA_MAX);
    NRF24radio.setRetries(0, 0);
    NRF24radio.setAutoAck(false);
    NRF24radio.disableDynamicPayloads();
    NRF24radio.setPayloadSize(32);
    NRF24radio.setCRCLength(RF24_CRC_16);
    NRF24radio.openWritingPipe(rf24gw_hub2node);
    NRF24radio.openReadingPipe(1,rf24gw_node2hub);
    NRF24radio.startListening();
    if (NRF24radio.isPVariant()) {
      sprintf(webmsg,PSTR("started"));
    } else {
      sprintf(webmsg,PSTR("error"));
    }
    tcpServer.begin();
    AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: TCP Server started on port %u"),RF24GW_TCP_PORTNO);
    udpServer.begin(RF24GW_UDP_PORTNO);
    AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: UDP Server started on port %u"),udpServer.localPort());
  }
}

bool append_until(Stream& source, char* buffer, int bufSize, char terminator) {
    int data=source.read();
    if (data>=0)  {
        int len=static_cast<int>(strlen(buffer));
        do {
            if (len<bufSize-1) {
                buffer[len++]=static_cast<char>(data);
            }
            if (data==terminator) {
                buffer[len]='\0';
                return true;
            }
            data=source.read();
        } while (data>=0);
        buffer[len]='\0';  
    }
    return false;
}

void rf24gw_handle(void) {
  if ( udpServer.localPort() == 0 ) {
    udpServer.begin(RF24GW_UDP_PORTNO);
    AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: >>> UDP Server restarted on port %u"),udpServer.localPort());
  }
  while ( NRF24radio.available() ) {
    NRF24radio.read(&payload, sizeof(payload));
    if (rf24_verboselevel & RF24GW_VERBOSERF24) {
        AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: Got radio from Node: %u"), payload.node_id);
    }
    memcpy(&udpdata.payload, &payload, sizeof(payload));
    udpdata.gw_no = RF24GW_GW_NO;
    udpServer.beginPacket(RF24GW_HUB_IP, RF24GW_HUB_UDP_PORTNO);
    udpServer.write((char*)&udpdata, sizeof(udpdata));
    udpServer.endPacket();
    sprintf(webmsg,PSTR("N>H: %u"), payload.node_id);
   }
  if (udpServer.parsePacket() > 0 ) {
    udpServer.read((char*)&udpdata, sizeof(udpdata));
    memcpy(&payload, &udpdata.payload, sizeof(payload));
    NRF24radio.stopListening();
    NRF24radio.write(&payload, sizeof(payload));
    NRF24radio.startListening(); 
    if (rf24_verboselevel & RF24GW_VERBOSERF24) {
        AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: send radio to Node: %u"), payload.node_id);
    }
    sprintf(webmsg,PSTR("H>N: %u"), payload.node_id);
  }
  WiFiClient client = tcpServer.available();
  if (client) {
     // Find a freee space in the array   
     for (int i = 0; i < RF24GW_MAX_TCP_CONNECTIONS; i++) {
         if (!clients[i].connected()) {
            // Found free space
            clients[i] = client;
            rf24gw_tcp_buffer[i][0]='\0';
            // Send a welcome message
            client.print(PSTR("RF24GW> "));
            return;
         }
     }
     client.stop();
  }  
  static int i=0;
  // Only one connection is checked in each call
  if (clients[i].available()) {
     // Collect characters until line break
     if (append_until(clients[i],rf24gw_tcp_buffer[i],sizeof(rf24gw_tcp_buffer[i]),'\n')) {        
         // Execute some commands
            if (strstr(rf24gw_tcp_buffer[i], "set") && strstr(rf24gw_tcp_buffer[i], "verbose") && strstr(rf24gw_tcp_buffer[i], "+rf24")) {
                clients[i].println(PSTR("Verboselevel rf24 is on"));  
                rf24_verboselevel |= RF24GW_VERBOSERF24;
                clients[i].stop();
                AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: Verboselevel rf24 on"));
            } else
            if (strstr(rf24gw_tcp_buffer[i], "set") && strstr(rf24gw_tcp_buffer[i], "verbose") && strstr(rf24gw_tcp_buffer[i], "-rf24")) {
                clients[i].println(PSTR("Verboselevel rf24 is off"));  
                rf24_verboselevel ^= RF24GW_VERBOSERF24;
                clients[i].stop();
                AddLog_P(LOG_LEVEL_INFO, PSTR("RF24GW: Verboselevel rf24 off"));
            } else
            { 
                clients[i].println(PSTR("Invalid command"));  
                clients[i].println(PSTR("Valid commands are:"));  
                clients[i].println(PSTR("set verbose <+/->rf24"));  
                clients[i].stop();
            }
         // Clear the buffer to receive the next line
         rf24gw_tcp_buffer[i][0]='\0';
     }
  }
  // Switch to the next connection for the next call
  if (++i >= RF24GW_MAX_TCP_CONNECTIONS) {
     i=0;
  }
    
}

void rf24gw_showWeb(void) {
  if (NRF24radio.isChipConnected()) {
    WSContentSend_PD(PSTR("{s}RF24GW{m}%s{e}"),webmsg);
  }
}

#endif

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv33(uint8_t function) {
  bool result = false;

  switch(function) {
    case FUNC_INIT:
      NRF24Detect();
#ifdef USE_RF24GW
      rf24gw_init();
    break;
    case FUNC_EVERY_50_MSECOND:
      rf24gw_handle();
    break;
    case FUNC_WEB_SENSOR:
      rf24gw_showWeb();
#endif
    break;
  }
  return result;
}

#endif  // USE_NRF24
#endif  // USE_SPI

