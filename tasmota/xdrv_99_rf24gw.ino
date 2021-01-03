/*
  xdrv_99_rf24gw.ino - RF24GW support for Tasmota

  Copyright (C) 2020  Norbert Wilms

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
*/
#ifdef USE_SPI
#ifdef USE_RF24GW
/*********************************************************************************************\
 * RF24GW implementation for Tasmota
 * RF24GW is part of RF24HUB and only usefull inside this project.
 * For details visit github.com/wilmsn/RF24Hub
 *
 */

/*********************************************************************************************/

#include <RF24.h>

// Verboselevel
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


WiFiUDP udp;
WiFiServer tcpServer(RF24GW_TCP_PORTNO);
WiFiClient clients[RF24GW_MAX_TCP_CONNECTIONS];
RF24 radio(RF24GW_RADIO_CE_PIN,RF24GW_RADIO_CSN_PIN);

uint8_t  rf24gw_node2hub[] = RF24GW_NODE2HUB;
uint8_t  rf24gw_hub2node[] = RF24GW_HUB2NODE;
char rf24gw_tcp_buffer[RF24GW_MAX_TCP_CONNECTIONS][30];
uint16_t rf24_verboselevel = RF24GW_STARTUPVERBOSELEVEL;


void rf24gw_setup() {

DEBUG_DRIVER_LOG(PSTR("1: RF24GW Test"));
DEBUG_SENSOR_LOG(PSTR("2: RF24GW Test"));
AddLog_P(LOG_LEVEL_DEBUG, PSTR("3: RF24GW Test"), "test");
  // init rf24
  radio.begin();
  radio.setChannel(RF24GW_CHANNEL);
  radio.setDataRate(RF24GW_SPEED);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(0, 0);
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(rf24gw_hub2node);
  radio.openReadingPipe(1,rf24gw_node2hub);
  radio.powerUp();
  radio.startListening();
  radio.printDetails();

  udp.begin(RF24GW_UDP_PORTNO);
  tcpServer.begin();

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


  while ( radio.available() ) {
    radio.read(&payload, sizeof(payload));
    if (rf24_verboselevel & RF24GW_VERBOSERF24) 
        AddLog_P(LOG_LEVEL_INFO, "RF24GW: Got radio from Node: %u", payload.node_id);
    memcpy(&udpdata.payload, &payload, sizeof(payload));
    udpdata.gw_no = RF24GW_GW_NO;
    udp.beginPacket(RF24GW_HUB_IP, RF24GW_HUB_UDP_PORTNO);
    udp.write((char*)&udpdata, sizeof(udpdata));
    udp.endPacket();
  }
  if (udp.parsePacket() > 0 ) {
    udp.read((char*)&udpdata, sizeof(udpdata));
    memcpy(&payload, &udpdata.payload, sizeof(payload));
    radio.stopListening();
    radio.write(&payload, sizeof(payload));
    radio.startListening(); 
    if (rf24_verboselevel & RF24GW_VERBOSERF24) 
        AddLog_P(LOG_LEVEL_INFO, "RF24GW: send radio to Node: %u", payload.node_id);
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
            client.print(F("RF24GW> "));
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
         // Send an echo back
//         clients[i].print(F("Echo: "));
//         clients[i].print(rf24gw_tcp_buffer[i]);
            
         // Execute some commands
            if (strstr(rf24gw_tcp_buffer[i], "set") && strstr(rf24gw_tcp_buffer[i], "verbose") && strstr(rf24gw_tcp_buffer[i], "+rf24")) {
                clients[i].println(F("Verboselevel rf24 is on"));  
                rf24_verboselevel |= RF24GW_VERBOSERF24;
                clients[i].stop();
                AddLog_P(LOG_LEVEL_INFO, "RF24GW: Verboselevel rf24 on");
            } else
            if (strstr(rf24gw_tcp_buffer[i], "set") && strstr(rf24gw_tcp_buffer[i], "verbose") && strstr(rf24gw_tcp_buffer[i], "-rf24")) {
                clients[i].println(F("Verboselevel rf24 is off"));  
                rf24_verboselevel ^= RF24GW_VERBOSERF24;
                clients[i].stop();
                AddLog_P(LOG_LEVEL_INFO, "RF24GW: Verboselevel rf24 off");
            } else
            { 
                clients[i].println(F("Invalid command"));  
                clients[i].println(F("Valid commands are:"));  
                clients[i].println(F("set verbose <+/->rf24"));  
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

#endif  // USE_RF24GW
#endif  // USE_SPI
