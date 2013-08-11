#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define DEBUG_CONSOLE

/*
 * Network configuration.
 */
byte NET_MAC_ADDR[] = { 0x90, 0xA2, 0xDA, 0x00, 0xF1, 0xE0 };

IPAddress NET_LOCAL_IP(10, 0, 0, 99);
#define NET_UDP_LOCAL_PORT 8007

/* Map output pins to relay channels */
#define RELAY_CHANNEL_START 2
#define RELAY_CHANNEL_END 9


/*
 * Relay channel functions.
 */
void relay_channel_setup() {
    for (byte channel = RELAY_CHANNEL_START; channel <= RELAY_CHANNEL_END; channel++) {
      pinMode(channel, OUTPUT);
      digitalWrite(channel, LOW);
    }
}


void send_status(IPAddress remoteAddress, int remotePort) {
  for (byte channel = RELAY_CHANNEL_START; channel <= RELAY_CHANNEL_END; channel++) {
    net_send_value(remoteAddress, remotePort, channel, bitRead(PORTD, channel));  
#ifdef DEBUG_CONSOLE
    Serial.print("Relay channel ");
    Serial.print(channel);
    Serial.print(": ");
    Serial.println(bitRead(PORTD, channel));
#endif
  }
}


void set_relay_channel(byte channel, boolean status) {
  digitalWrite(channel, status ? HIGH : LOW);
}


/*
 * Network functions.
 */
EthernetUDP ethernetUdp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

void net_setup() {
  Ethernet.begin(NET_MAC_ADDR, NET_LOCAL_IP);
  ethernetUdp.begin(NET_UDP_LOCAL_PORT);
}

void net_send_value(IPAddress remoteAddress, int remotePort, const int channel_id, const int value) {
  char packet[512];
  sprintf(packet, "{\"relay_channel\":%d, \"status\":%d}", channel_id, value);
  
  ethernetUdp.beginPacket(remoteAddress, remotePort);
  ethernetUdp.write(packet);
  ethernetUdp.endPacket();
}




void setup() {
  relay_channel_setup();
  net_setup();

#ifdef DEBUG_CONSOLE
  Serial.begin(9600);
  Serial.print("Relay control board started, listening on ");
  Serial.println(NET_UDP_LOCAL_PORT);
#endif
}


void loop() {
  int packetSize = ethernetUdp.parsePacket();
  if (ethernetUdp.available()) {
    ethernetUdp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    
    char command = packetBuffer[0];
    byte channel = packetBuffer[1];
    char terminate = packetBuffer[2];
    
    packetBuffer[0] = ' ';
    packetBuffer[1] = 0;
    packetBuffer[2] = ' ';
    
    if (channel >= RELAY_CHANNEL_START && channel <= RELAY_CHANNEL_END && terminate == '!') {
      set_relay_channel(channel, command == '+');
    }

    delay(500); 
    send_status(ethernetUdp.remoteIP(), ethernetUdp.remotePort());
    
#ifdef DEBUG_CONSOLE
    Serial.print("Received command: Turn ");
    Serial.print(command == '+' ? "ON" : "OFF");
    Serial.print(" channel ");
    Serial.println(channel);
#endif
  }
}
