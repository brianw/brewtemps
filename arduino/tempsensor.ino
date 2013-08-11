#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define DEBUG_CONSOLE

#define BOARD_ID 1

/*
 * Network configuration.
 */
byte NET_MAC_ADDR[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

IPAddress NET_LOCAL_IP(10, 0, 0, 100);
#define NET_UDP_LOCAL_PORT 8008

IPAddress NET_REMOTE_IP(10, 0, 0, 1);
#define NET_UDP_REMOTE_PORT 8008

/* Number of probes attached to the multiplexers */
#define AVAILABLE_PROBE_CHANNELS 8

/*
 * Pin configuration.
 */
#define PINS_MULTIPLEXER_INHIBIT 3
#define PINS_MULTIPLEXER_A A2
#define PINS_MULTIPLEXER_B A1
#define PINS_MULTIPLEXER_C A0

// Slave select; HIGH: ignore master, LOW: communication active
#define PINS_ADC_SPI_SELECT 5
#define PINS_ADC_SPI_CLOCK 8

// MOSI: Master Out Slave In
#define PINS_ADC_DATA_OUT 6
// MISO: Master In Slave Out
#define PINS_ADC_DATA_IN 7

#define CHECKSUM_MASK 1365


/*
 * Analog to Digital Converter (ADC) functions for a 12-bit Microchip MCP3208, code
 * originally copied from the Arudino playground:
 *     http://www.arduino.cc/playground/Code/MCP3208
 *
 * Datasheet:
 *     http://ww1.microchip.com/downloads/en/DeviceDoc/21298D.pdf
 *
 * See also:
 *     http://arduino.cc/en/Reference/SPI
 *     http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
 */
void adc_setup() {
  pinMode(PINS_ADC_SPI_SELECT, OUTPUT); 
  pinMode(PINS_ADC_DATA_OUT, OUTPUT); 
  pinMode(PINS_ADC_DATA_IN, INPUT); 
  pinMode(PINS_ADC_SPI_CLOCK, OUTPUT); 
 
  // Initialize with device disabled.
  _adc_device_off();
  digitalWrite(PINS_ADC_DATA_OUT, LOW); 
  digitalWrite(PINS_ADC_SPI_CLOCK, LOW); 
}

void _adc_spi_clock_tick() {
  digitalWrite(PINS_ADC_SPI_CLOCK, HIGH);
  digitalWrite(PINS_ADC_SPI_CLOCK, LOW);
}

void _adc_device_on() {
  digitalWrite(PINS_ADC_SPI_SELECT, LOW);
}

void _adc_device_off() {
  digitalWrite(PINS_ADC_SPI_SELECT, HIGH);
}


/*
 * Read a 12-bit integer from channel (0 indexed).
 */
int adc_read(const byte channel) {
  // Command bits: start, mode, channel (3), not used (3)
  byte command = B11000000;

  // Select channel.
  command |= (channel << 3);

  _adc_device_on();
  // Setup bits to be written.
  for (int i = 7; i >= 3; i--) {
    digitalWrite(PINS_ADC_DATA_OUT, command & 1 << i);
    _adc_spi_clock_tick();
  }

  // Ignore 2 null bits.
  for (int i = 0; i < 2; i++) {
    _adc_spi_clock_tick();
  }

  // Read bits from ADC.
  int adcvalue = 0;
  for (int i = 11; i >= 0; i--) {
    adcvalue += digitalRead(PINS_ADC_DATA_IN) << i;
    _adc_spi_clock_tick();
  }
  
  _adc_device_off();
  return adcvalue;
}


/* 
 * Multiplexer functions for a Texas Instruments CD4051B, an 8-channel 
 * multiplexer.
 *
 * Datasheet:
 *     http://docs-europe.electrocomponents.com/webdocs/0e31/0900766b80e31f3c.pdf
 *
 */
void multiplexer_setup() {
  pinMode(PINS_MULTIPLEXER_INHIBIT, OUTPUT);
  pinMode(PINS_MULTIPLEXER_A, OUTPUT);
  pinMode(PINS_MULTIPLEXER_B, OUTPUT);
  pinMode(PINS_MULTIPLEXER_C, OUTPUT);
}

void multiplexer_select_input(const byte input) {
  digitalWrite(PINS_MULTIPLEXER_INHIBIT, LOW);
  digitalWrite(PINS_MULTIPLEXER_A, input & 1 ? HIGH : LOW);
  digitalWrite(PINS_MULTIPLEXER_B, input & 2 ? HIGH : LOW);
  digitalWrite(PINS_MULTIPLEXER_C, input & 4 ? HIGH : LOW);
}


/*
 * Network functions.
 */
EthernetUDP ethernetUdp;
void net_setup() {
  Ethernet.begin(NET_MAC_ADDR, NET_LOCAL_IP);
  ethernetUdp.begin(NET_UDP_LOCAL_PORT);
}

void net_send_value(const int channel_id, const int value) {
  char packet[512];
  sprintf(packet, "{\"board_id\":%d, \"channel_id\":%d, \"adc_value\":%d, \"checksum\":%d}",
    BOARD_ID, channel_id, value, value ^ CHECKSUM_MASK);
  
  ethernetUdp.beginPacket(NET_REMOTE_IP, NET_UDP_REMOTE_PORT);
  ethernetUdp.write(packet);
  ethernetUdp.endPacket();
}


void send_probe_sample(const byte channel) {
  multiplexer_select_input(channel);
  delay(500);
  int adc_value = adc_read(0);
  net_send_value(channel, adc_value);
  
#ifdef DEBUG_CONSOLE
  Serial.print("Channel ");
  Serial.print(channel);
  Serial.print(": ");
  Serial.println(adc_value);
#endif
}


void setup() {
  multiplexer_setup();
  adc_setup();
  net_setup();

  multiplexer_select_input(3);

#ifdef DEBUG_CONSOLE
  Serial.begin(9600);
#endif
}


void loop() {
  for(byte channel = 0; channel < AVAILABLE_PROBE_CHANNELS; channel++) {
    send_probe_sample(channel);
  }
}
