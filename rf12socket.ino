// Alis RFM12 Socket / Powerstrip

#include <RFM12B.h>
#include <MilliTimer.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>   
#include <avr/wdt.h>
#include <util/delay.h>
#include "config.h"

//#define DEBUG 1
#define HEARTBEAT_PERIOD 10000 // in milliseconds

long lastPeriod = -1;

uint8_t RFM12B_KEY[16];

bool socket_state[NUM_SOCKETS];
uint8_t eeprom_mcu_id;

typedef struct {
	uint8_t mcu_id;
	uint8_t socket_id;
	uint8_t command;
	uint8_t state;
} socket_t;
socket_t socket;

RFM12B radio;
MilliTimer heartbeatTimer;

ISR(WDT_vect) { }

static void blinkLed (uint8_t repeats, uint8_t delay) {
	pinMode(LED_PIN, OUTPUT);
	for (uint8_t i=0; i < repeats; i++) {
		_delay_ms(delay);
		digitalWrite(LED_PIN, HIGH);
		_delay_ms(delay);
		digitalWrite(LED_PIN, LOW);
	}
}

uint8_t getBulkState() {
	uint8_t my_byte = 0;
  #ifdef DEBUG
	Serial.print("Bits: ");
  #endif
  for (int i = 0; i < NUM_SOCKETS; i++) {
		int my_bit = 0;
		my_bit = digitalRead(socket_ids[i]);
    #ifdef DEBUG
		Serial.print(my_bit);
    #endif
		socket_state[i] = my_bit;
		if (my_bit) {
			my_byte |= (1 << i);
		} else {
			my_byte &= ~(1 << i);
		}
  }
  #ifdef DEBUG
	Serial.println();
	Serial.println(my_byte);
  #endif
	return my_byte;
}

void setBulkState(byte b) {
  bool my_bit;
  for (int i = 0; i < NUM_SOCKETS; i++) {
    my_bit = b & (1 << i);
		socket_state[i] = my_bit;
		digitalWrite(socket_ids[i], my_bit);
    #ifdef DEBUG
		Serial.println(digitalRead(socket_ids[i]));
    #endif
  }
  #ifdef DEBUG
	Serial.println(b);
  #endif
}

void receiveRF12() {
	bool send_ack = 0;
	if (radio.ReceiveComplete() && (radio.CRCPass()) && (*radio.DataLen == sizeof(socket_t))) {
		socket = *(socket_t*) radio.Data;
		if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id > 0) && (socket.socket_id <= NUM_SOCKETS)) {
      // MCU ID and Socket ID are valid
			if (socket.command == 0) {
        // Command #0: Query Socket
				socket.state = digitalRead(socket.socket_id - 1);
				send_ack = 1;
			} else if (socket.command == 1) {
        // Command #1: Switch Socket
				if ((socket.state == 0) || (socket.state == 1)) {
					digitalWrite(LED_PIN, socket.state);
					digitalWrite(socket_ids[socket.socket_id - 1], socket.state);
					socket_state[socket.socket_id - 1] = socket.state;
					send_ack = 1;
				}
			} else if (socket.command == 2) {
        // Command #2: Toggle Socket
				socket.state = !socket_state[socket.socket_id - 1];
				digitalWrite(LED_PIN, socket.state);
				digitalWrite(socket_ids[socket.socket_id - 1], socket.state);
				socket_state[socket.socket_id - 1] = socket.state;
				send_ack = 1;
			}
		} else if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id == 0) && (socket.command == 3)) {
      // Command #3: Reset MCU ID
			eeprom_write_byte(EEPROM_MCU_ID, 0xff);
			eeprom_mcu_id = 0xff;
			send_ack = 1;
		} else if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id == 0) && (socket.command == 6)) {
      // Command #6: Bulk Query
      socket.state = getBulkState();
      send_ack = 1;
		} else if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id == 0) && (socket.command == 7)) {
      // Command #7: Bulk Set
      setBulkState(socket.state);
      send_ack = 1;
		} else if ((socket.command == 4) && (socket.mcu_id != 0) && (eeprom_mcu_id == 0xff)) {
      // Command #4: Set new MCU ID
      #ifdef DEBUG
			Serial.print("setting MCU ID");
      #endif
			eeprom_write_byte(EEPROM_MCU_ID, socket.mcu_id);
			eeprom_mcu_id = socket.mcu_id;
			send_ack = 1;
		}
    // Send ACK: This must happen AFTER all processing of rf12_data, etc.
		if (radio.ACKRequested() && (send_ack == 1)) {
			radio.SendACK(&socket, sizeof(socket));
		}
	}
}

void getMcuId() {
  #ifdef DEBUG
	Serial.println("get MCU ID...");
  #endif
	socket.mcu_id = eeprom_mcu_id;
	socket.socket_id = NUM_SOCKETS;
	socket.command = 5; // Command #5: Get new MCU ID

  radio.Send(1, &socket, sizeof(socket_t));

	long now = millis();
  while (now + 1000 > millis()) {
		receiveRF12();
	}
}

void heartbeat() {
  if (heartbeatTimer.poll(HEARTBEAT_PERIOD)) {
    #ifdef DEBUG
    Serial.print("sending Heartbeat for ");
    Serial.println(eeprom_mcu_id);
    #endif

    socket.mcu_id = eeprom_mcu_id;
    socket.socket_id = NUM_SOCKETS;
    socket.command = 8; // Command #8: Heartbeat
    socket.state = getBulkState();
    
    radio.Send(0, &socket, sizeof(socket_t), true);
    #ifdef DEBUG
    Serial.println("Heartbeat sent.");
    #endif
  }
}

void setup () {
  MCUSR = 0; // this is oh so important! It sets the MCU status register to 0, so the MCU doesn't keep rebooting itself!
  wdt_enable(WDTO_2S);

	eeprom_read_block(RFM12B_KEY, RF12_EEPROM_EKEY, 16);

	eeprom_mcu_id = eeprom_read_byte(EEPROM_MCU_ID);
	socket.mcu_id = eeprom_mcu_id;
	socket.state = 0;
	socket.socket_id = 0;

	radio.Initialize(NODE_ID, RF12_868MHZ, NETGROUP);
	radio.Encrypt(RFM12B_KEY);

  #ifdef DEBUG
	Serial.begin(57600);
	Serial.println("[RF12 Socket Debug]");
  #endif

	pinMode(LED_PIN, OUTPUT);
	blinkLed(3, 50);
	for (uint8_t i = 0; i < NUM_SOCKETS; i++) {
		pinMode(socket_ids[i], OUTPUT);
	}
}

void loop () {
	if (eeprom_mcu_id == 0xff) {
		getMcuId();
	} else {
		heartbeat();
		receiveRF12();
	}
  wdt_reset();
}
