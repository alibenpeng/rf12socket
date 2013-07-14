// Alis RFM12 Socket / Powerstrip
// The following types exsist so far:
//
// Type 1: Powerstrip with 5 individual sockets
// Type 2: Single socket
//
// Socket commands:
// S: switch
// Q: query socket
// C: request mcu_id
// E: write mcu_id to eeprom
// R: reset mcu_id in eeprom to 0xff

#include <JeeLib.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>   
#include <avr/wdt.h>
#include <util/delay.h>
#include "config.h"

bool socket_state[MAX_ID];
uint8_t eeprom_mcu_id;

typedef struct {
	uint8_t command;
	uint8_t mcu_id;
	uint8_t socket_id;
	uint8_t state;
} socket_t;
socket_t socket;


ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static void blinkLed (uint8_t repeats, uint8_t delay) {
	pinMode(LED_PIN, OUTPUT);
	for (uint8_t i=0;i<repeats;i++) {
		_delay_ms(delay);
		digitalWrite(LED_PIN, HIGH);
		_delay_ms(delay);
		digitalWrite(LED_PIN, LOW);
	}
	//pinMode(LED_PIN, INPUT);
}

void receiveRF12() {
	bool send_ack = 0;
	uint16_t rf12_node_id;
	if (rf12_recvDone() && (rf12_crc == 0) && (rf12_len == sizeof(socket_t))) {
		rf12_node_id = 0x1F & rf12_hdr;
		socket = *(socket_t*) rf12_data;
		if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id > 0) && (socket.socket_id <= MAX_ID)) {
			if (socket.command == 'S') {
				if ((socket.state == 0) || (socket.state == 1)) {
					digitalWrite(LED_PIN, socket.state);
					digitalWrite(socket_ids[socket.socket_id - 1], socket.state);
					socket_state[socket.socket_id - 1] = socket.state;
					send_ack = 1;
				}
			} else if (socket.command == 'Q') {
				socket.state = socket_state[socket.socket_id - 1];
				send_ack = 1;
			}
		} else if ((socket.mcu_id == eeprom_mcu_id) && (socket.socket_id == 0) && (socket.command == 'R')) {
			eeprom_write_byte(EEPROM_MCU_ID, 0xff);
			eeprom_mcu_id = 0xff;
			send_ack = 1;
		} else if ((socket.command == 'E') && (socket.mcu_id != 0) && (eeprom_mcu_id == 0xff)) {
			eeprom_write_byte(EEPROM_MCU_ID, socket.mcu_id);
			eeprom_mcu_id = socket.mcu_id;
			send_ack = 1;
		}
		// this must happen AFTER all processing of rf12_data, etc.
		if ((RF12_WANTS_ACK) && (send_ack == 1)) {
			socket.state = socket_state[socket.socket_id - 1];
			//rf12_sendNow(0, &socket, sizeof(socket));
			rf12_sendStart(RF12_ACK_REPLY, &socket, sizeof(socket));
		}
	}
}

void getMcuId() {
	socket.mcu_id = eeprom_mcu_id;
	socket.command = 'C';

  rf12_sendNow(0, &socket, sizeof(socket_t));
  //rf12_sendWait(1); // don't power down too soon

	long now = millis();
  while (now + 1000 > millis()) {
		receiveRF12();
	}

	//delay(1000);

	
}

void setup () {
  MCUSR = 0; // this is oh so important! It sets the MCU status register to 0, so the MCU doesn't keep rebooting itself!
  wdt_disable();

	rf12_initialize(NODE_ID, RF12_868MHZ, NETGROUP);
	rf12_encrypt(RF12_EEPROM_EKEY);
	pinMode(LED_PIN, OUTPUT);
	blinkLed(3, 50);
	for (uint8_t i = 0; i < MAX_ID; i++) {
		pinMode(socket_ids[i], OUTPUT);
	}
	eeprom_mcu_id = eeprom_read_byte(EEPROM_MCU_ID);
	socket.mcu_id = eeprom_mcu_id;
	socket.state = 0;
	socket.socket_id = 0;
}

void loop () {
	if (eeprom_mcu_id == 0xff) {
		getMcuId();
	} else {
		receiveRF12();
	}
}
