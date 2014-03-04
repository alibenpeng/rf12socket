// The following types exsist so far:
//
// Type  1: Powerstrip with 5 individual sockets (m168p)
// Type 2a: Single socket (m168p)
// Type 2b: Single socket (m328p)
//
// this must be set in the Makefile, too!

#define rf12_socket_type_2a

#define NODE_ID 8
#define NETGROUP 42
#define RF12_EEPROM_EKEY ((uint8_t *) 0x40)
#define EEPROM_MCU_ID ((uint8_t *) 0x50)

#if defined rf12_socket_type_1

  #define NUM_SOCKETS 5
  #define LED_PIN 4
  uint8_t socket_ids[NUM_SOCKETS] = { 16, 17, 18, 19, 0 };

#elif defined rf12_socket_type_2a || defined rf12_socket_type_2b

  #define NUM_SOCKETS 1
  #define LED_PIN A3
  uint8_t socket_ids[NUM_SOCKETS] = { 19 };

#else // dummy values for testing

  #define NUM_SOCKETS 1
  #define LED_PIN 4
  uint8_t socket_ids[NUM_SOCKETS] = { 19 };

#endif


