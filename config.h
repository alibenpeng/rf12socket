
#define NODE_ID 8
#define NETGROUP 42
#define EEPROM_MCU_ID (RF12_EEPROM_EKEY + RF12_EEPROM_ELEN)

#if defined rf12_socket_type_1

#define MAX_ID 5
#define LED_PIN 4
uint8_t socket_ids[MAX_ID] = { 16, 17, 18, 19, 0 };

#elif defined rf12_socket_type_2a || defined rf12_socket_type_2b

#define MAX_ID 1
#define LED_PIN 4
uint8_t socket_ids[MAX_ID] = { 19 };

#else // dummy values for testing

#define MAX_ID 1
#define LED_PIN 4
uint8_t socket_ids[MAX_ID] = { 19 };

#endif


