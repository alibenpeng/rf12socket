BOARD = rf12_socket_type_2a
SERIALDEV = /dev/ttyUSB0
LOCAL_CFLAGS := -D$(BOARD) -DDEBUG=0

include ../arduino.mk
