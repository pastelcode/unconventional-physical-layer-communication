# Host unit tests for the pure-C protocol modules.
#
#   make          build and run
#   make clean
#
# These compile the same crc8.c / frame.c / transaction.c sources that the
# firmware uses, with PHY_HOST_BUILD so protocol_config.h skips sdkconfig.h.

CC ?= cc
CFLAGS ?= -std=c11 -O2 -g
CFLAGS += -Wall -Wextra -Werror -DPHY_HOST_BUILD
INCLUDES = -I../main

SRCS = test_main.c ../main/crc8.c ../main/frame.c ../main/transaction.c
BIN = test_main

.PHONY: all run clean

all: run

$(BIN): $(SRCS) ../main/crc8.h ../main/frame.h ../main/transaction.h ../main/protocol_config.h
	$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
