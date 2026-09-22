# Host-side tests of the robot's decision logic: the real maze.c, search.c,
# storage.c, params.c, crc32.c and telemetry.c against a simulated robot (sim.c).
#   make            build and run (sanitizers on)
#   make run ARGS=-v   verbose: show the firmware log

SRC_DIR  := ../../src
FIRMWARE := $(SRC_DIR)/maze.c $(SRC_DIR)/search.c $(SRC_DIR)/storage.c $(SRC_DIR)/params.c $(SRC_DIR)/crc32.c \
            $(SRC_DIR)/telemetry.c
HOST     := test_main.c sim.c host_io.c fake_flash.c
CC       ?= cc
CFLAGS   := -std=c11 -O1 -g -Wall -Wextra -Wshadow -Werror -I$(SRC_DIR) -fsanitize=address,undefined -fno-omit-frame-pointer
BUILD    := build

.PHONY: all run clean
all: run

$(BUILD)/host_tests: $(FIRMWARE) $(HOST) sim.h $(wildcard $(SRC_DIR)/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(FIRMWARE) $(HOST) -lm

run: $(BUILD)/host_tests
	./$(BUILD)/host_tests $(ARGS)

clean:
	rm -rf $(BUILD)
