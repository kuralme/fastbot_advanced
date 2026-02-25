# Configuration
PORT ?= /dev/ttyUSB0
BAUD = 115200
BUILD_DIR := ./build
MAIN_DIR := ./main
COMPILE_DB := $(BUILD_DIR)/compile_commands.json

# Targets
.PHONY: all setup build flash monitor clean fresh reset tidy tidy-fix tidy-check

# --------------- ESP32 Build and Flashing -------------------------------
# Default action
all: build flash

setup:
	@echo "--- Setting ESP32 Target and Opening Menuconfig ---"
	idf.py set-target esp32
	idf.py menuconfig

build:
	@echo "--- Starting Build ---"
	idf.py build || exit 1

flash:
	@echo "--- Starting Flashing ---"
	idf.py -p $(PORT) -b $(BAUD) flash || exit 1

# Open Serial Monitor
monitor:
	idf.py -p $(PORT) monitor

# Deep clean
clean:
	@echo "--- Performing Deep Clean ---"
	rm -rf ./build/
	rm -rf ./components/micro_ros_espidf_component/micro_ros_dev
	rm -rf ./components/micro_ros_espidf_component/micro_ros_src
	rm -rf ./components/micro_ros_espidf_component/libmicroros.a
	rm -rf ./components/micro_ros_espidf_component/include
	@echo "Clean finished."

# Full reset: Clean rebuild, flash, and start agent
fresh: clean build flash up

# Extra - Helper: Reset the ESP32 via Serial DTR
reset:
	python3 -c "import serial; s=serial.Serial('$(PORT)', $(BAUD)); s.setDTR(False); s.setRTS(False); s.close()"

# --------------- Docker Agent Controls -------------------------------
# up:
# 	docker-compose up -d

# down:
# 	docker-compose down

# agent-logs:
# 	docker-compose logs -f uros_agent

# --------------- Clang-tidy jobs-------------------------------

TIDY_LOG := tidy_report.txt
GCC_INTERNAL := $(abspath $(shell xtensa-esp32-elf-gcc -print-file-name=include))
GCC_NEWLIB := $(abspath $(GCC_INTERNAL)/../../../../../xtensa-esp-elf/include)

tidy-check:
	@echo "--- Starting Clang-Tidy Analysis ---"
	@clang-tidy -p $(BUILD_DIR) $(MAIN_DIR)/*.c --quiet \
		--extra-arg="-isystem$(GCC_INTERNAL)" \
		--extra-arg="-isystem$(GCC_NEWLIB)" \
		--extra-arg="-D__IROM_ATTR=" \
		--extra-arg="-D__uint32_t=uint32_t" \
		--extra-arg="-D__uint16_t=uint16_t" \
		--extra-arg="-D__uint8_t=uint8_t" > $(TIDY_LOG) 2>&1 || true
	@echo -n "Total Warnings: "
	@grep -c "warning:" $(TIDY_LOG) || echo "0"
	@echo -n "Total Errors:   "
	@grep -c "error:" $(TIDY_LOG) || echo "0"
	@echo "--- Full report saved to $(TIDY_LOG) ---"

tidy-fix:
	@echo "--- Applying Auto-Fixes ---"
	@clang-tidy -p $(BUILD_DIR) $(EXTRA_ARGS) -fix $(MAIN_DIR)/*.c --quiet