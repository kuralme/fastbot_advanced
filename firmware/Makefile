# Configuration
PORT ?= /dev/ttyUSB0
BAUD = 115200
BUILD_DIR := ./build
MAIN_DIR := ./main
COMPILE_DB := $(BUILD_DIR)/compile_commands.json

# Targets
# .PHONY: all setup build flash monitor clean fresh up down agent-logs reset
.PHONY: all setup build flash monitor clean fresh reset tidy tidy-fix tidy-check

# Default action to perform all steps in development
all: build flash

setup:
	@echo "--- Setting ESP32 Target and Opening Menuconfig ---"
	idf.py set-target esp32
	idf.py menuconfig

# Clang-tidy targets
tidy-check:
	@if [ ! -f "$(COMPILE_DB)" ]; then \
		echo "Error: compile_commands.json not found. Run 'make build' first."; \
		exit 1; \
	fi
	@echo "--- Running clang-tidy analysis (read-only) ---"
	@clang-tidy -p $(BUILD_DIR) -header-filter='firmware/main/.*' -p 4 $(MAIN_DIR)/*.c

tidy: tidy-check
	@echo "--- Clang-tidy analysis complete ---"

tidy-fix: tidy-check
	@echo "--- Running clang-tidy with auto-fixes ---"
	@clang-tidy -p $(BUILD_DIR) -header-filter='firmware/main/.*' -fix -p 4 $(MAIN_DIR)/*.c
	@echo "--- Auto-fixes applied. Review changes and rebuild ---"

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

# Docker Agent Controls
# up:
# 	docker-compose up -d

# down:
# 	docker-compose down

# agent-logs:
# 	docker-compose logs -f uros_agent

# Extra - Helper: Reset the ESP32 via Serial DTR
reset:
	python3 -c "import serial; s=serial.Serial('$(PORT)', $(BAUD)); s.setDTR(False); s.setRTS(False); s.close()"