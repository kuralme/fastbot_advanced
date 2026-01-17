# Configuration
PORT ?= /dev/ttyUSB0
BAUD = 115200

# Targets
.PHONY: all setup build flash monitor clean fresh up down agent-logs reset

# Default action to perform all steps in development
all: down build flash up

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

# Docker Agent Controls
up:
	docker-compose up -d

down:
	docker-compose down

agent-logs:
	docker-compose logs -f uros_agent

# Extra - Helper: Reset the ESP32 via Serial DTR
reset:
	python3 -c "import serial; s=serial.Serial('$(PORT)', $(BAUD)); s.setDTR(False); s.setRTS(False); s.close()"