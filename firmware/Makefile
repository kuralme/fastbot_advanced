# Configuration
PORT ?= /dev/ttyUSB0
BAUD = 115200
BUILD_DIR := build
MAIN_DIR := main
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

# --------------- Static Analysis Jobs -------------------------------
GCC_INTERNAL := $(abspath $(shell xtensa-esp32-elf-gcc -print-file-name=include))
GCC_NEWLIB := $(abspath $(GCC_INTERNAL)/../../../../../xtensa-esp-elf/include)
TIDY_ARGS := --extra-arg="-target" --extra-arg="xtensa-esp32-elf" \
             --extra-arg="-Wno-unknown-argument" \
			 --extra-arg="-isystem$(GCC_INTERNAL)" \
			 --extra-arg="-isystem$(GCC_NEWLIB)" \
			 --extra-arg="-D__IROM_ATTR=" \
			 --extra-arg="-D__uint32_t=uint32_t" \
			 --extra-arg="-D__uint16_t=uint16_t" \
			 --extra-arg="-D__uint8_t=uint8_t"

GCC_LIB_CI := $(shell find /opt/esp/tools/xtensa-esp32-elf -name "include" -type d | grep "gcc" | head -n 1)
TOOLCHAIN_CI := /opt/esp/tools/xtensa-esp32-elf/opt/xtensa-esp32-elf/xtensa-esp32-elf
TIDY_ARGS_CI := --extra-arg="--target=riscv32" \
                --extra-arg="-Wno-unknown-argument" \
                --extra-arg="-isystem$(GCC_LIB_CI)" \
                --extra-arg="-isystem$(TOOLCHAIN_CI)/include" \
                --extra-arg="-isystem$(TOOLCHAIN_CI)/sys-include" \
                --extra-arg="-D__IROM_ATTR=" \
                --extra-arg="-D__extension__=" \
                --extra-arg="-D__gnuc_va_list=void*"

TIDY_LOG := tidy_report.txt
LOG ?= 0 # Set to 1 for summary log, 0 for full output

# Prepare the compilation database
prep-db:
	@echo "--- Cleaning up compile_commands.json for Clang ---"
	@# Remove GCC/Xtensa specific flags that Clang-Tidy doesn't recognize
	@sed -i 's/-fno-shrink-wrap//g' $(COMPILE_DB)
	@sed -i 's/-fno-tree-switch-conversion//g' $(COMPILE_DB)
	@sed -i 's/-fstrict-volatile-bitfields//g' $(COMPILE_DB)

prep-db-ci:
	@echo "--- Cleaning up compile_commands.json for Clang ---"
	@sed -i 's|".*/xtensa-esp32-elf-gcc"|"cc"|g' build/compile_commands.json
	@sed -i 's|".*/xtensa-esp32-elf-g++"|"c++"|g' build/compile_commands.json
	@sed -i 's/-fno-shrink-wrap//g' build/compile_commands.json
	@sed -i 's/-fno-tree-switch-conversion//g' build/compile_commands.json
	@sed -i 's/-fstrict-volatile-bitfields//g' build/compile_commands.json
	@sed -i 's/-mlongcalls//g' build/compile_commands.json
	
tidy-check: prep-db
	@echo "--- Running Clang-Tidy ---"
	@if [ "$(LOG)" = "1" ]; then \
		echo "--- Running Summary Analysis ---"; \
		clang-tidy --quiet -p $(BUILD_DIR) $(MAIN_DIR)/*.c $(TIDY_ARGS) > tidy_report.txt 2>&1 || true; \
		echo -n "Total Warnings: "; grep -c "warning:" tidy_report.txt || echo "0"; \
		echo -n "Total Errors:   "; grep -c "error:" tidy_report.txt || echo "0"; \
	else \
		echo "--- Full Tidy Output ---"; \
		clang-tidy --quiet -p $(BUILD_DIR) $(MAIN_DIR)/*.c $(TIDY_ARGS); \
	fi

tidy-check-ci: prep-db-ci
	@echo "--- Running Clang-Tidy ---"
		clang-tidy -p $(BUILD_DIR) $(MAIN_DIR)/*.c $(TIDY_ARGS_CI) \
			--header-filter="$(MAIN_DIR)/.*" \
			--quiet


tidy-fix:
	@echo "--- Applying Auto-Fixes ---"
	@clang-tidy --fix --quiet -p $(BUILD_DIR) $(MAIN_DIR)/*.c $(TIDY_ARGS)

cppcheck: prep-db
	@echo "--- Running Cppcheck ---"
	@cppcheck --project=$(COMPILE_DB) \
         --enable=all \
         --inline-suppr \
         --std=c11 \
         --suppress=missingIncludeSystem \
         --suppress=unusedFunction \
         --suppress=*:*esp-idf* \
         --suppress=*:*components* \
         --file-filter=$(MAIN_DIR)/* \
         --error-exitcode=1

lint: tidy-check cppcheck
	@echo "--- All static analysis passed! ---"