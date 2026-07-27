SHELL := cmd.exe

# Local defaults. Override from the command line when CCS or the SDK is
# installed elsewhere, for example:
#   gmake CCS_ROOT=E:/ti/ccs SDK_ROOT=E:/ti/mspm0_sdk_2_11_00_07
CCS_ROOT ?= D:/ccs
SDK_ROOT ?= $(CCS_ROOT)/mspm0_sdk_2_11_00_07
COMPILER_ROOT ?= $(CCS_ROOT)/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS

TIARMCLANG := $(COMPILER_ROOT)/bin/tiarmclang.exe
TIARMHEX := $(COMPILER_ROOT)/bin/tiarmhex.exe

PROJECT_ROOT := ..
LIB_ROOT := ../../libraries
BUILD_DIR := build
TARGET := mspm0g3507_ball_car

OUT_FILE := $(BUILD_DIR)/$(TARGET).out
HEX_FILE := $(BUILD_DIR)/$(TARGET).hex
MAP_FILE := $(BUILD_DIR)/$(TARGET).map

APP_SOURCES := \
	$(wildcard $(PROJECT_ROOT)/code/*.c) \
	$(wildcard $(PROJECT_ROOT)/user/src/*.c)

LIB_SOURCES := \
	$(LIB_ROOT)/sdk/ti_config/ti_msp_dl_config.c \
	$(LIB_ROOT)/zf_common/zf_common_clock.c \
	$(LIB_ROOT)/zf_common/zf_common_debug.c \
	$(LIB_ROOT)/zf_common/zf_common_fifo.c \
	$(LIB_ROOT)/zf_common/zf_common_font.c \
	$(LIB_ROOT)/zf_common/zf_common_interrupt.c \
	$(LIB_ROOT)/zf_device/zf_device_type.c \
	$(LIB_ROOT)/zf_driver/zf_driver_delay.c \
	$(LIB_ROOT)/zf_driver/zf_driver_exti.c \
	$(LIB_ROOT)/zf_driver/zf_driver_gpio.c \
	$(LIB_ROOT)/zf_driver/zf_driver_pit.c \
	$(LIB_ROOT)/zf_driver/zf_driver_pwm.c \
	$(LIB_ROOT)/zf_driver/zf_driver_soft_iic.c \
	$(LIB_ROOT)/zf_driver/zf_driver_timer.c \
	$(LIB_ROOT)/zf_driver/zf_driver_uart.c

STARTUP_SOURCE := $(SDK_ROOT)/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c
DRIVERLIB := $(SDK_ROOT)/source/ti/driverlib/lib/ticlang/m0p/mspm0g1x0x_g3x0x/driverlib.a

SOURCES := $(APP_SOURCES) $(LIB_SOURCES) $(STARTUP_SOURCE)
OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))
DEPS := $(OBJECTS:.o=.d)

SOURCE_DIRS := \
	$(PROJECT_ROOT)/code \
	$(PROJECT_ROOT)/user/src \
	$(LIB_ROOT)/sdk/ti_config \
	$(LIB_ROOT)/zf_common \
	$(LIB_ROOT)/zf_device \
	$(LIB_ROOT)/zf_driver \
	$(dir $(STARTUP_SOURCE))

vpath %.c $(SOURCE_DIRS)

TARGET_FLAGS := \
	-march=thumbv6m \
	-mcpu=cortex-m0plus \
	-mfloat-abi=soft \
	-mlittle-endian \
	-mthumb

INCLUDES := \
	-I"$(PROJECT_ROOT)/code" \
	-I"$(PROJECT_ROOT)/user/inc" \
	-I"$(LIB_ROOT)/sdk/ti_config" \
	-I"$(SDK_ROOT)/source/third_party/CMSIS/Core/Include" \
	-I"$(SDK_ROOT)/source" \
	-I"$(LIB_ROOT)/zf_common" \
	-I"$(LIB_ROOT)/zf_components" \
	-I"$(LIB_ROOT)/zf_device" \
	-I"$(LIB_ROOT)/zf_driver"

CFLAGS := \
	-std=c11 \
	$(TARGET_FLAGS) \
	-O2 \
	-g \
	-Wall \
	-Wextra \
	-Wno-unused-parameter \
	-ffunction-sections \
	-fdata-sections \
	-D__MSPM0G3507__ \
	-D__USE_SYSCONFIG__ \
	$(INCLUDES)

LDFLAGS := \
	$(TARGET_FLAGS) \
	-O2 \
	-g \
	-Wl,-m"$(MAP_FILE)" \
	-Wl,--diag_wrap=off \
	-Wl,--display_error_number \
	-Wl,--warn_sections \
	-Wl,--rom_model \
	-Wl,--unused_section_elimination

.PHONY: all clean rebuild

all: $(HEX_FILE)

$(BUILD_DIR):
	@if not exist "$(subst /,\,$@)" mkdir "$(subst /,\,$@)"

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo TI ArmClang: $<
	@"$(TIARMCLANG)" -c $(CFLAGS) -MMD -MP -MF"$(BUILD_DIR)/$*.d" -MT"$@" -o"$@" "$<"

$(OUT_FILE): $(OBJECTS) device_linker.cmd
	@echo TI Arm Linker: $@
	@"$(TIARMCLANG)" $(LDFLAGS) -o"$@" $(OBJECTS) "$(DRIVERLIB)" -Wl,-l"./device_linker.cmd" -Wl,-llibc.a

$(HEX_FILE): $(OUT_FILE)
	@echo TI Arm Hex: $@
	@"$(TIARMHEX)" --memwidth=8 --romwidth=8 --diag_wrap=off --intel -o "$@" "$<"

clean:
	@if exist "$(subst /,\,$(BUILD_DIR))\*.o" DEL /F /Q "$(subst /,\,$(BUILD_DIR))\*.o"
	@if exist "$(subst /,\,$(BUILD_DIR))\*.d" DEL /F /Q "$(subst /,\,$(BUILD_DIR))\*.d"
	@if exist "$(subst /,\,$(BUILD_DIR))\$(TARGET).out" DEL /F /Q "$(subst /,\,$(BUILD_DIR))\$(TARGET).out"
	@if exist "$(subst /,\,$(BUILD_DIR))\$(TARGET).hex" DEL /F /Q "$(subst /,\,$(BUILD_DIR))\$(TARGET).hex"
	@if exist "$(subst /,\,$(BUILD_DIR))\$(TARGET).map" DEL /F /Q "$(subst /,\,$(BUILD_DIR))\$(TARGET).map"

rebuild: clean all

-include $(DEPS)
