CXX ?= g++

BUILD_DIR ?= build
TARGET ?= c2d_receiver
SRCS ?= c2d_receiver.cpp azure_iot_hsm_stubs.cpp
INSTALL_DIR ?= .

AZURE_IOT_PREFIX ?= /opt/ros/jazzy
AZURE_IOT_INCLUDE_DIR ?= $(AZURE_IOT_PREFIX)/include
AZURE_IOT_LIB_DIR ?= $(AZURE_IOT_PREFIX)/lib
AZURE_IOT_ARCH_LIB_DIR ?= $(AZURE_IOT_PREFIX)/lib/aarch64-linux-gnu
AZURE_IOT_CFLAGS ?= -I$(AZURE_IOT_INCLUDE_DIR)
AZURE_IOT_LIBS ?= -L$(AZURE_IOT_LIB_DIR) -L$(AZURE_IOT_ARCH_LIB_DIR) -liothub_client -liothub_client_mqtt_transport -lumqtt -laziotsharedutil
AZURE_IOT_EXTRA_LIBS ?= -lprov_auth_client -lparson
AZURE_IOT_OPTIONAL_LIBS ?=

CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
SYSTEM_LIBS ?= -lpthread -lssl -lcrypto -lcurl -luuid
LDLIBS ?= -Wl,--start-group $(AZURE_IOT_LIBS) $(AZURE_IOT_EXTRA_LIBS) $(AZURE_IOT_OPTIONAL_LIBS) -Wl,--end-group $(SYSTEM_LIBS)

.PHONY: all install clean

all: $(BUILD_DIR)/$(TARGET)

install: $(BUILD_DIR)/$(TARGET)
	mkdir -p $(INSTALL_DIR)
	install -m 755 $(BUILD_DIR)/$(TARGET) $(INSTALL_DIR)/$(TARGET)

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/$(TARGET): $(SRCS) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(AZURE_IOT_CFLAGS) $(CXXFLAGS) $(SRCS) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -rf $(BUILD_DIR)
