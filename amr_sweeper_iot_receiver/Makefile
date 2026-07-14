CXX ?= g++

TARGETS := cloud-listener
COMMON_SRCS := azure_iot_hsm_stubs.cpp

AZURE_IOT_PREFIX ?= /opt/azure-iot-sdk-c
AZURE_IOT_INCLUDE_DIR ?= $(AZURE_IOT_PREFIX)/include
AZURE_IOT_LIB_DIR ?= $(AZURE_IOT_PREFIX)/lib
AZURE_IOT_ARCH_LIB_DIR ?= $(AZURE_IOT_PREFIX)/lib/aarch64-linux-gnu

CPPFLAGS += -I$(AZURE_IOT_INCLUDE_DIR)
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pedantic
LDFLAGS += \
        -L$(AZURE_IOT_LIB_DIR) \
        -L$(AZURE_IOT_ARCH_LIB_DIR) \
        -Wl,-rpath,$(AZURE_IOT_LIB_DIR) \
        -Wl,-rpath,$(AZURE_IOT_ARCH_LIB_DIR)
LDLIBS += -Wl,--start-group \
        -liothub_client \
        -liothub_client_mqtt_transport \
        -lumqtt \
        -laziotsharedutil \
        -lprov_auth_client \
        -lparson \
        -Wl,--end-group \
        -lpthread \
        -lssl \
        -lcrypto \
        -lcurl \
        -luuid

.PHONY: all clean

all: $(TARGETS)

cloud-listener: cloud-listener.cpp $(COMMON_SRCS)
        $(CXX) $(CPPFLAGS) $(CXXFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

clean:
        rm -f $(TARGETS)

