CPPFLAGS :=
LDFLAGS :=

# ARM ABI of the target platform
ifeq ($(TEGRA_ARMABI),)
TEGRA_ARMABI ?= aarch64-linux-gnu
endif

# Location of the target rootfs
ifeq ($(shell uname -m), aarch64)
TARGET_ROOTFS :=
else
ifeq ($(TARGET_ROOTFS),)
$(error Please specify the target rootfs path if you are cross-compiling)
endif
endif

# Use absolute path for better access from everywhere
TOP_DIR 	:= ../
CLASS_DIR 	:= $(TOP_DIR)/nvidia_source
UTILS_DIR	:= $(TOP_DIR)/nvidia_source/utils

ifeq ($(shell uname -m), aarch64)
CROSS_COMPILE :=
else
CROSS_COMPILE ?= aarch64-unknown-linux-gnu-
endif
AS             = $(AT) $(CROSS_COMPILE)as
LD             = $(AT) $(CROSS_COMPILE)ld
CC             = $(AT) $(CROSS_COMPILE)gcc
CPP            = $(AT) $(CROSS_COMPILE)g++
AR             = $(AT) $(CROSS_COMPILE)ar
NM             = $(AT) $(CROSS_COMPILE)nm
STRIP          = $(AT) $(CROSS_COMPILE)strip
OBJCOPY        = $(AT) $(CROSS_COMPILE)objcopy
OBJDUMP        = $(AT) $(CROSS_COMPILE)objdump

# Specify the logical root directory for headers and libraries.
ifneq ($(TARGET_ROOTFS),)

CPPFLAGS += --sysroot=$(TARGET_ROOTFS)
LDFLAGS += \
	-Wl,-rpath-link=$(TARGET_ROOTFS)/lib/$(TEGRA_ARMABI) \
	-Wl,-rpath-link=$(TARGET_ROOTFS)/usr/lib/$(TEGRA_ARMABI) \
	-Wl,-rpath-link=$(TARGET_ROOTFS)/usr/lib/$(TEGRA_ARMABI)/tegra
endif

# header files
CPPFLAGS += -std=c++11 \
	-I"$(TARGET_ROOTFS)/usr/include/$(TEGRA_ARMABI)" \
	-I"$(TOP_DIR)/include"


# dependent libraries
LDFLAGS += \
	-lpthread -lv4l2 \
	-lnvbuf_utils \
	-L"$(TARGET_ROOTFS)/usr/lib/$(TEGRA_ARMABI)" \
	-L"$(TARGET_ROOTFS)/usr/lib/$(TEGRA_ARMABI)/tegra"

CC = g++ # C++ compiler

ARGUS_UTILS_DIR := ../include/Utils

INCLUDE += ../include

CFLAGS += -fPIC -g -Wall  # C flags

CPPFLAGS += \
	-I"$(ARGUS_UTILS_DIR)" \
	-I"$(INCLUDE)"

LDFLAGS += -shared -lnveglstream_camconsumer -largus -L/usr/lib/aarch64-linux-gnu/tegra/

RM = rm -f  # rm command

TARGET_LIB += libines_argus.so # target lib

SRCS := 	\
				ines_argus.cpp \
				BufferConsumer.cpp \
				SyncBufferConsumer.cpp \
				H264Consumer.cpp \
				H264Encoder.cpp \
				$(ARGUS_UTILS_DIR)/Thread.cpp \
				$(wildcard $(CLASS_DIR)/*.cpp)
OBJS += $(SRCS:.cpp=.o)

.PHONY: all
all: ${TARGET_LIB}

# compile recources
%.o: %.cpp $(SRCS)
	$(CC) $(CPPFLAGS) -c $< -o $@  $(CFLAGS)

# create library
$(TARGET_LIB): $(OBJS)
	$(CC) -o $@ $(OBJS) $(CPPFLAGS) ${LDFLAGS}

.PHONY: clean
clean:
	-${RM} ${TARGET_LIB} ${OBJS} $(SRCS:.cpp=.d)
