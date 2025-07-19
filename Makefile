CC := gcc

#L5CX ULD inc paths
L5CX_CORE_INCLUDE_PATH = -I./drivers/l5cx_uld/user/uld-driver/inc
L5CX_PLATFORM_INCLUDE_PATH = -I./drivers/l5cx_uld/user/platform
L5CX_EXAMPLES_INCLUDE_PATH = -I./drivers/l5cx_uld/user/examples

#L1X API inc paths
L1X_CORE_INCLUDE_PATH = -I./drivers/l1x_uld/API/core
L1X_PLATFORM_INCLUDE_PATH = -I./drivers/l1x_uld/API/platform

BASE_CFLAGS = -Wall -Werror -Wno-missing-braces
CFLAGS_RELEASE = -Os -g0

# L5CX ULD SOURCES
L5CX_LIB_CORE_SOURCES =\
	$(wildcard ./drivers/l5cx_uld/user/uld-driver/src/*.c)

L5CX_LIB_PLATFORM_SOURCES =\
	$(wildcard ./drivers/l5cx_uld/user/platform/*.c)
# remove this pls
L5CX_LIB_EXAMPLES_SOURCES =\
	$(wildcard ./drivers/l5cx_uld/user/examples/*.c)

# L1X API SOURCES
L1X_LIB_CORE_SOURCES =\
	$(wildcard ./drivers/l1x_uld/API/core/*.c)

L1X_LIB_PLATFORM_SOURCES =\
	$(wildcard ./drivers/l1x_uld/API/platform/*.c)

# L5CX
L5CX_LIB_SOURCES := $(L5CX_LIB_CORE_SOURCES) $(L5CX_LIB_PLATFORM_SOURCES)# $(L5CX_LIB_EXAMPLES_SOURCES)
L5CX_INCLUDE_PATH = $(L5CX_CORE_INCLUDE_PATH) $(L5CX_PLATFORM_INCLUDE_PATH) $(L5CX_EXAMPLES_INCLUDE_PATH)

#L1X
L1X_LIB_SOURCES := $(L1X_LIB_CORE_SOURCES) $(L1X_LIB_PLATFORM_SOURCES)
L1X_INCLUDE_PATH = $(L1X_CORE_INCLUDE_PATH) $(L1X_PLATFORM_INCLUDE_PATH)

LIB_SOURCES := $(L5CX_LIB_SOURCES) $(L1X_LIB_SOURCES)
INCLUDE_PATH = $(L5CX_INCLUDE_PATH) $(L1X_INCLUDE_PATH)

CFLAGS = $(BASE_CFLAGS) $(CFLAGS_RELEASE) $(INCLUDE_PATH)

TARGET = background_ranging

LIBS = -lwiringPi

all:
	$(CC) $(CFLAGS) -o background_ranging ./background_ranging.c $(LIB_SOURCES) $(LIBS)

clean:
	rm -f $(TARGET)
