#File : core_portme.mak

# Flag : OUTFLAG
#	Use this flag to define how to to get an executable (e.g -o)
OUTFLAG= -o

# Flag : CC
#	Use this flag to define compiler to use
CC = $(ARCH)-elf-gcc

# Flag : CFLAGS
#	Use this flag to define compiler options. Note, you can add compiler options from the command line using XCFLAGS="other flags"

WITHOUT_FLOAT = true
ENDIANFLAGS = -EL

# Default load offset - bootloader is at 0x00000000
ifndef LOADADDR
  LOADADDR = 0x400
endif

ifeq ($(findstring 0x8, ${LOADADDR}),)
MK_CFLAGS += -DBRAM
endif

# Includes
MK_INCLUDES += -I${BASE_DIR}include
MK_STDINC = -nostdinc -include sys/param.h

# Libs
ifeq ($(ARCH),mips)
 LIBDIR = ${BASE_DIR}lib/${ARCH}el
else
 LIBDIR = ${BASE_DIR}lib/${ARCH}
endif

ifndef WITHOUT_LIBS
 ifdef WITHOUT_FLOAT
  MK_LIBS = ${LIBS} -lcint
 else
  MK_LIBS = ${LIBS} -lc
 endif
endif

# MIPS-specific flags
ifeq ($(ARCH),mips)
 MK_CFLAGS += -march=f32c
 MK_CFLAGS += ${ENDIANFLAGS}
 MK_CFLAGS += -G 32768
endif

# MIPS-specific flags
ifeq ($(ARCH),riscv)
 MK_CFLAGS += -m32 -mno-muldiv
endif

MK_CFLAGS += ${MK_STDINC} ${MK_INCLUDES}

#MK_CFLAGS += -Wextra -Wsystem-headers -Wshadow -Wpadded -Winline
MK_CFLAGS += -ffreestanding

# Optimization options
MK_CFLAGS += -O2
MK_CFLAGS += -finline-functions -finline-limit=32
MK_CFLAGS += -fpeel-loops -funroll-all-loops
MK_CFLAGS += -fipa-cp-clone -fipa-pta
MK_CFLAGS += -fselective-scheduling -fselective-scheduling2

# Linker flags
#MK_LDFLAHS += ${ENDIANFLAGS}
#MK_LDFLAGS += -N
MK_LDFLAGS += -Wl,--section-start=.init=${LOADADDR}
MK_LDFLAGS += -Wl,--library-path=${LIBDIR}
MK_LDFLAGS += -nostartfiles -nostdlib
ifndef WITHOUT_LIBS
 MK_LDFLAGS += -lcrt0
endif
MK_LDFLAGS += ${MK_LIBS}

PORT_CFLAGS = ${MK_CFLAGS}

FLAGS_STR = "$(PORT_CFLAGS) $(XCFLAGS) $(XLFLAGS) $(LFLAGS_END)"
CFLAGS = $(PORT_CFLAGS) -I$(PORT_DIR) -I. -DFLAGS_STR=\"$(FLAGS_STR)\"

#Flag : LFLAGS_END
#	Define any libraries needed for linking or other flags that should come at the end of the link line (e.g. linker scripts). 
#	Note : On certain platforms, the default clock_gettime implementation is supported but requires linking of librt.
LFLAGS_END = ${MK_LDFLAGS}

# Flag : PORT_SRCS
# 	Port specific source files can be added here
PORT_SRCS = $(PORT_DIR)/core_portme.c

#../../lib/src/sio_poll.c

# Flag : LOAD
#	For a simple port, we assume self hosted compile and run, no load needed.

# Flag : RUN
#	For a simple port, we assume self hosted compile and run, simple invocation of the executable

#For native compilation and execution
LOAD = echo Loading done
RUN = 

OEXT = .o
EXE = .$(ARCH)

# Target : port_pre% and port_post%
# For the purpose of this simple port, no pre or post steps needed.

.PHONY : port_prebuild port_postbuild port_prerun port_postrun port_preload port_postload
port_pre% port_post% : 

# FLAG : OPATH
# Path to the output folder. Default - current folder.
OPATH = ./
MKDIR = mkdir -p

