#DIR OUT are passed from shell command

LIB_DIR = $(DIR)/libraries
TMP = $(DIR)/tmp
HDR = $(LIB_DIR)/includes.h
EXE = $(TMP)/main
OBJS = $(TMP)/main.o $(TMP)/i2cio.o $(TMP)/i2cio16.o $(TMP)/wiring.o $(TMP)/Wire.o $(TMP)/Peripherals.o
CC = g++
DEBUG = -g
CFLAGS = -c
LFLAGS = -lrt -lpthread

$(TMP)/main: $(OBJS) $(HDR)
	$(CC) $(LFLAGS) $(OBJS) -o $(TMP)/main
	cp $(TMP)/main $(OUT)

$(TMP)/main.o: $(TMP)/main.cpp $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(TMP)/main.cpp -o $(TMP)/main.o

$(TMP)/Peripherals.o: $(LIB_DIR)/Peripherals/Peripherals.cpp $(LIB_DIR)/Peripherals/Peripherals.h $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(LIB_DIR)/Peripherals/Peripherals.cpp -o $(TMP)/Peripherals.o

$(TMP)/i2cio.o: $(LIB_DIR)/Peripherals/i2cio.cpp $(LIB_DIR)/Peripherals/i2cio.h $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(LIB_DIR)/Peripherals/i2cio.cpp -o $(TMP)/i2cio.o

$(TMP)/i2cio16.o: $(LIB_DIR)/Peripherals/i2cio16.cpp $(LIB_DIR)/Peripherals/i2cio16.h $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(LIB_DIR)/Peripherals/i2cio16.cpp -o $(TMP)/i2cio16.o

$(TMP)/wiring.o: $(LIB_DIR)/cores/wiring.cpp $(LIB_DIR)/cores/wiring.h $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(LIB_DIR)/cores/wiring.cpp -o $(TMP)/wiring.o

$(TMP)/Wire.o: $(LIB_DIR)/Wire/Wire.cpp $(LIB_DIR)/Wire/Wire.h $(HDR)
	$(CC) $(CFLAGS) $(DEBUG) $(LIB_DIR)/Wire/Wire.cpp -o $(TMP)/Wire.o

clean:
	\rm $(TMP)/*.o $(TMP)/main
