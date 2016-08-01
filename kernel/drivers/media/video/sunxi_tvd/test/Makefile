CC := arm-none-linux-gnueabi-gcc
LINUX_DIR=../../../../..
CFLAGS := -I$(LINUX_DIR)/include
TARGET := tvd_test
EXTRA_LIBS += -lpthread
TVD_TEST_SRC = tvd_test.c video.c

.PHONY: all clean


all: $(TARGET)

tvd_test:$(TVD_TEST_SRC)
	$(CC) $(CFLAGS) $(TVD_TEST_SRC) -o $(TARGET) $(EXTRA_LIBS)

clean:
	rm -rf $(TARGET)
