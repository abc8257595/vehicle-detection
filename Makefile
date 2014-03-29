CC = gcc
CFLAGS = `pkg-config opencv --cflags --libs opencv` -o $@ $^ -I/usr/local/include/opencv2 -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_gpu -lopencv_ts -lopencv_video -lopencv_objdetect -lopencv_ml -lpthread
SRC = $(wildcard *.cpp)
TAR = $(patsubst %.cpp,%,$(SRC))
MAX = $(wildcard max*.cpp)

all:$(TAR)
	echo "compiling $(SRC)"
$(TAR):%:%.cpp
	$(CC) $(CFLAGS) 

max:$(MAX)
	$(CC) $(CFLAGS) 

clean:
	rm $(TAR)

.PHONY:all clean
