ROBOCLAW = roboclaw.o
EXAMPLES = roboclaw-test
CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -c $(DEBUG)
CXX_FLAGS = -O2 -Wall -c $(DEBUG)
LFLAGS = -O2 -Wall $(DEBUG)

roboclaw.o : roboclaw.h roboclaw.c
	$(CC) $(CFLAGS) roboclaw.c
		
all : $(ROBOCLAW) $(EXAMPLES)
	
examples: $(EXAMPLES)
			
roboclaw-test : roboclaw.o roboclaw-test.o
	$(CC) $(LFLAGS) roboclaw.o roboclaw-test.o -o roboclaw-test
		
roboclaw-test.o : roboclaw.h examples/roboclaw-test.c
	$(CC) $(CFLAGS) examples/roboclaw-test.c
		
clean:
	\rm -f *.o examples/*.o $(ROBOCLAW) $(EXAMPLES)
