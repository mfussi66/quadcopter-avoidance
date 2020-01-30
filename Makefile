##---------------------------------------
## Target file to be compiled by default
##---------------------------------------

MAIN=main

##---------------------------------------
## CC is the compiler to be used
##---------------------------------------

CC=gcc

##---------------------------------------
## CFLAGS are the options passed to the compiler
##---------------------------------------

CFLAGS= -Wall -std=c11 -lpthread -lrt -lm

##---------------------------------------
## OBJS are the object files to be linked
##---------------------------------------

#OBJ2= mylib2
#OBJS= $(MAIN).o $(OBJ1).o $(OBJ2).o

OBJ1= task
OBJ2= graphics
OBJ3= model

OBJS= $(MAIN).o $(OBJ1).o $(OBJ2).o $(OBJ3).o

##---------------------------------------
## LIBS are the external libraries to be used
##---------------------------------------

LIBS=`allegro-config --libs`

##---------------------------------------
##DEPENDENCIES
##---------------------------------------
$(MAIN): $(OBJS)
	$(CC) -o $(MAIN) $(OBJS) $(LIBS) $(CFLAGS)

$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c

$(OBJ1).o: $(OBJ1).c
	$(CC) -c $(OBJ1).c

$(OBJ2).o: $(OBJ2).c
	$(CC) -c $(OBJ2).c

$(OBJ3).o: $(OBJ3).c
	$(CC) -c $(OBJ3).c
	
clean:
	@rm -f $(MAIN) *.o
