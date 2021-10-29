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

CFLAGS= -Wall -std=c11 -lpthread -lrt -lm -ltoml

##---------------------------------------
## SRC relative path of source files
##---------------------------------------

SRC= src

##---------------------------------------
## OBJS are the object files to be linked
##---------------------------------------

#OBJ2= mylib2
#OBJS= $(MAIN).o $(OBJ1).o $(OBJ2).o

OBJ1= task
OBJ2= graphics
OBJ3= model
OBJ4= configure

OBJS= $(MAIN).o $(OBJ1).o $(OBJ2).o $(OBJ3).o $(OBJ4).o

##---------------------------------------
## LIBS are the external libraries to be used
##---------------------------------------

LIBS=`allegro-config --libs`

##---------------------------------------
##DEPENDENCIES
##---------------------------------------
$(MAIN): $(OBJS)
	$(CC) -o $(MAIN) $(OBJS) $(LIBS) $(CFLAGS)

$(MAIN).o: $(SRC)/$(MAIN).c
	$(CC) -c $(SRC)/$(MAIN).c

$(OBJ1).o: $(SRC)/$(OBJ1).c
	$(CC) -c $(SRC)/$(OBJ1).c

$(OBJ2).o: $(SRC)/$(OBJ2).c
	$(CC) -c $(SRC)/$(OBJ2).c

$(OBJ3).o: $(SRC)/$(OBJ3).c
	$(CC) -c $(SRC)/$(OBJ3).c

$(OBJ4).o: $(SRC)/$(OBJ4).c
	$(CC) -c $(SRC)/$(OBJ4).c
	
clean:
	@rm -f $(MAIN) *.o
