# Generic Makefile
# http://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

IDIR =../include
CC=gcc
CXX=g++
CPPFLAGS=-I$(IDIR) -Wall -std=c++11

ODIR=../obj
LDIR =../lib
BDIR =../bin

LIBS=-lm -lpthread

_DEPS =
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = LSM9DS1.o main.o INS.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

InertialTracking: $(OBJ)
		$(CXX) -o $(patsubst %,$(BDIR)/%,$@) $^ $(CPPFLAGS) $(LIBS)

$(ODIR)/LSM9DS1.o:
		cd $(IDIR)/LSM9DS1-RPI && make LSM9DS1.o
		cp $(IDIR)/LSM9DS1-RPI/LSM9DS1.o $(ODIR)/LSM9DS1.o

$(ODIR)/%.o: %.cpp $(DEPS)
		$(CXX) -c -o $@ $< $(CPPFLAGS)

.PHONY: clean

clean:
		rm -f $(ODIR)/*.o $(BDIR)/* *~ core $(INCDIR)/*~
		cd $(IDIR)/LSM9DS1-RPI && make clean

