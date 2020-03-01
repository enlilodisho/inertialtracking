# Generic Makefile
# http://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/

IDIR =../include
CC=gcc
CXX=g++
CPPFLAGS=-I$(IDIR)

ODIR=../obj
LDIR =../lib
BDIR =../bin

LIBS=-lm

_DEPS = 
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = InertialTracking.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.cpp $(DEPS)
		$(CXX) -c -o $@ $< $(CFLAGS)

InertialTracking: $(OBJ)
		$(CXX) -o $(patsubst %,$(BDIR)/%,$@) $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
		rm -f $(ODIR)/*.o $(BDIR)/* *~ core $(INCDIR)/*~

