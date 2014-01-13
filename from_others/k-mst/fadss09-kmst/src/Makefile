# soll Debugging-Information in das Programm hineincompiliert werden?
DEBUGLEVEL = 0
#DEBUGLEVEL = 1

# Compiler definieren
CXX = g++

# =============================================================================
# GCC/C++ Flags + Includes + Libraries

ifeq ($(DEBUGLEVEL),1)
  CPPFLAGS = -g -fno-inline -DDEBUG -DPRINTALL
else
  CPPFLAGS = -pipe -O3 -fomit-frame-pointer -s -DNDEBUG
endif

CPPFLAGS += -Wall -Wno-deprecated -Wno-non-virtual-dtor

# =============================================================================
# Compile Directives

all: test

clean:
	rm -f test *.o *~ *.so

test: test.o dheamaxflow.o
	$(CXX) -o test $^ $(CPPFLAGS)

test.o: dheamaxflow.h test.cpp
	$(CXX) -c test.cpp $(CPPFLAGS)

dheamaxflow.o: dheamaxflow.h dheamaxflow.cpp
	$(CXX) -c dheamaxflow.cpp $(CPPFLAGS)


