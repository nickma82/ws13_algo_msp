## licensefile for ILOG cplex
## ILOG_LICENCE_FILE auf license file setzten
## in  ~/.bashrc:
## source /home1/share/cplex/initenv

HOSTNAME = $(shell hostname)
USERNAME = $(shell whoami)


EXEC = ./kmst -f data/fad-02.dat -a flow -k 11

DEBUG = 1

CPLEX_VERSION = 11.0
CPLEX_DIR = /home1/share/ILOG/cplex-$(CPLEX_VERSION)

# if you want to use boost, this (old) version is 
# available on the servers
#BOOST_DIR = /home1/share/boost
#BOOST_INCL = include/boost-1_31

SRCDIR = src
OBJDIR = obj

# the default for 64 bit
ARCH = x86-64


# or alternatively, for 32 bit systems
#CXXFLAGS = -m32
#ARCH = x86-32

# we need to use 4.1 (instead of 4.2 or 4.3) as CPLEX
# is not yet compatible with newer compilers
GCC = gcc-4.1
GPP = g++


CPPFLAGS = -DIL_STD \
	-isystem $(CPLEX_DIR)/include \
	-isystem $(CPLEX_DIR)/concert/include

CXXFLAGS += -Wall -Wno-non-virtual-dtor -pipe

LDFLAGS = -L$(CPLEX_DIR)/lib/$(ARCH)/$(GCC)/static_pic \
	-L$(CPLEX_DIR)/concert/lib/$(ARCH)/$(GCC)/static_pic

LDFLAGS += -lilocplex -lcplex -lconcert -lm -lpthread

ifeq ($(DEBUG), 1)
  CXXFLAGS += -g -p
else
  CXXFLAGS += -O2
endif


STARTUP_SOURCE = $(SRCDIR)/Main.cpp

CPP_SOURCES = \
	src/Global.cpp \
	src/Instance.cpp \
	src/kMST_BC.cpp \
	src/kMST_BC_CutCallback.cpp \
	src/Tools.cpp \
	src/dheamaxflow.cpp


# $< the name of the related file that caused the action.
# $* the prefix shared by target and dependent files.
# $? is the names of the changed dependents.
# $@ is the name of the file to be made.
# $^ dependencies above


# ----- object files ---------------------------------------------------------------

OBJ_FILES = $(addprefix $(OBJDIR)/, $(patsubst %.cpp,%.o, \
	$(patsubst src/%, %, $(CPP_SOURCES) ) ) )
STARTUP_OBJ = $(addprefix $(OBJDIR)/, $(patsubst %.cpp,%.o, \
	$(patsubst src/%, %,$(STARTUP_SOURCE) ) ) )


all: kmst

depend:
	@echo 
	@echo "\033[0;36m\033[1mcreating dependencies ... \033[0m"
	$(GPP) -MM $(CPPFLAGS) $(CPP_SOURCES) $(SINGLE_FILE_SOURCES) \
	$(STARTUP_SOURCE) $(LD_FLAGS) \
	| sed -e "s/.*:/$(OBJDIR)\/&/" > depend.in

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(SRCDIR)/%.h
	@echo
	@echo "\033[0;36m\033[1mcompiling $<\033[0m"
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o $@ -c $< 


# for startup (no header file available)
$(OBJDIR)/Main.o: $(SRCDIR)/Main.cpp
	@echo 
	@echo "\033[0;36m\033[1mcompiling $<\033[0m"
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o $@ -c $< 

# ----- linking --------------------------------------------------------------------


kmst: $(STARTUP_OBJ) $(OBJ_FILES)	
	@echo 
	@echo "\033[0;36m\033[1mlinking Main ... \033[0m"
	@echo
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o kmst $(OBJ_FILES) $(STARTUP_OBJ) $(LDFLAGS)


# ----- debugging and profiling ----------------------------------------------------

gdb: all
	gdb --args $(EXEC)

clean:
	rm -rf obj/*.o obj/dheamaxflow/*.o Main gmon.out kmst

doc: all
	doxygen doc/doxygen.cfg

# ------ include dependency file ---------------------------------------------------
include depend.in
