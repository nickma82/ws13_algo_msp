# For home installation adjust CPLEX_DIR, ARCH, GPP, CXXFLAGS, etc.

EXEC = ./kmst -f data/g01.dat -m scf -k 5

DEBUG = 1

CPLEX_VERSION = 12.5
#CPLEX_DIR = /home/nickma/ILOG/cplex-$(CPLEX_VERSION)
#CPLEX_DIR = /home/lukas/apps/CPLEX_algo
CPLEX_DIR = $(HOME)/apps/CPLEX_algo

SRCDIR = src
OBJDIR = obj

# 32bit: x86, 64bit: x86-64
ARCH = x86-64

GPP = g++-4.6

CPPFLAGS = -DIL_STD \
	-isystem $(CPLEX_DIR)/cplex/include \
	-isystem $(CPLEX_DIR)/concert/include

CXXFLAGS += -Wall -Wno-non-virtual-dtor -pipe -std=c++0x

LDFLAGS = -L$(CPLEX_DIR)/cplex/lib/$(ARCH)_sles10_4.1/static_pic \
	-L$(CPLEX_DIR)/concert/lib/$(ARCH)_sles10_4.1/static_pic

LDFLAGS += -lilocplex -lcplex -lconcert -lpthread

ifeq ($(DEBUG), 1)
  CXXFLAGS += -g -p
else
  CXXFLAGS += -O3
endif


STARTUP_SOURCE = $(SRCDIR)/Main.cpp

CPP_SOURCES = \
	src/Instance.cpp \
	src/kMST_ILP.cpp \
	src/Tools.cpp \


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
	@echo "creating dependencies ..."
	$(GPP) -MM $(CPPFLAGS) $(CPP_SOURCES) $(SINGLE_FILE_SOURCES) \
	$(STARTUP_SOURCE) $(LD_FLAGS) \
	| sed -e "s/.*:/$(OBJDIR)\/&/" > depend.in

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(SRCDIR)/%.h
	@echo
	@echo "compiling $<"
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o $@ -c $< 


# for startup (no header file available)
$(OBJDIR)/Main.o: $(SRCDIR)/Main.cpp
	@echo 
	@echo "compiling $<"
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o $@ -c $< 

# ----- linking --------------------------------------------------------------------


kmst: $(STARTUP_OBJ) $(OBJ_FILES)	
	@echo 
	@echo "linking ..."
	@echo
	$(GPP) $(CPPFLAGS) $(CXXFLAGS) -o kmst $(OBJ_FILES) $(STARTUP_OBJ) $(LDFLAGS)


# ----- debugging and profiling ----------------------------------------------------

gdb: all
	gdb --args $(EXEC)

clean:
	rm -rf obj/*.o kmst gmon.out

doc: all
	doxygen doc/doxygen.cfg

# ------ include dependency file ---------------------------------------------------
include depend.in
