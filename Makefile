
## Getting source files automatically
SOURCEDIR := $(shell pwd)
SOURCES_FULL_PATH := $(shell find $(SOURCEDIR) -maxdepth 1 -name '*.cc')
SOURCES := $(shell basename -a $(SOURCES_FULL_PATH))
  
OFILES = $(SOURCES:.cc=.o)

#LINUX
# COMPILE_LEVEL := O2 # optimization of code
ifndef COMPILE_LEVEL
	DEBUG_BUILD := -g
	COMPILE_LEVEL := O0 #disable optimization
endif

CPPFLAGS= $(DEBUG_BUILD) -$(COMPILE_LEVEL) -I../../include -I/usr/include/eigen3 -L../../bin -L../../lib -std=c++17 -mavx -pthread -Wl,-rpath,'$$ORIGIN'
CXXFLAGS = -lmujoco -lGL -lm -lglfw

FOLDER = bin
TARGET = dbpendulum

all: $(TARGET) clean
# all:
# 	$(CXX) $(CPPFLAGS) main.cc $(CXXFLAGS) -o $(FOLDER)/$(TARGET)
 

$(TARGET): $(OFILES) bin_folder
	$(CXX) $(CPPFLAGS) $(OFILES) $(CXXFLAGS) -o $(FOLDER)/$(TARGET) 

# own implicit rules for the future:
# bin/%.o : %.c
#         $(COMPILE.c) $(OUTPUT_OPTION) $<

bin_folder: 
	mkdir -p $(FOLDER)

clean:
	rm -f $(OFILES)

fclean: clean 
	rm -rf $(FOLDER)

.PHONY:  all clean flean