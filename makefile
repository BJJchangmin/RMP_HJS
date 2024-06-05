# Compiler
COMMON=-O2 -I../../include -L../../lib -L/home/jinsong/casadi/build/lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib -Wl,-rpath,/home/jinsong/casadi/build/lib
CC = gcc
CXX = g++

# Compile options - include paths for header files
CXXFLAGS = -std=c++11 
CFLAGS = -std=c11 

# header files directory
INCDIR = -I./include -I../../include -I./source -I/home/user/Desktop/mujoco_cpp/mujoco/include -I/home/jinsong/casadi -I/home/jinsong/casadi/build 

# Library paths
LIBS = -lmujoco -lglfw -lcasadi -ldl

HEADDIR = include
SRCDIR = source
OBJDIR = obj

# List all source files
SRCS = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(SRCDIR)/*.c) main.cpp

# Derive object file names from source file names
OBJS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(SRCS)))
OBJS += $(OBJDIR)/main.o

# Root directory (i.e. current directory) of project
ROOT = main

# Linking
all: $(OBJDIR) $(OBJS)
	$(CXX) $(COMMON) $(CXXFLAGS) $(INCDIR) $(OBJS) $(LIBS) -o $(ROOT)

# Compile each source file to an object file
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@
	
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@

$(OBJDIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@

# Clean the project
clean:
	rm -f $(OBJDIR)/*.o $(ROOT)

# #LINUX
# COMMON=-O2 -I../../include -L../../lib -L/home/jinsong/casadi/build/lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib -Wl,-rpath,/home/jinsong/casadi/build/lib
# LIBS = -lmujoco -lglfw -lcasadi -ldl
# CC = g++

# # header files directory
# INCDIR = -I./include -I../../include -I./source -I/home/user/Desktop/mujoco_cpp/mujoco/include -I/home/jinsong/casadi -I/home/jinsong/casadi/build 


# ROOT = main

# all:
# 	$(CC) $(COMMON) $(INCDIR) main.cpp $(LIBS) -o $(ROOT)

# main.o:
# 	$(CC) $(COMMON) $(INCDIR) -c main.cpp 

# clean:
# 	rm -f *.o $(ROOT)

