# the default env configuration that is used when not
# specified in env.mk
CARG = 
LARG = 
USER = armsom
HOST = 192.168.68.88
LCORES =-j8
RCORES =-j4

# project structure
SRC_DIR = src	
INCLUDE_DIR = include
DEV_INCLUDE = devinclude
OBJ_DIR = obj
RUN_DIR = run
MOUNT_FOLDER = mount
WORKSPACE_NAME = $(shell basename ${PWD})

# C++ configurations
CC = aarch64-linux-gnu-g++
CFLAGS = -g $(ADDINC) -I$(INCLUDE_DIR) -I$(SRC_DIR) $(CARG) -std=c++20
LIBS = -lpthread -lgpiod $(LARG)

# files and compliation results
SRC_SOURCES = $(shell find $(SRC_DIR) -name '*.cpp')
SRC_OBJECTS_TEMP = $(patsubst %.cpp,%.o,$(SRC_SOURCES))
SRC_OBJECTS = $(patsubst $(SRC_DIR)/%,$(OBJ_DIR)/src/%,$(SRC_OBJECTS_TEMP))

INCLUDE_SOURCES = $(shell find $(INCLUDE_DIR) -name '*.cpp')
INCLUDE_OBJECTS_TEMP = $(patsubst %.cpp,%.o,$(INCLUDE_SOURCES))
INCLUDE_OBJECTS = $(patsubst $(INCLUDE_DIR)/%,$(OBJ_DIR)/include/%,$(INCLUDE_OBJECTS_TEMP))

EXECUTABLE = $(RUN_DIR)/main

REMOTE_OBJECTS = $(shell find $(OBJ_DIR) -name '*.o')

## == Compilation

# create executable file from all object files
$(EXECUTABLE): $(SRC_OBJECTS) $(INCLUDE_OBJECTS)
	mkdir -p $(RUN_DIR)
	$(CC) $(SRC_OBJECTS) $(INCLUDE_OBJECTS) -o $@ $(LIBS)

# create object file for every source file in source
$(OBJ_DIR)/src/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# create object file for every source file in include
$(OBJ_DIR)/include/%.o: $(INCLUDE_DIR)/%.cpp
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# links pre-copied objects on the remote target (hybrid link)
hlink: $(REMOTE_OBJECTS)
	mkdir -p $(RUN_DIR)
	$(CC) $(REMOTE_OBJECTS) -o $(EXECUTABLE) $(LIBS)


## == Local object compilation
local_compile: $(SRC_OBJECTS) $(INCLUDE_OBJECTS)

## == Remote target management

# run main binary remote target
remote_start: 
	ssh -t $(USER)@$(HOST) "cd projects/$(WORKSPACE_NAME); sudo $(RUN_DIR)/main"

# envoke build on remote target
remote_build:
	ssh $(USER)@$(HOST) "bash -c \"cd projects/$(WORKSPACE_NAME) && make $(RCORES)\""

# envoke hlink on remote target
remote_hlink:
	ssh $(USER)@$(HOST) "bash -c \"cd projects/$(WORKSPACE_NAME) && make hlink\""

# copy sources and Makefile to remote target
copy_sources:
	ssh $(USER)@$(HOST) "rm -rf projects/$(WORKSPACE_NAME)/* && mkdir -p projects/$(WORKSPACE_NAME)/$(SRC_DIR) && mkdir -p projects/$(WORKSPACE_NAME)/$(INCLUDE_DIR)"
	scp -r ../$(WORKSPACE_NAME)/$(SRC_DIR) $(USER)@$(HOST):projects/$(WORKSPACE_NAME)/
	scp -r ../$(WORKSPACE_NAME)/$(INCLUDE_DIR) $(USER)@$(HOST):projects/$(WORKSPACE_NAME)/
	scp -r ../$(WORKSPACE_NAME)/Makefile $(USER)@$(HOST):projects/$(WORKSPACE_NAME)/Makefile

# copy compiled objects and Makefile to remote target
copy_objects:
	ssh $(USER)@$(HOST) "rm -rf projects/$(WORKSPACE_NAME)/* && mkdir -p projects/$(WORKSPACE_NAME)/$(SRC_DIR) && mkdir -p projects/$(WORKSPACE_NAME)/$(INCLUDE_DIR)"
	scp -r ../$(WORKSPACE_NAME)/$(OBJ_DIR) $(USER)@$(HOST):projects/$(WORKSPACE_NAME)/
	scp -r ../$(WORKSPACE_NAME)/Makefile $(USER)@$(HOST):projects/$(WORKSPACE_NAME)/Makefile

# open shell on remote target
shell:
	ssh -t $(USER)@$(HOST) "cd projects/$(WORKSPACE_NAME); exec '$$SHELL'"

# invokes ssh-keygen
keygen:
	ssh-keygen
	
# invokes ssh copy ID for the remote host
keycopy:
	ssh-copy-id $(USER)@$(HOST)


## == shortcuts and convenience features

# quickly copy files to target, compile and start on target
start:
	make copy_sources
	make remote_build
	make remote_start

hstart: 
	make local_compile $(LCORES) ADDINC=-I$(DEV_INCLUDE)
	make copy_objects
	make remote_hlink
	make remote_start

# quickly copy files to target and compile them
build:
	make copy_sources
	make remote_build

# localy builds objects, then copies them to the target and links them there
hbuild:
	make local_compile $(LCORES) ADDINC=-I$(DEV_INCLUDE)
	make copy_objects
	make remote_hlink

# mount the 
mount:
	mkdir -p $(MOUNT_FOLDER)
	sshfs -o allow_other $(USER)@$(HOST):/ $(MOUNT_FOLDER)

unmount:
	sudo umount $(MOUNT_FOLDER)

# remove all subfolders and files of the object dir and the executable
clean:
	rm -rf $(EXECUTABLE) $(OBJ_DIR)/*