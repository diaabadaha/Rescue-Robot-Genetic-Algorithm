# ============================================================================
# Rescue Robot Project - Combined Makefile (Backend + GUI)
# ============================================================================

CC      := gcc
CSTD    := -std=c11
WARN    := -Wall -Wextra
OPT     := -g
DEFS    := -D_POSIX_C_SOURCE=200809L

# ----------------------------------------------------------------------------
# Include paths
# ----------------------------------------------------------------------------
INC_CORE := -Iinclude
INC_GUI  := -Irescue_robot_gui/include
CFLAGS   := $(CSTD) $(WARN) $(OPT) $(DEFS) $(INC_CORE) $(INC_GUI)

# ----------------------------------------------------------------------------
# Libraries
# ----------------------------------------------------------------------------
CORE_LIBS := -lm -lrt -lpthread
GUI_LIBS  := -lglfw -lGL -lGLU -lm

# ----------------------------------------------------------------------------
# Directories
# ----------------------------------------------------------------------------
BIN_DIR := bin
CORE_SRC_DIR := src
GUI_SRC_DIR  := rescue_robot_gui/src

# ----------------------------------------------------------------------------
# Sources
# ----------------------------------------------------------------------------
CORE_SRCS := \
    $(CORE_SRC_DIR)/parent.c \
    $(CORE_SRC_DIR)/child_pool.c \
    $(CORE_SRC_DIR)/ipc.c \
    $(CORE_SRC_DIR)/grid.c \
    $(CORE_SRC_DIR)/ga.c \
    $(CORE_SRC_DIR)/astar.c \
    $(CORE_SRC_DIR)/config.c \
    $(CORE_SRC_DIR)/menu.c \
    $(CORE_SRC_DIR)/performance_comparison.c
    
GUI_SRCS := \
    rescue_robot_gui/src/gui_main.c \
    rescue_robot_gui/src/gui.c \
    rescue_robot_gui/src/gui_camera.c \
    rescue_robot_gui/src/gui_renderer.c \
    rescue_robot_gui/src/gui_ipc_reader.c \
    rescue_robot_gui/src/glad.c \
    src/grid.c



# ----------------------------------------------------------------------------
# Targets
# ----------------------------------------------------------------------------
CORE_BIN := $(BIN_DIR)/rescue_robot
GUI_BIN  := $(BIN_DIR)/rescue_robot_gui

# ----------------------------------------------------------------------------
# Build rules
# ----------------------------------------------------------------------------
.PHONY: all clean core gui

all: core gui

# Backend (GA + simulation)
core: $(BIN_DIR) $(CORE_BIN)

$(CORE_BIN): $(CORE_SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(CORE_LIBS)

# GUI
gui: $(BIN_DIR) $(GUI_BIN)

$(GUI_BIN): $(GUI_SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(GUI_LIBS)

# ----------------------------------------------------------------------------
# Utils
# ----------------------------------------------------------------------------
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR)
