# =============================================================================
#  Mini ECU — Makefile
#  Works with GCC on Windows (MinGW/MSYS2) and Linux/macOS.
# =============================================================================

TARGET   := mini_ecu
CC       := gcc

# Directories
SRC_DIR  := src
INC_DIR  := include
OBJ_DIR  := build

# Source files
SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

# Flags
CFLAGS := \
    -std=c99          \
    -Wall             \
    -Wextra           \
    -Wpedantic        \
    -Wconversion      \
    -Wshadow          \
    -I$(INC_DIR)      \
    -DECU_PLATFORM_NATIVE=1

# Debug build (default)
CFLAGS += -g -O0

# ---- Targets ----------------------------------------------------------------

.PHONY: all clean run

all: $(OBJ_DIR)/$(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@
	@echo "Build complete: $@"

run: all
	./$(OBJ_DIR)/$(TARGET)

clean:
	rm -rf $(OBJ_DIR)

# =============================================================================
#  Windows nmake / MSVC alternative
#  Use:  nmake /f Makefile.nmake
#  (see Makefile.nmake in the project root)
# =============================================================================
