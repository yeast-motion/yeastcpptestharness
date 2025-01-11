# Makefile for CMake projects

# Set the default build type to Release
BUILD_TYPE ?= Debug

# Set the build directory
BUILD_DIR ?= build

ADDITIONAL_INCLUDE_PATHS = \
	../yeastcpp/include;/usr/local/include/wpilibNewCommands;/usr/local/include/wpilibc;/usr/local/include/hal;/usr/local/include/ntcore \

# Set the install directory
INSTALL_DIR ?= /usr/local

.PHONY: all clean configure build install

all: build

clean:
	rm -rf $(BUILD_DIR)

configure:
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DADDITIONAL_INCLUDE_PATHS="$(ADDITIONAL_INCLUDE_PATHS)"

build: configure
	cd $(BUILD_DIR) && $(MAKE)

install: build
	cd $(BUILD_DIR) && $(MAKE) install DESTDIR=$(INSTALL_DIR)

help:
	@echo "Usage:"
	@echo "  make [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all        Build the project"
	@echo "  clean      Clean the build directory"
	@echo "  configure  Configure the project"
	@echo "  build      Build the project"
	@echo "  install    Install the project"
	@echo "  help       Display this help message"