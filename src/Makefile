#CXX := g++
CXX := arm-unknown-linux-gnueabi-g++
CXXFLAGS := -fPIC -O3 -g -Wall -Werror -pg
MAJOR := 0
MINOR := 1
NAME := I2C
VERSION := $(MAJOR).$(MINOR)

lib: lib$(NAME).so.$(VERSION) lib$(NAME).so

lib$(NAME).so: lib$(NAME).so.$(VERSION)
        ldconfig -v -n .
        ln -s lib$(NAME).so.$(MAJOR) lib$(NAME).so

lib$(NAME).so.$(VERSION): $(NAME)Device.o
        $(CXX) $(CXXFLAGS) -shared -Wl,-soname,lib$(NAME).so.$(MAJOR) $^ -o $@

clean:
        $(RM)  *.o *.so*
