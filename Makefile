.PHONY: all clean run

all: libhx711.so example

libhx711.so: hx711.cpp hx711.hpp
	g++ hx711.cpp -o libhx711.so -shared -O3 -g -flto -fPIC -rdynamic

clean:
	rm -vf libhx711.so example

example: example.cpp libhx711.so
	g++ example.cpp -o example -flto -O1 -g -fPIE -rdynamic -L . -l hx711

run: example
	LD_LIBRARY_PATH="$(PWD)" ./example
