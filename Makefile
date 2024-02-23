main: clean mkdir
	g++ main.cpp -o build/main

mkdir:
	mkdir -p build

clean: mkdir
	rm -r build/*