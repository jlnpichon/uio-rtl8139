all:
	cd kmod && make
	cd user && make

clean:
	cd kmod && make clean
	cd user && make clean
