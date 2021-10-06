main: main.cpp FMII.h
	g++ -O2 -std=c++11 -I/usr/include/eigen3 main.cpp -I/usr/include/python3.8 -lpython3.8 -o main