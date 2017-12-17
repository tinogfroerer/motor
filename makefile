motor : motor.o
	c++ -std=c++11 motor.cpp -lpololu-tic-1 -o motor

motor.o : motor.cpp motor.h transformation.h estimator.h
	c++ -std=c++11 -L/usr/local/lib -c motor.cpp 

