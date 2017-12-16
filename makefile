motor : motor.cpp motor.h transformation.h estimator.h
	c++ -std=c++11 -I/usr/local/include -lpololu-tic-1 motor.cpp -o motor
