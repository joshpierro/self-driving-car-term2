# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Background and purpose
This repo contains my submission for Udacity SDC nanodegree PID Controller project. The PID controller is a common simple, effective and accepted technique for actuating autonomous vehicles. This implementation was built in C++ and was based on the lectures in the PID control module. 

## P-I-D components and their effects

### Proportional Component (P)
The proportional component allows the car to steer proportional to the car's distance from the center of the lane, or cross track error (CTE). It determines the speed and resilliance of a system to respond to input. 

### Integral Component (I)  
The integral componet is responsible for mitigating biases and prolonged error in the CTE. These biases may include, but are not limited to systematic errors in hardware and software, steering drift, centrifugal pull around large turns and other externalities like gusts of wind.  

### Differential Component (D) 
The Differential component prevents overshooting and provides smoothing when correcting by calculating the derivative of the CTE. This prevents the car from over-oscillating or over-correcting, which would result in a wobbly experience.

### Hyperparameter Selection and Approach 
