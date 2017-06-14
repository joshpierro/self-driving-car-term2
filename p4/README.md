# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Background and purpose
This repo contains my submission for Udacity SDC nanodegree PID Controller project. The PID controller is a common, simple, effective and widely accepted technique for actuating autonomous vehicles. This implementation was built in C++ and the solution was based on the lectures in the PID control module. 

## P-I-D components and their effects

### Proportional Component (P)
The proportional component allows the car to steer proportional to the car's distance from the center of the lane, or cross track error (CTE). It determines the speed and resilience of a system to respond to input. 

### Integral Component (I)  
The integral component is responsible for mitigating biases and prolonged error in the CTE. These biases may include, but are not limited to systematic errors in hardware and software, steering drift, centrifugal pull around large turns and other externalities like gusts of wind.  

### Differential Component (D) 
The Differential component prevents overshooting and provides smoothing when correcting by calculating the derivative of the CTE. This prevents the car from over-oscillating or over-correcting, which would result in a wobbly experience.

## Approach and Hyper-parameter Selection
My approach for this project was to get it working end to end and then fine tune my hyper-parameters and methods. I first implemented the provided method stubs (Init,  UpdateError, TotalError) and got the main function working. In main, I created two PID controllers, one for controlling the steering and one for controlling the throttle. My final parameters were derived with trial and error. 

For my steering controller, I started with an arbitrary and high value (1) for my p parameter. I gradually reduced this to .3 to mitigate over-steering and correcting. I also started with a value of 1 for my D parameter and I gradually raised it to 4, which decreased and smoothed out the wobbliness of the run. The I parameter had minimal impact on the simulation and I settled on .0005. 

In my throttle controller I was never able to calculate consistent and reliable values to send to the simulator through parameter tuning. So, in the end, I ended up coming up with a ham fisted solution that classified the values into buckets and those buckets were used to control the throttle. 

<pre>
       if(throttle_value < .1){
            throttle_value = 0.3;
          }
          else if(throttle_value < .2){
            throttle_value = 0.2;
          }else{
            throttle_value = 0.1;
          }
</pre>

My final parameters are as follows:
<pre>
  Steering (P=0.3,I=.0005,D=4);
  Throttle (P=1,I=0,D=0.5);
</pre> 




