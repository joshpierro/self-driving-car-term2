# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Background and Purpose
This repo contains my submission for the Udacity SDC nanodegree Model Predictive Control (MPC) project. MPC is an advanced technique and optimization approach to actuate autonomous vehicles. It uses a mathematical dynamics process model to predict and optimize the future behavior of systems [1]. This implementation was built in C++ and tries to calculate an optimal trajectory for the car simulator to follow around the track. My solution was based on the MPC lectures and was heavily influenced by the <a href='https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be'> Self-Driving Car Project Q&A | MPC Controller video</a>.


### Model Description 
This implementation uses a Kinetic model, which is a simplification of a dynamic vehicle model. It uses model state and output from each time-step to calculate the next, as well as plot a trajectory. Model inputs include: vehicle location (x,y), orientation (psi), velocity (v), cross track error (cte), error of psi, as well as acceleration (a) and steering angle (delta). Acceleration and steering angle are used as actuators to propel and guide the car. The update step employs the following equation:

<br>
<img src="https://github.com/joshpierro/self-driving-car-term2/blob/master/p5/images/update.png"/>
<br>

### Timestep Length and Elapsed Duration Rationale
On the suggestion of the Udacity office hours - MPC Q&A, I selected the values 10 for N and .1 for dt. This represents the number of time steps (N) and duration of a time-step (dt), and together they define the duration of time where the model can predict the trajectory. 

<pre>
    //one second into the future
    size_t N = 10;
    double dt = .1;
</pre>

These values seemed to work very well, so I never experimented with them. 

### Polynomial Fitting and MPC Preprocessing Discussion 
At each step, a bit of per-processing was performed on the car's location data (waypoints) before a polynomial was calculated. First the values of x,y,and psi were normalized, in order to simplify the math in the polynomial calculation. Next the x and y vectors had to be cast into VectorXd collection, so they could be used in the provided polynomial method (polyfit()). 

<pre>
     //convert vector double to VectorXd for polyfit function
      Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
      Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
</pre>
 
 Polyfit() returns the coefficients of a third order polynomial. 
 
 <pre>
     auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
 </pre> 
 
 
### MPC Latency
The purpose of latency in the model is to simulate real world driving conditions where the car does respond to commands instantly, as well as the latency between getting sensor data and processing it. This value was left at the recommended 100 ms. 

### Other Findings - Parameter Tuning - Conclusion.  
I was pleasantly surprised to find that I could easily implement and end to end solution (where the car successfully traveled around the track) using the lecture material and by paying close attention to the MPC Q&A session. Further performance was gained by tweaking the cost functions and increasing the velocity parameter (to 200). In the end, I was able to get the car over 90 MPH! 

### Final Result
Click the image below to see a video of my final result.

<a href="https://youtu.be/6SooeFUN2fM" target="_blank">
<img src="https://github.com/joshpierro/self-driving-car-term2/blob/master/p5/images/96mph.png"/>
</a>
<br>

[1] Study of Model Predictive Control for Path-Following Autonomous Ground Vehicle Control under Crosswind Effect - https://www.hindawi.com/journals/jcse/2016/6752671/
