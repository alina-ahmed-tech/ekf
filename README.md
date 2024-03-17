# ekf
Python Implementation of the Extended Kalman Filter (EKF) in a Robot Localisation Problem Simulation - Probabilistic Robotics

This is my implementation of the Extended Kalman Filter (EKF) in Python. The EKF is implemented to estimate the state of a robot in a simulation navigating an environment. The purpose of this ekf implementation is to enhance the accuracy of robot localization by taking into account uncertainties.

"A robot that carries a notion of its own uncertainty and that acts accordingly is superior to one that does not." - S. Thrun, W. Burgard and D. Fox, ‘Probabilistic Robotics’, MIT press, 2005.

I also visualised:<br>
  a.	The true (ground-truth) trajectory (x_true), the observation trajectory (z) and the estimated trajectory (x_est).<br>
  b.	The estimated covariance ellipse at each current estimated state. <br>

The following graph is generated from my code:
<img width="1487" alt="Screenshot 2024-02-22 at 02 34 22" src="https://github.com/alina-ahmed-tech/ekf/assets/130942761/eda852c9-2632-4da9-b3fc-4487cbd56bfb">


Where: <br>
  •	The orange ellipses represent the estimated covariance ellipses at each current estimated state. <br>
  •	The blue plot represents the (ground-truth) true trajectory. <br>
  •	The red plot represents the estimated trajectory. <br>
  •	The green plot represents the observation trajectory. <br>
  •	The x-axis corresponds to the X position of the robot. <br>
  •	The y-axis corresponds to Y position of the robot. <br>
  <br>
-------------------------------------- DETAILS -------------------------------------------
![Slide2](https://github.com/alina-ahmed-tech/ekf/assets/130942761/6e2186fe-6318-4317-9a3b-3dc36d2cdb42) <br>
![Slide3](https://github.com/alina-ahmed-tech/ekf/assets/130942761/95fe5492-d5ee-44bb-8527-fecb0085bfa1)<br>
![Slide4](https://github.com/alina-ahmed-tech/ekf/assets/130942761/a973b765-ba6a-4fbf-a7d7-25b413d9d429) <br>
![Slide5](https://github.com/alina-ahmed-tech/ekf/assets/130942761/c4575687-c187-4eeb-97f5-25cc0a883d2a) <br>


