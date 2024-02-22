# ekf
Python Implementation of the Extended Kalman Filter (EKF) in a Robot Localisation Problem Simulation - Probabilistic Robotics

This is my implementation of the Extended Kalman Filter (EKF) in Python. The EKF is implemented to estimate the state of a robot in a simulation navigating an environment. The purpose of this ekf implementation is to enhance the accuracy of robot localization by taking into account uncertainties.

"A robot that carries a notion of its own uncertainty and that acts accordingly is superior to one that does not." - S. Thrun, W. Burgard and D. Fox, ‘Probabilistic Robotics’, MIT press, 2005.

I also visualised:
  a.	The true (ground-truth) trajectory (x_true), the observation trajectory (z) and the estimated trajectory (x_est).
  b.	The estimated covariance ellipse at each current estimated state. 

The following graph is generated from my code:
<img width="1456" alt="Screenshot 2024-02-22 at 02 24 15" src="https://github.com/alina-ahmed-tech/ekf/assets/130942761/c5e04f53-a609-4db9-9317-ae8753766bdd">

Where:
  •	The orange ellipses represent the estimated covariance ellipses at each current estimated state. 
  •	The blue plot represents the (ground-truth) true trajectory.
  •	The red plot represents the estimated trajectory.
  •	The green plot represents the observation trajectory. 
  •	The x-axis corresponds to the X position of the robot.
  •	The y-axis corresponds to Y position of the robot.
