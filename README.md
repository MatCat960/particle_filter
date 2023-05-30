# particle_filter

This repo contains a C++ library for Particle Filter Localization.
The library does not include ROS dependencies, but the package provides a ROS 2 wrapper node for ROS 2 integration.

## Dependencies

 - Eigen3

## ROS Wrapper Usage
### TODO



## Particle Filter steps
A particle filter is a recursive Bayesian state estimator that uses discrete particles to approximate the posterior distribution of an estimated state. Its algorithm is useful for online state estimation of a non-linear system according to the dynamic model of the robot and measurements taken (see [[1]](#1)).

Process and measurement noise distribution are also taken into account, resulting in the definition of a probability distribution of the real robot’s state. The procedure for particle filter state estimation is presented in details in [[2]](#2).

To summarize, we can define an initialization phase, followed by the recursive iteration over four steps: prediction, weight update, resampling, and state estimation.

 0. *Initialization*: A specific number of particles $P \in \mathbb{N}$ is generated according to a known distribution or uniformly distributed within the environment. Each particles represents an hypothesis of the state variables.
  ```math
  x(0)_{k} \sim p(x(0))
  ```  
In the above equation, $x(0)_k$ is the initial state of the generic particle $k$, $p(x(0))$ is the initial distribution and the operator $\sim$ is used to denote that the particle is randomly obtained from the probability distribution.
1. *Prediction*: The state of each particle is propagated following the state transition model of the system $f(x(t-1)_k, u(t))$, where $u(t)$ is the control input. The result is a new particles distribution.
```math
  x(t)_k = f(x(t-1)_k, u(t))
  ```  
2. *Weight update*: The likelihood of sensor measurements $y(t)$ is exploited to update the weight of each particle.
```math
  w(t)_k = p(y(t) | x(t)_k)
  ```  
3. *Resampling*: The particles are resampled according to their weights, in order to give more weight to particles that are more likely to match the observed data. This means that the new set of particles will be more concentrated in regions of the state space that have the highest probability.
4. *State Estimation*: The state estimate $\hat{x}(t)$ is calculated as a a weighted sum of particles.
```math
  \hat{x}(t) =  \frac{\sum w(t)_k x(t)_k}{\sum w(t)_k}
  ```  

Once the state estimate has been obtained, the algorithm jumps back at step 1 to start a new iteration.

### References

<a id="1">[1]</a>  Ristic, B., Arulampalam, S., & Gordon, N. (2003). _Beyond the Kalman filter: Particle filters for tracking applications_. Artech house.

<a id="2">[2]</a>  Arulampalam, M. S., Maskell, S., Gordon, N., & Clapp, T. (2002). A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. _IEEE Transactions on signal processing_, _50_(2), 174-188.
