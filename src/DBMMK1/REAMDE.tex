# Dynamic Bicycle Model Mk1 (DBMMK1)

**NOTE: This design is intended as a placeholder. Further on the proyect, other architectures should be considered**

**NOTE 2: Github kinda sucks balls so it wont display the matrixes at all. Just open this file with any good md formatter and youre golden**

## Overview
The **Dynamic Bicycle Model (DBM)** is a mathematical representation of a wheeled vehicle’s motion, considering lateral dynamics, yaw rate, and steering input. The model accounts for the time step between updates, making it suitable for implementation in **Extended Kalman Filters (EKF)** and other discrete-time state estimation algorithms.

## State Representation
The vehicle state is represented by the vector:

\[
\mathbf{x} =
\begin{bmatrix}
p_x \\ p_y \\ \varphi\\ u \\ v \\ \omega
\end{bmatrix}
\]

where:
- \(p_x, p_y\): Global position of the vehicle
- \(\varphi\): Heading angle (yaw)
- \(u, v\): Velocity components in the body frame (longitudinal & lateral)
- \(\omega\): Yaw rate (angular velocity)

## Control Inputs
The control input vector incorporates the time step \( dt \) explicitly:

\[
\mathbf{u} =
\begin{bmatrix}
a \\ \delta \\ \Delta t
\end{bmatrix}
\]

where:
- \(a_x\): Longitudinal acceleration (gas)
- \(\delta_f\): Front wheel steering angle
- \(\Delta t\): Time step (discretization interval)

## Measure Vector 
The measure vector is the same as the state vector except for the linear velocities. 

\[
\mathbf{x} =
\begin{bmatrix}
x \\ y \\ \varphi \\ \omega
\end{bmatrix}
\]

where:
- \(x, y\): Global position of the vehicle
- \(\varphi\): Heading angle (yaw)
- \(\omega\): Yaw rate (angular velocity)


## DBM Continuous-Time Equations
Im actually to tired to explain this. Refer to the first 2 pages of the [paper i *stole* the DBM from](https://arxiv.org/abs/2011.09612). More specifically, the first model described in segment **II** 

The similitudes with the paper end here. Please please pretty please familiriarize yourself with \(f, F_{y1}, F_{y2}\) before trying to understand what sort of divine whitchcraft is going on next.
What we did was expand this model to the following discrete(???) model

\(x_{k+1}\) = \(x_k + \Delta t*f(x_k, u_k)\)

The Jacobian of this model is calculated analitically as follows:

\[
\mathbf{F} = \mathbf{I} + \Delta t
\begin{bmatrix}
0 & 0 & -u\sin(\varphi) -v\cos(\varphi) & \cos(\varphi) & -\sin(\varphi) & 0 \\
0 & 0 & u\cos(\varphi) -v\sin(\varphi) & \sin(\varphi) & \cos(\varphi) & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & -\frac{1}{m}(\frac{\partial F_{y1}}{\partial u}\sin(\delta)) &
\omega-\frac{1}{m}(\frac{\partial F_{y1}}{\partial v}\sin(\delta)) &
v-\frac{1}{m}(\frac{\partial F_{y1}}{\partial \omega}\sin(\delta)) \\
0 & 0 & 0 & -\omega + \frac{1}{m}(\frac{\partial F_{y1}}{\partial u}\cos(\delta)+\frac{\partial F_{y2}}{\partial u}) &
\frac{1}{m}(\frac{\partial F_{y1}}{\partial v}\cos(\delta)+\frac{\partial F_{y2}}{\partial v}) &
-u+\frac{1}{m}(\frac{\partial F_{y1}}{\partial v}\cos(\delta)+\frac{\partial F_{y2}}{\partial v}) \\
0 & 0 & 0 & \frac{1}{I_z}(L_f\frac{\partial F_{y1}}{\partial u}\cos(\delta)-L_r\frac{\partial F_{y2}}{\partial u}) &
\frac{1}{I_z}(L_f\frac{\partial F_{y1}}{\partial v}\cos(\delta)-L_r\frac{\partial F_{y2}}{\partial v}) &
\frac{1}{I_z}(L_f\frac{\partial F_{y1}}{\partial \omega}\cos(\delta)-L_r\frac{\partial F_{y2}}{\partial \omega}) \\
\end{bmatrix}
\]

This unholy beast scares me and deprives me of rest at night. Unsurprisingly, its not over, since we still have to provide the partial derivatives of \(F_{y1}\) and \(F_{y_2}\). I am fully aware the next notation would instantly pulverize any mathematician so handle with care:

\[
\frac{\partial F_{y1}}{\partial (u, v, \omega)} =
\begin{bmatrix}
k_f(-\frac{v+L_f\omega}{u^2}) & k_f(\frac{1}{u}) & k_f(\frac{L_f}{u})
\end{bmatrix}
\]

and similarly

\[
\frac{\partial F_{y2}}{\partial (u, v, \omega)} =
\begin{bmatrix}
k_r(-\frac{v+L_r\omega}{u^2}) & k_r(\frac{1}{u}) & k_r(\frac{L_r}{u})
\end{bmatrix}
\]

Once again, im too lazy and hate `.md` too much to bother explaining the rest of the parameters. Luckily there explained in `Table 1` in the [almighty chinese paper](https://arxiv.org/abs/2011.09612) `(long live the cpp)`

The rest of the filter is a degenerated case of the **EKF** where the measure model is a partial of the state model

## Measure prediction
Since our measure vector is a subset of our state vector, the measure prediction function is a partial of the state model;

# IMPORTANT
My description does NOT make justice to the model its based on. Check the [SOURCE](https://arxiv.org/abs/2011.09612)
