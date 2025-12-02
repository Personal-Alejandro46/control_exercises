# Dynamic Matrices for 2-DOF Robotic Arm

This section describes how to implement the **inertia matrix** `D(q)` and the **Coriolis matrix** `C(q, q_dot)` for a **2-DOF planar robotic manipulator**.

---

## Inertia Matrix \( D(q) \)

The inertia matrix is defined by the following equations:

\[
\begin{aligned}
D_{11} &= I_{zz1} + I_{zz2} + l_{c1}^2 m_1 + l_1^2 m_2 + 2 \cos(\theta_2) l_1 l_{c2} m_2 + l_{c2}^2 m_2 \\
D_{12} &= I_{zz2} + \cos(\theta_2) l_1 l_{c2} m_2 + l_{c2}^2 m_2 \\
D_{21} &= I_{zz2} + \cos(\theta_2) l_1 l_{c2} m_2 + l_{c2}^2 m_2 \\
D_{22} &= I_{zz2} + l_{c2}^2 m_2
\end{aligned}
\]

### Python Implementation

```python
import numpy as np

# Define the inertia matrix D
D = np.zeros((2, 2))

D[0, 0] = Izz1 + Izz2 + lc1**2 * m1 + l1**2 * m2 + 2 * np.cos(t2) * l1 * lc2 * m2 + lc2**2 * m2
D[0, 1] = Izz2 + np.cos(t2) * l1 * lc2 * m2 + lc2**2 * m2
D[1, 0] = Izz2 + np.cos(t2) * l1 * lc2 * m2 + lc2**2 * m2
D[1, 1] = Izz2 + lc2**2 * m2
```
## Coriolis and Centrifugal Matrix \( C(q, \dot{q}) \)

The Coriolis and centrifugal terms are defined as:

\[
\begin{aligned}
C_{11} &= -2 \sin(\theta_2) \, l_1 \, l_{c2} \, m_2 \, \dot{\theta}_2 \\
C_{12} &= - \sin(\theta_2) \, l_1 \, l_{c2} \, m_2 \, \dot{\theta}_2 \\
C_{21} &= \sin(\theta_2) \, l_1 \, l_{c2} \, m_2 \, \dot{\theta}_1 \\
C_{22} &= 0
\end{aligned}
\]

### Python Implementation

```python
import numpy as np

# Define the Coriolis and centrifugal matrix C
C = np.zeros((2, 2))

C[0, 0] = -2 * np.sin(t2) * l1 * lc2 * m2 * t2_dot
C[0, 1] = -np.sin(t2) * l1 * lc2 * m2 * t2_dot
C[1, 0] =  np.sin(t2) * l1 * lc2 * m2 * t1_dot
C[1, 1] = 0
```

# ⚡ Actuator Dynamics

In the previous section, we presented the mathematical models of some robots. Now that you know what these systems’ equations look like (it may seem abstract, but control algorithms really require you to understand them), we will explain **how the input torque vector \( \tau \)** of Equation (1) is physically generated.

---

## 🔩 How is the torque \( \tau \) generated?

Before starting, it’s important to emphasize that a **DC motor** (as shown in the figure) is usually modeled as an **RL circuit**, as shown below.

1. The **input voltage \( V(t) \)** is applied to the DC motor.  
   This voltage is typically controlled by **H-bridge circuits**, which are in turn controlled by **PWM (Pulse Width Modulation)** signals.

2. The voltage input generates a **back electromotive force \( V_b \)** and a **current** through the motor.

3. The current and voltage in the motor induce a **torque** and cause changes in the **motor’s angular position**.

4. The generated torque and angular position are **amplified and reduced**, respectively, through **mechanical transmission systems** (chains, belts, or gears).

5. The **output torque** acts on the **robot’s joint**, producing the **joint angles \( q \)** we want to control.

---

## DC Motor Dynamics

After applying Kirchhoff’s laws and electromechanical relations, we obtain the following relationship between the motor voltage \( V(t) \), the load torque \( \tau_{load} \), and the motor angle \( \theta_m \):

\[
J_m \ddot{\theta}_m + \left(B_m + \frac{K_b K_m}{R_m}\right)\dot{\theta}_m = \frac{K_m}{R_m}V_k - r \cdot \tau_{load}
\tag{1}
\]

Where:
- \( J_m \): motor inertia  
- \( B_m \): joint friction coefficient  
- \( \frac{K_b K_m}{R_m} \): internal motor friction  
- \( \frac{K_m}{R_m} \): gain that converts voltage to torque  

If the load is the robot arm link:

\[
\tau_{load} = \tau_k, \quad q_k = r \cdot \theta_m
\]

Here, \( q_k \) is the **link angle** we want to control, and \( r \) is the **mechanical reduction ratio**.  
If \( r = 1 \), the motor is directly connected to the link.

---

### Note:
If there are additional reductions (belts, pulleys, chains, etc.), then:
\[
\tau_{load} \neq \tau_k, \quad q_k \neq r \cdot \theta_m
\]
and a more complex analysis is required.

---

##  Combined Dynamic Model

The robot dynamics can be expressed as:

\[
D(q)\ddot{q} + C(q, \dot{q}) + g(q) = \tau =
\begin{bmatrix}
\tau_1 \\
\tau_2 \\
\vdots \\
\tau_n
\end{bmatrix}
\tag{2}
\]

and the joint angles as:

\[
q =
\begin{bmatrix}
q_1 \\
q_2 \\
\vdots \\
q_n
\end{bmatrix}
=
\begin{bmatrix}
r_1 \theta_{m1} \\
r_2 \theta_{m2} \\
\vdots \\
r_n \theta_{mn}
\end{bmatrix}
\tag{3}
\]

---

## DC Motor Model for Each Joint

For each motor \( k \), the equation becomes:

\[
J_{mk} \ddot{\theta}_{mk} + \left(B_{mk} + \frac{K_{bk} K_{mk}}{R_{mk}}\right)\dot{\theta}_{mk} = \frac{K_{mk}}{R_{mk}}V_k - r_k \tau_k
\quad (k = 1,2,\dots,n)
\tag{4}
\]

Define the following constants:

\[
\begin{aligned}
J_{effk} &= J_{mk} &\quad \text{(effective inertia)} \\
B_{effk} &= B_{mk} + \frac{K_{bk} K_{mk}}{R_{mk}} &\quad \text{(effective friction)} \\
K_{effk} &= \frac{K_{mk}}{R_{mk}} &\quad \text{(voltage-torque gain)} \\
d_k &= \tau_k &\quad \text{(disturbance torque)}
\end{aligned}
\tag{5–8}
\]

Then, the DC motor model simplifies to:

\[
J_{effk} \ddot{\theta}_{mk} + B_{effk}\dot{\theta}_{mk} = K_{effk}V_k - r_k d_k
\quad (k = 1,2,\dots,n)
\tag{9}
\]

---

## Initialization of Simulation Parameters

```python
dt = 0.01  # step time for the simulation

m1, m2 = 1.0, 1.0  # masses of links 1 and 2
l1, l2 = 1.0, 1.0  # lengths of links 1 and 2
lc1, lc2 = 0.5, 0.5  # distances to center of mass
Izz1 = (1.0 / 12.0) * (0.1 * 0.01 + l1**2)
Izz2 = (1.0 / 12.0) * (0.1 * 0.01 + l2**2)

grav = 9.81  # gravity

Jeff1 = Izz1 + m1 * lc1**2
Jeff2 = Izz2 + m2 * lc2**2

K1, K2 = 0.1, 0.1  # voltage-torque gain
Beff1, Beff2 = 0.001, 0.001  # friction coefficients
red1, red2 = 1, 1  # reduction ratios
```

## Dynamic Simulation
```
# inertia matrix
D = D_robot(m1, m2, l1, l2, lc1, lc2, Izz1, Izz2, q)
# coriolis term
C = C_robot(m1, m2, l1, l2, lc1, lc2, Izz1, Izz2, q, dq)
# gravity vector
g = g_robot(m1, m2, l1, l2, lc1, lc2, Izz1, Izz2, q, grav)

# matrix of inertias
JM = np.diagflat(np.array([(Jeff1) / (red1**2), (Jeff2) / (red2**2)]))
# matrix of frictions
BM = np.diagflat(np.array([(Beff1) / (red1**2), (Beff2) / (red2**2)]))

# dynamic simulation - equation (1)
ddq = np.linalg.inv(D + JM) @ (input - C @ dq - BM @ q - g)

# update angular velocities (derivative approximation)
dq = dq + dt * ddq
# update joint angles
q = q + dt * dq
```

Hmm, surely all makes some of sense, except dt, how about that? Colloquialy, it is the time to update your simulation (it is impossible for us to simulate a continuous world without losing some precision).
- Given a small dt, we will have more precision in the simulation, in counterpart it will be too slow.
- Given a big dt, we will have less precision n in the simulation, in counterpart it will be so fast.

## Derivative Approximation

Since computing derivatives is computationally expensive, we use the **forward finite difference method** to approximate them:

\[
\dot{q} \approx \frac{q(t + dt) - q(t)}{dt}
\tag{10}
\]

This is the **numerical method** used to update the **velocities** and **positions** of the joints at each simulation step.

---

### Control Goal

The main control objective is to:

> **Control the joint angles \( q_1, q_2, \dots, q_n \)**  
> using the **motor voltages \( V_1, V_2, \dots, V_n \)** as inputs.

The relationship between these variables can be summarized as:

\[
\text{Voltage} \; \rightarrow \; \text{Torque} \; \rightarrow \; \text{Arm Angles}
\]


## Derivative Control

Yes, "rate of change" — surely you are thinking this control is formulated with a derivative.  
The mathematical form of the **derivative control** is given by:

\[
u = K_D \cdot (\dot{x}_{setpoint} - \dot{x})
\tag{1}
\]

Where:
- \( K_D \) is the **derivative gain**,
- \( x_{setpoint} \) is the desired value of \( x \),
- \( x \) is the actual state of the dynamical system.

---

### Time Domain and Derivative Notation

It is important to emphasize that in equation (1) the domain of the variables is **time**.  
The “dot” over \( x \) ( \( \dot{x} \) ) denotes the time derivative, which is defined as:

\[
\dot{x} = \frac{dx(t)}{dt}
\tag{2}
\]

This notation is shorter than writing \( \frac{dx(t)}{dt} \), so throughout this course, we will use the **dot notation** ( \( \dot{x} \) ) to represent the derivative.

---

### Expanded Form

The derivative control can also be written as:

\[
u(t) = K_D \cdot \left( \frac{d x_{setpoint}(t)}{dt} - \frac{d x(t)}{dt} \right)
\tag{3}
\]

---

## Discrete-Time Implementation

Before proceeding to the exercise, an important question is:  
**How do we actually implement a derivative in discrete time?**

This can be answered from the definition of the derivative:

\[
\dot{x} = \lim_{T_s \to 0} \frac{x(t + T_s) - x(t)}{T_s}
\tag{4}
\]

Usually, the letter \( h \) is used instead of \( T_s \), but here we denote \( T_s \) as the **sample time**.

Instead of taking the limit as \( T_s \to 0 \), we approximate \( T_s \) with a small fixed value.  
Thus, we obtain the discrete-time approximation:

\[
\dot{x} \approx \frac{x(t + T_s) - x(t)}{T_s}
\tag{5}
\]

This simple formula is extremely useful — it is widely used to simulate and implement control laws in **digital systems**.

---

### Updating State Values

By solving (5) for \( x(t + T_s) \), we obtain:

\[
x(t + T_s) := x(t) + T_s \cdot \dot{x}
\tag{6}
\]

The symbol “:=” means assignment, i.e., the value \( x(t) + T_s \cdot \dot{x} \) is assigned to the variable \( x(t + T_s) \).

There are more advanced approximations for derivatives (e.g., **Runge-Kutta**), but all of them originate from this basic definition.

---

### Sample Time

In simulations, the sample time \( T_s \) is the same as `dt` used in the code.

The following lines simulate the robot dynamics using this discrete derivative approximation:

```python
# update angular velocities using derivative approximation
dq = dq + dt * ddq
# update joint angles using derivative approximation
q = q + dt * dq


## Derivative Control in a 2-DOF Robotic Arm

The derivative control can be applied to both links of the robot arm using the following control laws:

\[
V_1(t) = K_{D1} \, [ \dot{\theta}_{d1} - \dot{\theta}_1 ]
\tag{7}
\]

\[
V_2(t) = K_{D2} \, [ \dot{\theta}_{d2} - \dot{\theta}_2 ]
\tag{8}
\]

---

### Where:

- \( V_1, V_2 \): input voltages applied to motors 1 and 2  
- \( K_{D1}, K_{D2} \): derivative gains  
- \( \dot{\theta}_{d1}, \dot{\theta}_{d2} \): desired angular velocities  
- \( \dot{\theta}_1, \dot{\theta}_2 \): actual angular velocities

---