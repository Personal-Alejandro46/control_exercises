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