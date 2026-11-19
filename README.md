# Two-Link Planar Arm: Kinematics in MATLAB

Forward kinematics, Jacobian analysis, and algebraic inverse kinematics for a two-link planar robot arm, implemented in MATLAB and paired with a SolidWorks model of the mechanism.

The user enters a target point `(x, y)`. The program checks that the point is reachable, computes the joint angles, and animates the arm moving from its starting pose to the target.

## Features

- Symbolic forward kinematics using homogeneous transformation matrices
- Symbolic Jacobian and its determinant, used to locate singular configurations
- Algebraic inverse kinematics, packaged as the reusable function `ik.m`
- Reachability check that rejects points outside the arm's workspace
- Animation of the arm moving to the requested target
- SolidWorks parts and assembly of the mechanism

## Repository contents

| File | Description |
|---|---|
| `robot_arm.mlx` | Live Script: derivation of the kinematics and Jacobian, user input, and animation |
| `ik.m` | Function returning the joint angles for a target point |
| `Base.SLDPRT`, `Link1.SLDPRT`, `Link2.SLDPRT`, `Shaft.SLDPRT` | SolidWorks parts |
| `Assembly.SLDASM` | SolidWorks assembly of the arm |

## Getting started

**Requirements**

- MATLAB with the Symbolic Math Toolbox
- SolidWorks 2021 or later (only needed to open the CAD files)

**Run**

1. Download or clone this repository and keep all files in the same folder, so that `robot_arm.mlx` can find `ik.m`.
2. Open `robot_arm.mlx` in MATLAB and run it.
3. When prompted, enter the `x` and `y` coordinates of the target point.

The link lengths are set near the top of the script (default `l1 = 1`, `l2 = 0.7`). In the CAD model, one unit corresponds to 100 mm.

## Mathematics

Let `a1` and `a2` be the link lengths and `θ1`, `θ2` the joint angles.

**Forward kinematics**

$$x = a_1\cos\theta_1 + a_2\cos(\theta_1+\theta_2)$$

$$y = a_1\sin\theta_1 + a_2\sin(\theta_1+\theta_2)$$

**Jacobian**

$$J = \begin{pmatrix} -a_1\sin\theta_1 - a_2\sin(\theta_1+\theta_2) & -a_2\sin(\theta_1+\theta_2) \\ a_1\cos\theta_1 + a_2\cos(\theta_1+\theta_2) & a_2\cos(\theta_1+\theta_2) \end{pmatrix}$$

$$\det J = a_1 a_2 \sin\theta_2$$

The determinant vanishes at `θ2 = 0` or `θ2 = π`, when the arm is fully stretched or folded back on itself. These are the singular configurations.

**Inverse kinematics (algebraic)**

$$\cos\theta_2 = \frac{x^2 + y^2 - a_1^2 - a_2^2}{2a_1a_2}, \qquad \sin\theta_2 = -\sqrt{1-\cos^2\theta_2}$$

$$k_1 = a_1 + a_2\cos\theta_2, \qquad k_2 = a_2\sin\theta_2$$

$$\theta_1 = \operatorname{atan2}(k_1 y - k_2 x,\; k_1 x + k_2 y)$$

A target is reachable only if `|a1 - a2| <= sqrt(x² + y²) <= a1 + a2`. The result was verified by feeding the computed angles back into the forward equations, which returns the requested point.

## Limitations

- The inverse solution uses a single elbow configuration (negative `θ2`). The other configuration is obtained by taking the positive root for `sin θ2`.
- The animation interpolates the joint angles linearly, so the end effector does not travel in a straight line to the target.
- The SolidWorks assembly models the geometry and revolute joints of the arm. It is not linked to MATLAB, and its joints are not driven by the computed angles.

## Roadmap

- Trace trajectories such as circles and squares with the end effector
- Add velocity control using the Jacobian
- Derive the arm dynamics
- Drive the CAD model directly from MATLAB
