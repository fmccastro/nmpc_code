# MSc thesis code

This repo contains the code that supports the development of a master thesis on the development of a **real-time nonlinear model predictive control based method to safely guide and control a wheeled mobile robot on rough, uneven terrain**. **Optimal path planning** methods are also explored. It is intended to provide a fast review of the usefulness of this work to researchers.

The [acados](https://github.com/acados/acados) library provided key methods to effectively deliver control algorithms that meet stringent real-time requirements. On the other hand, [CasADi](https://github.com/casadi/casadi) provided efficient symbolic nonlinear constrained optimization methods to generate optimal path planning algorithms.

The full thesis document can be found [here](https://github.com/fmccastro/mscthesis_pdf/blob/22f496074fc8c4963af305fde6342fcd7fdb02b9/Thesis.pdf).

`Python3.8` is the code based language.

##	Authors
1. Francisco Castro
2. Prof. Rodrigo Ventura (supervisor)

##	Content
The structure of this repo is similar to a ROS package as the first intention of this work was to make use of ROS-Noetic and Gazebo to output results. Due to infeasibililty of the Gazebo physics-engine to appropriately simulate the traversal of a wheeled mobile robot on rough, rigid terrain, the author resorted to Python simulation in order to achieve the work's main goal. Then, the main code files of the repo are outlined as follows:

1. [Configuration files](https://github.com/fmccastro/nmpc_code/tree/0f28a1b68c4998d8ebb971e4df233e82e0493463/nmpc_applications/config):
   *These files include the robot specs, the cost function weights, model constraints and solver specs. The obstacles shape file is also defined here.*
   
2. [Simulation scripts](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts): *This folder contains the scripts that yielded the results presented in the thesis. The files with no folder attached generated the optimal path planning results. Then, the folder [Real-time one stage OCP on rough terrain](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts/Real-time%20one%20stage%20OCP%20on%20rough%20terrain) contains the scripts that simulate the controller reference tracking on rough terrain with and without obstacles. The folder [Obstacle avoidance](https://github.com/fmccastro/nmpc_code/tree/c45579955c95297d7a88256a15d47549140c6684/nmpc_applications/scripts/Obstacle%20avoidance) contains the test files that generate the artificial potential field and check the proper implementation of the [CIAO](https://ieeexplore.ieee.org/abstract/document/9197206) algorithm.*

3. [Classes](https://github.com/fmccastro/nmpc_code/tree/2c90244cca1f40b8f738b991e24b3a8614588505/nmpc_applications/src/classes): *This folder contains the methods that each script calls to properly output trustworthy results. The source code of the algorithms developed in this thesis is located here.*

##  Main results

### Optimal path planning

1. Generation of traversability maps

<div align="center">
	<table>
		<tr>
	    	<td valign="middle">
				<b>Height minimization nonlinear program</b>
				<p></p>
				<table>
				  <tr>
				    <td style="text-align: right; padding-right: 10px;">
						Minimize
					</td>
				    <td>$$J = z$$</td>
				  </tr>
				  <tr>
				    <td style="text-align: right; padding-right: 10px;">
						w.r.t
					</td>
				    <td>$$\mathbf{w} = (z, \phi, \theta)$$</td>
				  </tr>
				  <tr>
				    <td style="text-align: right; padding-right: 10px;">
						subject to
					</td>
				    <td>$$\forall_k: z_k - h \left( x_k, y_k \right) \ge 0$$</td>
				  </tr>
				  <tr>
				    <td></td>
				    <td>$$\mathbf{\Psi}_{\max} - \left|\mathbf{\Psi}\right| \ge 0$$</td>
				  </tr>
				</table>
				<p></p>
				<p></p>
				<b>Terrain roughness computation</b>
				<p></p>
				$$\bar{h}_d \left( i, j \right) = \frac{ \sum_{ x_i,\,y_j \in \mathcal{H}_2 } h \left( x_i, \, y_j \right) }{ n \left( \underline{ \mathcal{H} }_2 \right) },\; \underline{ \mathcal{H} }_2 :=  \forall \left( x_i,\, y_j \right) \in \mathcal{H}_2$$
				<p></p>
				$$\sqrt{ \frac{ \sum_{ x_i, y_j \in \mathcal{H}_2 } \left( h_d \left( i,\,j \right) - \bar{h}_d \left( i, j \right) \right)^2 }{ n \left( \underline{ \mathcal{H} }_2 \right) } }$$
			</td>
	  		<td>
				Refined worst-case traversability map example
	    		<img src="https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/mapRefinement%2BPoints%2B0.9%2B0.2%2B1.png" width="500">
	  		</td>
		</tr>
	</table>
</div>

2. Generation of potential flows by applying the Eikonal equation (solution computed with ![skfmm](https://github.com/scikit-fmm/scikit-fmm.git))

	(1a) $$| \nabla u \left( x, y \right) | F\left(x, y\right) = 1$$
	<p></p>
	(1b) $$u(\Gamma) = 0$$, such that $$(x, y), \Gamma \in \mathcal{M}_2$$

	-![Example of potential flow generated with the Eikonal equation.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/potentialFlow%2BmapRefinement%2BPoints%2B0.9%2B0.2.pdf)

	-![Comparison among paths generated with different types of maps.](https://github.com/fmccastro/mscthesis_pdf/blob/7b91f8b9ee3b3ccbd17daf7faa5a1f8c68620eb5/Figures/comparisonOfPaths.pdf)

### Real-time safe nonlinear optimal control of a mobile robot on rough terrain

#### Solver definition

$$l \left( \mathbf{x}, \mathbf{u}, \mathbf{r}_{def}, s \right)_k = \lVert x - r_{x, def}, y - r_{y, def}, e_{\psi}, v_x, \omega_z, \delta t_l, \delta t_r \rVert_{Q, k}^2 + \lVert u_l, u_r \rVert_{R, k}^2 + \frac{1}{2} s_k^2 Z_k + s_k z_k$$
$$m \left( \mathbf{x}, \mathbf{r}_{def}, s\right)_k = \Vert x - r_{x,def},y - r_{y,def}, e_{\psi}, v_x, \omega_z, \delta t_l, \delta t_r \Vert_{Q, k}^2 + \frac{1}{2} s_k^2 Z_k + s_k z_k$$
$$\underset{\mathbf{x}_k, \mathbf{u}_k, s_k}{\text{minimize}} \sum_{k=0}^{N-1} l\left(\mathbf{x}_k,\mathbf{u}_k,\mathbf{r}_{k,def}, s_k\right) + m\left(\mathbf{x}_N, \mathbf{r}_{N, def}, s_N\right)$$
$$\mathbf{x}_0 = \mathbf{\hat{x}}_0$$
$$\mathbf{\dot{x}}(t) = \mathbf{f}\left(\mathbf{x}(t), \mathbf{u}(t), \mathbf{p}\right)$$
$$\left(m g \sin(\theta_k) \frac{1}{4} + \delta t_{l, k}\right)^2 + \left(m g \cos(\theta_k) \sin(\phi_k) \frac{1}{4}\right)^2 - \left(\mu m g \cos(\theta_k) \cos(\phi_k) \frac{1}{4}\right)^2 \leq 0$$
$$\left(m g \sin(\theta_k) \frac{1}{4} + \delta t_{r, k}\right)^2 + \left(m g \cos(\theta_k) \sin(\phi_k) \frac{1}{4}\right)^2 - \left(\mu m g \cos(\theta_k) \cos(\phi_k) \frac{1}{4}\right)^2 \leq 0$$
$$\lVert \mathbf{p}_k - \mathbf{c}_k^* \rVert_2^2 - \left( d_\mathcal{O} \left( \mathbf{c}_k^* \right) - \underline{d} \right)^2 \leq s_k$$
$$s_k \geq 0$$
$$\begin{pmatrix}
(v_x)_{min}\\
(\omega_z)_{min}
\end{pmatrix} \leq \begin{pmatrix}
v_x\\
\omega_z
\end{pmatrix} \leq \begin{pmatrix}
(v_x)_{max}\\
(\omega_z)_{max}
\end{pmatrix}$$

#### Path tracking results

An example of the application of the above solver to path tracking is presented [here](https://github.com/fmccastro/mscthesis_pdf/blob/22f496074fc8c4963af305fde6342fcd7fdb02b9/Figures/Obstacle%20avoidance/Safe%20NMPC/Several_paths.pdf). Regardless of the initial conditions, the mobile robot is always at a minimum distance $\underline{d}$ from the closest obstacle. Slack variables, $s_k$, provide some flexibility to the solver in order to avoid infeasibility, specially when the safe balls radius size is small. Then, when the mobile robot has no other choice than briefly enter unsafe space, the slack variables increase. A [video](https://www.youtube.com/watch?v=xHckwLaS_dQ&t=421s) was prepared to showcase the states, controls and solver performance evolution during the traversal of rough terrain on an environment filled with obstacles, with the solver described above.

## Teleoperation node (miscellaneous)
A teleoperation node for the [mobile robot](https://github.com/fmccastro/nmpc_code/blob/ab60baa7eb3b822d2609c6c4235bec53f61a5c24/nmpc_description/robots/pioneer3at/urdf/pioneer3at.urdf) is coded [here](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelTorques.py). You can take the code and adjust to your needs. This approach allows you to easily teleoperate a wheeled mobile robot from a PyGame GUI by mouse control, resembling a joystick.
This first teleoperation node controls wheel torques, whereas this second [teleoperation node](https://github.com/fmccastro/nmpc_code/blob/913261a1084ce2b29de2d6007a0622224d8becd1/nmpc_applications/src/mouse_joy_wheelRates.py) controls wheel rates.

[![Wheel torque teleoperation video demonstration](https://raw.githubusercontent.com/fmccastro/nmpc_code/master/assets/thumbnail.jpg)](https://youtu.be/Sgt95OHfLIY)
