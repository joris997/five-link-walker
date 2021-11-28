# five-link-walker

![out](https://user-images.githubusercontent.com/56355937/143733180-be915d45-0b1b-411a-adb6-ae3bd71dca72.gif)

Stable locomotion using IO linearization, CLFs, and TSC. This was initially made to get a better understanding of bipedal robotic locomotion and get familiar with some of the 'standard' basic control approaches to  5-link walkers. These 5-link walkers are close to being the easiest, morphologically sound walker. For a detailed description of the biped in question, please refer to [1] which is well documented

![Screenshot from 2021-11-26 14-44-58](https://user-images.githubusercontent.com/56355937/143659579-b9a3b0bb-2a8c-4a35-99f0-e7492538fdec.png)

The 5-link walker code ('main.m') allows for setting a down-step height to analyze some heuristic version of the robustness of the control strategies, affecting by controller gains and other decision variables. Nothing too formal though but fun to play around with!

## Usage:
Just run 'main.m'. You can change the controller type before the main loop by changing the string 'controller' on line 100. If you wish to make changes to the underlying biped model, make the changes in 'generate.m' and run it again (this will take a long time with symbolic equations and simplyfing them!).

For the CLF and TSC controller, you will need [CVX](http://cvxr.com/cvx/).

## System of equations
We consider a standard system of equations.

<a href="https://www.codecogs.com/eqnedit.php?latex=\dot{x}&space;=&space;f(x)&space;&plus;&space;g(x)u" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dot{x}&space;=&space;f(x)&space;&plus;&space;g(x)u" title="\dot{x} = f(x) + g(x)u" /></a>

where

<a href="https://www.codecogs.com/eqnedit.php?latex=x&space;=&space;\begin{bmatrix}&space;q&space;\\&space;\dot{q}\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x&space;=&space;\begin{bmatrix}&space;q&space;\\&space;\dot{q}\end{bmatrix}" title="x = \begin{bmatrix} q \\ \dot{q}\end{bmatrix}" /></a>


## Input-output linearization
According to [2]

$h$ is defined as the difference between the actual outputs (a linear mapping of the state, identity in this instance) and the desired outputs, obtained from the offline trajectory optimization using HZD.

<a href="https://www.codecogs.com/eqnedit.php?latex=h&space;=&space;H_0q&space;-&space;h_d" target="_blank"><img src="https://latex.codecogs.com/gif.latex?h&space;=&space;H_0q&space;-&space;h_d" title="h = H_0q - h_d" /></a>

Lie derivative of the outputs.

<a href="https://www.codecogs.com/eqnedit.php?latex=L_fh&space;=&space;(H_0&space;-&space;\frac{dh}{dt}c)\dot{q}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?L_fh&space;=&space;(H_0&space;-&space;\frac{dh}{dt}c)\dot{q}" title="L_fh = (H_0 - \frac{dh}{dt}c)\dot{q}" /></a>

Second Lie derivative with respect to the controlled PDE

<a href="https://www.codecogs.com/eqnedit.php?latex=L_gL_fh&space;=&space;(H_0&space;-&space;\frac{dh}{dt}c)D^{-1}B" target="_blank"><img src="https://latex.codecogs.com/gif.latex?L_gL_fh&space;=&space;(H_0&space;-&space;\frac{dh}{dt}c)D^{-1}B" title="L_gL_fh = (H_0 - \frac{dh}{dt}c)D^{-1}B" /></a>

Second Lie derivative with respect to the autonomous PDE

<a href="https://www.codecogs.com/eqnedit.php?latex=L_fL_fh&space;=&space;-\frac{d^2h}{dt^2}(c&space;q)^2&space;&plus;&space;(H_0&space;-&space;\frac{dh}{dt}c)D^{-1}F_{vec}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?L_fL_fh&space;=&space;-\frac{d^2h}{dt^2}(c&space;q)^2&space;&plus;&space;(H_0&space;-&space;\frac{dh}{dt}c)D^{-1}F_{vec}" title="L_fL_fh = -\frac{d^2h}{dt^2}(c q)^2 + (H_0 - \frac{dh}{dt}c)D^{-1}F_{vec}" /></a>

Auxilliary output

<a href="https://www.codecogs.com/eqnedit.php?latex=v&space;=&space;-(1/\epsilon)K_D&space;L_fh&space;-&space;(1/\epsilon^2)K_Ph" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v&space;=&space;-(1/\epsilon)K_D&space;L_fh&space;-&space;(1/\epsilon^2)K_Ph" title="v = -(1/\epsilon)K_D L_fh - (1/\epsilon^2)K_Ph" /></a>

Mapping to the actual output

<a href="https://www.codecogs.com/eqnedit.php?latex=u&space;=&space;L_gL_fh^{-1}(v&space;-&space;L_fL_fh)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?u&space;=&space;L_gL_fh^{-1}(v&space;-&space;L_fL_fh)" title="u = L_gL_fh^{-1}(v - L_fL_fh)" /></a>




## RES-CLFs
According to [3]. Defined as a Quadratic Program (QP). Requires the CVX MATLAB optimization package for solving! However, creating something with MATLAB's 'quadprog' should be straightforward enough, given the following equations. 'icare' is the solution of the algebraic continuous time Ricatti equations.

<a href="https://www.codecogs.com/eqnedit.php?latex=\eta&space;=&space;\begin{bmatrix}&space;h&space;\\&space;L_fh&space;\end{bmatrix}\\&space;F&space;=&space;\begin{bmatrix}&space;0&space;&&space;I&space;\\&space;0&space;&&space;0\end{bmatrix}\\&space;G&space;=&space;\begin{bmatrix}&space;0&space;\\&space;I&space;\end{bmatrix}\\&space;I_{\epsilon}&space;=&space;diag(\frac{1}{\epsilon}[M\times1],1[M\times1])\\&space;Q&space;=&space;I[2M]\\&space;P&space;=&space;icare(F,G,Q)\\&space;P_{\epsilon}&space;=&space;I_{\epsilon}PI_{\epsilon}\\&space;V&space;=&space;\eta^TP_{\epsilon}\eta\\&space;L_{FV}&space;=&space;\eta^T&space;(F^T&space;P_{\epsilon}&space;&plus;&space;P_{\epsilon}F)\eta&space;\\&space;L_{GV}&space;=&space;2\eta^TP_{\epsilon}G\\\\&space;minimize&space;(L_gL_fhu&space;&plus;&space;L_fL_fh)^T(L_gL_fhu&space;&plus;&space;L_fL_fh)&space;&plus;&space;\delta^2\\&space;s.t.&space;(L_{FV}&space;&plus;&space;L{GV}(L_gL_fhu&space;&plus;&space;L_fL_fh))&space;\leq&space;-\frac{1}{\epsilon}&space;\frac{min(eig(Q))}{max(eig(P))}V&space;&plus;&space;\delta" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\eta&space;=&space;\begin{bmatrix}&space;h&space;\\&space;L_fh&space;\end{bmatrix}\\&space;F&space;=&space;\begin{bmatrix}&space;0&space;&&space;I&space;\\&space;0&space;&&space;0\end{bmatrix}\\&space;G&space;=&space;\begin{bmatrix}&space;0&space;\\&space;I&space;\end{bmatrix}\\&space;I_{\epsilon}&space;=&space;diag(\frac{1}{\epsilon}[M\times1],1[M\times1])\\&space;Q&space;=&space;I[2M]\\&space;P&space;=&space;icare(F,G,Q)\\&space;P_{\epsilon}&space;=&space;I_{\epsilon}PI_{\epsilon}\\&space;V&space;=&space;\eta^TP_{\epsilon}\eta\\&space;L_{FV}&space;=&space;\eta^T&space;(F^T&space;P_{\epsilon}&space;&plus;&space;P_{\epsilon}F)\eta&space;\\&space;L_{GV}&space;=&space;2\eta^TP_{\epsilon}G\\\\&space;minimize&space;(L_gL_fhu&space;&plus;&space;L_fL_fh)^T(L_gL_fhu&space;&plus;&space;L_fL_fh)&space;&plus;&space;\delta^2\\&space;s.t.&space;(L_{FV}&space;&plus;&space;L{GV}(L_gL_fhu&space;&plus;&space;L_fL_fh))&space;\leq&space;-\frac{1}{\epsilon}&space;\frac{min(eig(Q))}{max(eig(P))}V&space;&plus;&space;\delta" title="\eta = \begin{bmatrix} h \\ L_fh \end{bmatrix}\\ F = \begin{bmatrix} 0 & I \\ 0 & 0\end{bmatrix}\\ G = \begin{bmatrix} 0 \\ I \end{bmatrix}\\ I_{\epsilon} = diag(\frac{1}{\epsilon}[M\times1],1[M\times1])\\ Q = I[2M]\\ P = icare(F,G,Q)\\ P_{\epsilon} = I_{\epsilon}PI_{\epsilon}\\ V = \eta^TP_{\epsilon}\eta\\ L_{FV} = \eta^T (F^T P_{\epsilon} + P_{\epsilon}F)\eta \\ L_{GV} = 2\eta^TP_{\epsilon}G\\\\ minimize (L_gL_fhu + L_fL_fh)^T(L_gL_fhu + L_fL_fh) + \delta^2\\ s.t. (L_{FV} + L{GV}(L_gL_fhu + L_fL_fh)) \leq -\frac{1}{\epsilon} \frac{min(eig(Q))}{max(eig(P))}V + \delta" /></a>


## TSC
According to [4]. Defined as a Quadratic Program (QP). Requires the CVX MATLAB optimization package for solving! However, creating something with MATLAB's 'quadprog' should be straightforward enough, given the following equations:

<a href="https://www.codecogs.com/eqnedit.php?latex=Y&space;=&space;h_{actual}&space;-&space;h_d&space;\\&space;\dot{Y}&space;=&space;J_y\dot{q}&space;-&space;\frac{dh}{dt}&space;\\&space;\ddot{Y}&space;=&space;J_y(A_qu&space;&plus;&space;b_q)&space;&plus;&space;\dot{J}_y\dot{q}&space;-&space;\frac{d^2h}{dt^2}&space;\\&space;\ddot{Y}^*&space;=&space;-K_P&space;Y&space;-&space;K_D&space;\dot{Y}&space;\\\\&space;minimize&space;(\ddot{Y}&space;-&space;\ddot{Y}^*)^T(\ddot{Y}&space;-&space;\ddot{Y}^*)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Y&space;=&space;h_{actual}&space;-&space;h_d&space;\\&space;\dot{Y}&space;=&space;J_y\dot{q}&space;-&space;\frac{dh}{dt}&space;\\&space;\ddot{Y}&space;=&space;J_y(A_qu&space;&plus;&space;b_q)&space;&plus;&space;\dot{J}_y\dot{q}&space;-&space;\frac{d^2h}{dt^2}&space;\\&space;\ddot{Y}^*&space;=&space;-K_P&space;Y&space;-&space;K_D&space;\dot{Y}&space;\\\\&space;minimize&space;(\ddot{Y}&space;-&space;\ddot{Y}^*)^T(\ddot{Y}&space;-&space;\ddot{Y}^*)" title="Y = h_{actual} - h_d \\ \dot{Y} = J_y\dot{q} - \frac{dh}{dt} \\ \ddot{Y} = J_y(A_qu + b_q) + \dot{J}_y\dot{q} - \frac{d^2h}{dt^2} \\ \ddot{Y}^* = -K_P Y - K_D \dot{Y} \\\\ minimize (\ddot{Y} - \ddot{Y}^*)^T(\ddot{Y} - \ddot{Y}^*)" /></a>

## Sources
1:
@book{westervelt2018feedback,
  title={Feedback control of dynamic bipedal robot locomotion},
  author={Westervelt, Eric R and Grizzle, Jessy W and Chevallereau, Christine and Choi, Jun Ho and Morris, Benjamin},
  year={2018},
  publisher={CRC press}
}

2: 
@article{westervelt2003hybrid,
  title={Hybrid zero dynamics of planar biped walkers},
  author={Westervelt, Eric R and Grizzle, Jessy W and Koditschek, Daniel E},
  journal={IEEE transactions on automatic control},
  volume={48},
  number={1},
  pages={42--56},
  year={2003},
  publisher={IEEE}
}

3:
@article{ames2014rapidly,
  title={Rapidly exponentially stabilizing control lyapunov functions and hybrid zero dynamics},
  author={Ames, Aaron D and Galloway, Kevin and Sreenath, Koushil and Grizzle, Jessy W},
  journal={IEEE Transactions on Automatic Control},
  volume={59},
  number={4},
  pages={876--891},
  year={2014},
  publisher={IEEE}
}

4:
@inproceedings{wensing2013generation,
  title={Generation of dynamic humanoid behaviors through task-space control with conic optimization},
  author={Wensing, Patrick M and Orin, David E},
  booktitle={2013 IEEE International Conference on Robotics and Automation},
  pages={3103--3109},
  year={2013},
  organization={IEEE}
}






