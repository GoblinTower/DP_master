# Master thesis

This Master thesis concerns the topic of Dynamic Positioning (DP). All the files stored here were used in generating the results given in the thesis.
All code is written in MATLAB. The final report is available on the usn.no webpage [LINK WILL BE UPDATED LATER]. It can also be found [here](https://github.com/GoblinTower/DP_master/blob/main/Thesis/Master_thesis_v11.pdf).
The thesis itself was written in LaTeX.

A presentation is given [here](https://github.com/GoblinTower/DP_master/tree/main/Presentation). It is written in PowerPoint.

The code is dependent on the MSS (Marine System Simulator) library by Professor Dr. Thor Inge Fossen.
The system identification uses the DSR and DSR_e subspace system identification libraries developed by Associate Professor Dr. David Di Ruscio.

The thesis explores:
- DP (Dynamic Positioning) using LQ optimal control
- DP (Dynamic Positioning) using Model Predictive Control (MPC)
- DP (Dynamic Positioning) using Nonlinear Model Predictive Control (NMPC)
- DP using LQ optomal control based on identified model using DSR, DSR_e and PEM
- Path follwoing and trajectory tracking
- Green DP (environmentally friendly DP mode)
- Thruster allocation

## File structure

**Master/Tools/:** Contains different functions used frequently.  
**Master/Tools/Tests/:** Test files for some of the functions.  

**Libraries/DSR/:** Contains the DSR and DSR_e functions used for system identification.  

**Presentation/:** Presentation used when presenting project for the examineers.

**Thesis/:** Contains Master thesis.

**Master/DP model/Linearization/:** Contains files for identifying eigenvalues for the supply model and a simple attempt of linearization. However, the information stored in the rotation matrix is important and need to be preserved.  
**Master/DP model/LQ/:** Simulations using Linear Quadratic (LQ) optimal control.  
**Master/DP model/MPC/:** Simulations using Model Predictive Control (MPC).  
**Master/DP model/NMPC/:** Simulations using Nonlinear Model Predictive Control (NMPC).  
**Master/DP model/Simulation/:** Early attempt of model simulation (should not be used, depreciated).  
**Master/DP model/System Identification/:** Contains files for running system identification and simulation files that uses model based controllers (LQ optimal control) to control the original vessel models.  
**Master/DP model/Tracking/:** Contains simulations for path-following and trajectory tracking.  
**Master/DP model/GreenDP/:** Contains simulations for GreenDP.  
**Master/DP model/Thruster allocation/:** Contains simulations for thruster allocation.  

Generally all simulations files read the setup from different scenario files. Thus several different scenarios can be run from the same simulation file. Furthermore, the workspace data are stored at the end of the simulations. These files are later used to generate plots of interest.
Example of the simulation-sceanrio file relation (one - many):

<pre>
LQ Optimal Control Simulation file ---> LQ optimal control no disturbance  
                                   ---> LQ optimal control with disturbance  
                                   ---> LQ optimal control without wind forces  
                                   ---> LQ optimal control with disturbance and sensor noise
                                   etc.  
</pre>

While plotting files are automatically generated and stored after each simulation, it is recommended to use the workspace files to generate plots as it allows for more fine tuning after finishing the simulations. In some of the more compuationally intensive simulations (especially when running
Monte Carlo) this is very important. On my computer the Monte Carlo simulation related to GreenDP took over 48 hours to finish; With workspace files any plot can be made after the fact without having to spend an extra 48 hours.

## ERRATA (Master thesis)
Errors are listed chronologically:  
1. Figure 3.1 shouid contain the world "LQ optimal control" in the upper left blue box instead of "MPC/NMPC".
2. Figure 3.72 should does not contain correct depiction of thruster forces along the lefternmost column. See presentation power point for correct depiction.
3. Figure 3.74 should does not contain correct depiction of thruster forces along the lefternmost column. See presentation power point for correct depiction. This was due to error in the plotting file (fixed).
4. Reference 26 contains the wrong link (correct should be https://home.usn.no/~roshans/mpc/).

Please message if any error is found in the code or in the thesis. If you have any question contact me on GitHub.






