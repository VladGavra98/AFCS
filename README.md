# Automatic Flight Control System Design (AFCS):

This repository contains the Matlab & Simulink files used for designing an automatic flight controller for the F-16 fighter jet. 



The controller contains the following features:
 - Load-factor feedback using an accelerometer.  
 - Longituinal and lateral stability augmentation (SAS).
 - Longitudinal control augmnetation (CAS) accoring to Gibson criteria and MIL-specs.
 - Glideslope follower.
 - Flare & touch-down  optimal controller.
 
 
 The group project follows the guidlines of AE4301 course, as part of the Aerospace Engineering Master.
 
 ![image](https://user-images.githubusercontent.com/43482835/105977034-95895400-6099-11eb-9443-a532f0efac3d.png)


# Files (corresponding to the chapters in the assignment manual)
## Chapter 4
The script used for plotting Figure 2 and Figure 3 of the report may be found in the FindF16dynamics.m file.

## Chapter 5 
The script used for the longitundinal open loop analysis may be found in the Task_5.m file, while the one created for lateral open loop analysis may be found in the Task_5_lat.m file.

## Chapter 6
The code developed for the pitch rate command design tasks may be found in the CAS_task6.m file.

## Chapter 7
The files used for chapter 7, where the glideslope and flare controller are designed can be found in the F16 Simulation/GlideSlopeFlare folder. The simulink model is in the 'glideslope.slx' file. 

The state-space matrix coefficient A, B, C, D need to be generated before running the simulation, by running 'getMatrixValues.m'. Here, the longitudinal model is loaded from the 
'f16long5000ft300fts.mat', which contains the longitudinal F16 model linearised and trimmed at an altitude of 5000 ft and an airspeed of 300 ft/s.

'pitchTuner.m' and 'speedTuner.m' are helper files which were used to attempt to tune the controllers, but they are not needed to run the simulation.

Finally, 'trimValues.txt' contains the trim values obtained, but this file is not used for the simulation.
