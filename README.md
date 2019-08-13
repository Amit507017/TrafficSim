# TrafficSim

A matlab-based simulation tool for defining and simulating traffic-scenarios consisting of dynamic road traffic-participants 
such as vehicles, pedestrians, bicyclist, etc. with any types of road infrastructure (straight/curved roads, intersections, roundabout,etc.)
It is completely modular with the possibility of defining own dynamic models and controllers. A two-track vehicle dynamic model is 
already included. No Matlab toolbox is necessary and basic Matlab is sufficient for running the code. For Matlab 2018 onwards, please 
use the files in the folder "TrafficSim2018".

# Basic instructions to get started

1) Run gui.m
2) Click on radio button ‘Road’.
3) Define the road parameters and click on ‘New Road’ button.
4) Define reference points (at least 4) on the axis of the left side and press ‘Enter’.
5) A road will be drawn through the reference points with defined parameters given it does not violate the geometric constraints (radius of curvature is too large for the width of the road).
6) Click on the radio button ‘Vehicle’ and ‘Vehicle Parameters’ section will be displayed.
7) Click on button ‘New’ and click on the axis to place the vehicles. Press ‘Enter’ once you finish placing vehicles.
8) Right click on the vehicle and change one of the vehicles type to EGO.
9) To change the properties of the vehicle, click on ‘Edit’ button on the top followed by clicking the vehicle whose properties has to be changed.
10) The vehicles properties will be shown on right hand side.
11) Change the properties and click on ‘Save’ button in the right bottom corner.
12) Click on the ‘Define Scenario’ button.
13) A panel with list of objects will be displayed on the right hand side.
14) Select the object (will turn to yellow) and define its reference path by selecting the tracks of the vehicle or manually.
15) Click on the ‘Select the Track’ followed by clicking on the dotted reference track from the axis. Click ‘Enter’ after finishing the selection.
16) After defining reference paths for all objects, click on ‘Simulate’ button. 
17) Scenarios will be simulated with defined parameters. In each scenario initial position or initial velocity of one of the object will be changed.
18) .mat files are generated for each scenario which will have all parameters information.

# Parameter Description in .mat file
All the parameters are described in "Parameter_TrafficSim.xlsx"

# Citation
Following papers used this tool for generating many thousand scenarios. Please cite one of this paper if you use this tool in your projects.

• A. Chaulwar, M. Botsch, T. Krueger, and T. Miehling, “Planning of Safe Trajectories in Dynamic Multi-Object Traffic-Scenarios”, Journal of Traffic and Logistics Engineering, 2016. 
• A. Chaulwar, M. Botsch, W. Utschick, “A Hybrid Machine Learning Approach for Planning Safe Trajectories in Complex Traffic-Scenarios, IEEE Int. Conference on Machine Learning and Applications, 2016.
• A. Chaulwar, M. Botsch, W. Utschick, “A Machine Learning Based Biased- Sampling Approach for Planning Safe Trajectories in Complex Traffic-Scenarios, IEEE Intelligent Vehicle Symposium, 2017. 
• A. Chaulwar, M. Botsch, W. Utschick, “Generation of Reference Trajectories for Vehicle Motion Planning, International Conference of Artificial Neural Networks, 2018. 
• A. Chaulwar, H. Al-Hashimi, M. Botsch, W. Utschick, “Efficient Hybrid Machine Learning Algorithm for Trajectory Planning in Critical Traffic-Scenarios, International Conference of Intelligent Transportation Engineering, 2019.

# Acknowledgment
This tool was developed in CARISSMA, a vehicle safety research institute at TH Ingolstadt, under the supervision of Prof. Michael Botsch. His other research assistants, Lakshman Balsubramnian and Alberto Flores Fernandez, have extended the tool to make it compatible for Matlab 2018 version.
