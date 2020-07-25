# mocoUpDown
Exe:gcolo
Files: addition.h MocoJumpGoal.cpp MocoJumpGoal.h 
	RegisterTypes_osimMocoJumpGoal.cpp RegisterTypes_osimMocoJumpGoal.h
	delp.txt readdelp.h osimMocoJumpGoalDLL.h
	
Try to model simple jumping with moco goal
Create custom moco goal for the jump predicted height
Olso can add final ground force to second cost.
At end the forward analisys is run with the control trjectory results.
The initial model is without limits and with symmetric activation

The high order integration is close to the moco result only at start
after small time the results get far

The moco goal is only for the final state, nothing integrated
the goal for the ground force is not working so good

The initial state is fixed and the initial vel is zero

Exe:fwd
Files: myForward.cpp

also myForward is run on the text results with low steps
this result is closer to the moco

Eventreporter is plotting the ground forces

The optimal forces are fixed, 300
The initial activations are from forward static by inversedynamicsolver 

The results from the fwd and moco are compared at excel
C:\Users\ostr\Google Drive (ostr@post.bgu.ac.il)\PHD\opensim\C++\jump\compare_colo_fwd.xls

