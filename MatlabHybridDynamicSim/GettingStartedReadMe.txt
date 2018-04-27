
-Add the folder 'MatlabHybridDynamicSim' to your Matlab path (with subfolders)

This contains the utility functions for running simulations in Matlab, 
and two example packages:  'BallInBox' and 'BasicSLIP'

The first example will be 'BallInBox' which is a simple simulation involving transitions
each time the ball collides with a wall. 

To make sure your Matlab set up is ready to run the simulation: 
- Enter the command 'TestBallInBox'
This runs an example simulation in the background and checks the results against known results.  

If this is successful, you are ready to run an example simulation. 

Open 'Example_BallInBox'

This script sets up and runs a simple simulation using the Hybrid Dynamic Simulation toolbox provided. 

A structure containing simulation parameters (params) is created and passed to the function 
'BuildBallInBox', which generates a set of parameterized anonymous functions to run in the simulation. 

'dynamicModel' is the structure containing the parameterized anonymous functions

The structure 'timeParams' sets up some simulation settings:
	-time limit for running the simulation
	-time increment for plotting hte output
	-maximum number of phases to complete in the simulation

(There are other simulation options that can be set: see the 'Help' at the top of 'RunHybridDynamics')

The function 'RunHybridDynamics' runs the simulation based on the
 initialStates, dynamicModel and timeParams

The function 'MakeBallInBoxMovie' animates the results. 

Once you have successfully run an example, you can try adjusting the parameters 


'BasicSLIP' Example
------
This example creates a simulation of running using the simple spring-loaded-inverted-pendulum model
Example_MassSpringRunner is the main script to open and run the simulation.
