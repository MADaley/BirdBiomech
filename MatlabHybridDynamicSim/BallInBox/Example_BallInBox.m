%Run Ball in Box example
% Set parameters
params.g = 9.81;    %gravity
params.m = 1; %the ball weighs 1 kg
params.r = 1; % radius of 1m
params.cor = 0.8; %coefficient of restitution
params.wallXmin = 0;
params.wallXmax = 10;
params.wallYmin = 0;
params.wallYmax = 10;
% params.timeStep = .0001; %time increment produced for plotting output 
params.movieTimeStep = 0.025; %time increment used for plotting the movie, >= timeStep

%Build a specific instance of the model with these parameters
%(output is an array of functions to put into  RunHybridDynamics)
%See the Build function for details on the dynamic equations
dynamicModel = BuildBallInBox(params);

%Time parameters for this model
timeParams.timeLimit = 12;%time limit for simulation
timeParams.timeStep = .001; %time increment produced for plotting output 
timeParams.maxNumPhases = 70;%Set number of completed phases in simulation
% timeParams.maxCPUTime = 0.001;

%Initial Conditions 
initialStates(1) = 5; %x position
initialStates(2) = 5; %y position
initialStates(3) = -6.5; %x velocity
initialStates(4) = 10; %y velocity
initialStates=initialStates';

[statesOverTime, phasesOverTime, timeVector, transitions, finalPhaseIndex, exitCode]  = RunHybridDynamics(initialStates, dynamicModel, timeParams);

 %Animate the ball in box
 [centeredMovie,handle] = MakeBallInBoxMovie(statesOverTime(1:4,:), params, timeParams);
