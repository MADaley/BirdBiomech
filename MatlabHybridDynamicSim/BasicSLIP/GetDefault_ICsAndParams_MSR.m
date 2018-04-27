function [newModel] = GetDefault_ICsAndParams_MSR(model,exceptParams,exceptStates)
% Sets default model parameter and simulation parameter
% values for the Mass Spring Runner Simulation.
% params are stored in substructure in model structure:
% model.params
% Does not replace params named in 'exceptParams'
% Sets default initialStateGuess (Initial condition/ICs)
% Does not replace states named in 'exceptStates'

if ~exist('exceptParams','var')
    exceptParams = [];
end
if ~exist('exceptStates','var')
    exceptStates = [];
end
if ~exist('model','var') || isempty(model)
    model = [];
end
newModel = model;

if ~isfield(model,'buildFunctionHandle') || isempty(model.buildFunctionHandle)
    % Identify model build function to generate the
    % array of functions for the HybridDynamicEngine
    % See BuildMassSpringRunner example
    buildFunctionHandle = @BuildMassSpringRunner;
else
    buildFunctionHandle = model.buildFunctionHandle;
end

if ~isfield(model,'timeParams') || isempty(model.timeParams)
    timeStep = 0.00001; % time increment produced for plotting output (needs to be in both timeParams and params
    timeParams.timeLimit = 2;% time limit for simulation
    timeParams.timeStep = timeStep; % time increment produced for plotting output
    timeParams.maxNumSpPhase(1) = 1;% set phase to count for step count
    timeParams.maxNumSpPhase(2) = 1;% set number of times to execute this phase
else
    timeParams = model.timeParams;
end

if ~isfield(model,'defParams') || isempty(model.defParams)
    % detault model parameters
    defParams.apexLegTheta = 2.4444; %   2.2332 angle of the leg relative to right ground horizontal
    defParams.g = 9.81;    % gravity
    defParams.m = 80;      % body mass in kg
    defParams.Lnot = 1; % resting length of the spring
    defParams.k = 5000; % spring stiffness try, for example 5000, 10000, 20000, 30000, 40000, 50000
    % Assume the center of mass of the leg is about 2/3rds Lnot:
    % Calculate leg retraction velocity based on pendulum period
    defParams.legVel = -2.*sqrt(defParams.g./(defParams.Lnot.*0.667));
    % dY position at apex, can be used to simulate a drop step,
    % or searching for different limit cycles
    defParams.vertStepHeight = 0;
    %Set simulation time parameters
    defParams.timeStep = timeStep; %This variable needs to be in both structures
    defParams.movieTimeStep = 0.005; % time increment used for plotting the movie
else
    defParams = model.defParams;
end

paramNames = fieldnames(defParams);
if isfield(newModel,'params')
    newParams = newModel.params;
    for i = 1:length(paramNames)
        curParam = paramNames{i};
        skipParam = any((strcmp(curParam,exceptParams)));
        if ~skipParam || ~isfield(newParams,curParam)
            newParams.(curParam) = defParams.(curParam);
        end
    end
else
    newParams = defParams;
end

if ~isfield(model,'stateNames') || isempty(model.stateNames)
    stateNames{1} = 'body CoM x';
    stateNames{2} = 'body CoM y';
    stateNames{3} = 'body_x_dot';
    stateNames{4} = 'body_y_dot';
    stateNames{5} = 'foot x';
    stateNames{6} = 'foot y';
else
    stateNames = model.stateNames;
end

if ~isfield(model,'defInitialStates') || isempty(model.defInitialStates)
    % Set default initial condition values for the Mass Spring Runner Simulation.
    % initialStates are always in a vector, so stateNames is output separately
    % A cell array of state names to be used in GUIs or other user interface
    % Initial Conditions
    defInitialStates(1) = 0;% initial horizontal position
    defInitialStates(2) =   0.9445; % bounce height  in metres 0.91862
    defInitialStates(3) = 5; % forward velocity at apex in metres per second    10.039
    defInitialStates(4) = 0; % vertical velocity at apex
    defInitialStates(5) = defInitialStates(1)-(defParams.Lnot.*cos(defParams.apexLegTheta));
    defInitialStates(6) = defInitialStates(2)-(defParams.Lnot.*sin(defParams.apexLegTheta));
    defInitialStates=defInitialStates';
else
    defInitialStates = model.defInitialStates;
end

if isfield(newModel,'initialStateGuess')
    newInitialStates = newModel.initialStateGuess;
    for i = 1:length(defInitialStates)
        resetTrue = isempty(union(i,exceptStates));
        if resetTrue
            newInitialStates(i) = defInitialStates(i);
        end
    end
else
    newInitialStates = defInitialStates;
end

[newParams,newInitialStates] = FixCoupledValues_SLIP(newParams,newInitialStates);

newModel.params = newParams;
newModel.initialStateGuess = newInitialStates;
newModel.stateNames = stateNames;
newModel.timeParams = timeParams;
newModel.defParams = defParams;
newModel.defInitialStates = defInitialStates;
newModel.buildFunctionHandle = buildFunctionHandle;
newModel.setSwingPeriod = 0.315;
newModel.setMeanVel = 10;

% Initial Conditions and params that result in similar gait timing
% (stance, swing duration) as real guinea fowl running fast
%       (around 4.3 m/s mean V)
% Actual guinea fowl values for near maximum speed running:
% speed 4.2 m/s, stance period 0.0825s, swing period 0.1425s, DF: 0.367
% Stance half period freq: 6Hz, Swing half period frequency: 3.5 Hz
% (about 3.3-3.9 Fhat)
% initialStates(1) = 0;% initial horizontal position
% initialStates(2) =  0.22507; % bounce height
% initialStates(3) = 4.4; % forward velocity at apex
% initialStates(4) = 0; % vertical velocity at apex
% initialStates(5) = initialStates(1)-(params.Lnot.*cos(params.apexLegTheta));
% initialStates(6) = initialStates(2)-(params.Lnot.*sin(params.apexLegTheta));
% initialStates=initialStates';
% kleg = 500, Lnot = 0.3, m = 1.8, legVel = -0.87, apexLegTheta = 2.3081
