function [summaryData,exitCode] = SLIP_RunOneStrideSummary(model,stateFieldName,animateMode)
% Runs through one stride of the SLIP model generates summary
% data about the resulting gait cycle.
% This should be run after a limit cycle has been found
%   This function does not check for a limit cycle or 
%        try to find one.  
%Inputs:
% model: structure containing the following elements:
%   .limitCycleStates: vector OneStride simulation intial states values
%   .buildFunctionHandle: function handle string
%       identifiying the associated BuildModelFunction, 
%       formated for HybridDynamicEngine
%   .timeParams: simulation time parameters formated for HybridDynamicEngine
%   .params:  model parameters
% stateFieldName: string indicating which field in model to use for the
% initialStates. Typically either 'initialStateGuess' or 'limitCycleStates;
% If animateMode = 1, a move is generated
% If animateMode = 0, no moving is generated
% This function is specific to a SLIP model because of the specific
% subfunctions associated with generating the movie and ground forces
% 

initialStateGuess = model.(stateFieldName);
buildFunctionHandle = model.buildFunctionHandle;
timeParams = model.timeParams;
params = model.params;

if ~exist('animateMode','var') || isempty(animateMode)
    animateMode = 0;
end

% Set maximum number of times phase 1
% is executed to 1 (to run single stride)
timeParams.maxNumSpPhase(1) = 1;
timeParams.maxNumSpPhase(2) = 1;

% Build a specific instance of the model with these parameters
% (output is an array of functions to put into RunHybridDynamics)
% See the Build function for details on the dynamic equations
dynamicModel =  feval(buildFunctionHandle,params);

% Run simulation with the specified model, initial state...
%   and simulation time parameters
[statesOverTime, phasesOverTime,...
    timeVector, transitions, finalPhaseIndex, exitCode]...
    = RunHybridDynamics(initialStateGuess, dynamicModel, timeParams);

if ~isempty(transitions)
    finalStates = transitions{end}{3};
else
    finalStates = statesOverTime(:,end);
end
finalStates(1) = 0;


% Get forces etc.
% (in the simple spring system can be back
% calculated from body and footpositions)
legInd = [5:6];
[summaryData,legforces] = CalculateForces_Gen(statesOverTime,phasesOverTime,params,legInd);
summaryData.finalStates = finalStates;
summaryData.statesOverTime = statesOverTime;
summaryData.timeVector = timeVector;
summaryData.legforces = legforces;

% If the model changes, a different function will have to be substituted
% for the one above
%     isStance = (phasesOverTime==2)';

if animateMode == 1
    % Animate the mass spring runner: a model specific graphing function
    labelGraph =0;
    subPlotData = legforces;
    bodyStates = statesOverTime(1:4,:);
    footStates = statesOverTime(legInd,:);
    [summaryData.progFigHandle] = MakeProgressGraph_wForces(bodyStates, footStates, timeVector,params, labelGraph,subPlotData);
    tilefigs;
else
 summaryData.progFigHandle = [];
end