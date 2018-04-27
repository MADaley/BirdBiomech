function [finalStates,statesOverTime,exitCode] = RunOneStride(initialStates,dynamicModel,timeParams)
% [finalStates,statesOverTime,exitCode] =
%RunOneStride(initialStates,dynamicModel,timeParams)
%
% Author:  Monica Daley 2007
%
% Set maximum number of times phase 1 is executed to 1 (to run single stride)
% Output the final states for limit cycle checking.
% If using for stability analysis, the input initial states must have
% already been determined to be a limit cycle

timeParams.maxNumSpPhase(1) = 1;
timeParams.maxNumSpPhase(2) = 1;

[statesOverTime, phasesOverTime, timeVector, transitions,currentPhaseIndex, exitCode]  = RunHybridDynamics(initialStates, dynamicModel, timeParams); %#ok<ASGLU>
% [statesOverTime, phasesOverTime, timeVector, transitions,
% currentPhaseIndex,exitCode,solverSteps] = RunHybridDynamics(initialStates, dynamicModel, timeParams,odeSolverH,odeOptPairs)

if ~isempty(transitions)
    finalStates = transitions{end}{3};
else
    finalStates = statesOverTime(:,end);
end
finalStates(1) = 0;

end