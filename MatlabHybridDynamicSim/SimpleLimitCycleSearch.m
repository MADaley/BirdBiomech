function [exitResult, limitCycleStates,fVal,msg,numStrides] = SimpleLimitCycleSearch(initialStateGuess, dynamicModel, timeParams, limitCheckStates, maxNumStrides, limitCycleThreshold)
%
% Author:  Monica Daley 2007
%

if ~exist('limitCheckStates','var')
    limitCheckStates = [1:length(initialStateGuess)]';
end

if ~exist('maxNumStrides','var')
    maxNumStrides = 1; %default set to one to allow use in partialDeriv function
end

if ~exist('limitCycleThreshold','var')
    limitCycleThreshold = 0.0000001;
end

limitCycleStates = initialStateGuess;

%Set maximum number of times phase 1 is executed (to count strides)
timeParams.maxNumSpPhase(1) = 1;
timeParams.maxNumSpPhase(2) = 1;

initialStates = initialStateGuess;
numStrides = 0;
isLimitCycle = false;
hasFailed = false;

while ~ isLimitCycle && ~ hasFailed && (numStrides < maxNumStrides)
    [statesOverTime, phasesOverTime, timeVector, transitions, finalPhaseIndex, exitCode]  = RunHybridDynamics(initialStates, dynamicModel, timeParams); %#ok<ASGLU>
    numStrides = numStrides + 1;
    if ~isempty(transitions)
        finalStates = transitions{end}{3};
    else
        finalStates = statesOverTime(:,end);
    end
    meanSquared = (sum((statesOverTime(limitCheckStates,:).^2),2))./size(statesOverTime(limitCheckStates,:),2);
    fVal = (sum(((finalStates(limitCheckStates) - initialStates(limitCheckStates)).^2)./meanSquared))./size(limitCheckStates,1);
    if exitCode == -3
        if fVal <= limitCycleThreshold %#ok<BDSCI>
            isLimitCycle = true;
            msg = 'Limit cycle found';
            disp(msg);
            limitCycleStates(limitCheckStates) = finalStates(limitCheckStates);
        end
    else
        hasFailed = true;
        msg = 'Model has failed: undesired terminal condition reached in simulation';
        disp(msg);
        limitCycleStates = nan(length(initialStateGuess),1);
    end
    initialStates(limitCheckStates) = finalStates(limitCheckStates);
end
exitResult(1) = exitCode;
if isLimitCycle
    exitResult(2) = 1;
elseif hasFailed
    exitResult(2) = -1;
    msg = 'Failed to find limit cycle';
    disp(msg);
    disp('Number of strides')
    disp(numStrides);
elseif (numStrides >= maxNumStrides)
    exitResult(2) = -2;
    msg = 'Reached maximum number of strides';
    disp(msg);
else
    error('Should never reach here. Check while statement and exitResults.')
end


