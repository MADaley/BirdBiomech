function [exitResult, newModel, msg] = FindOneLimitCycle_GaitOptimized(model, searchOptions)
% [exitResult, newModel, msg] = FindOneLimitCycle(model, searchOptions)
%
% Author:  Monica Daley 2007
%
% Inputs:
% model: structure containing the following elements:
%   .initialStateGuess: vector containing search starting values
%   .buildFunctionHandle: function handle string
%       identifiying the associated BuildModelFunction,
%       formated for HybridDynamicEngine (see 'RunHybridDynamics')
%   .timeParams: simulation time parameters formated for HybridDynamicEngine
%   .params:  model parameters
%
% searchOptions: a structure with fields corresponding to the search
%   options described below.
% .searchMode:
%      default is 1
%    1: gradient-based unconstrained nonlinear optimisation (fminun)
%    2: gradient-free unconstrained nonlinear optimisation (fminsearch)
%    3: simple:  runs model until it either falls, reaches a limit cycle or
%     maximum number of steps.
%    If mode 3 is used, a variable called maxNumStrides should also be
%   included (default 20)
%
%   .maxNumStrides: required only if searchMode = 3
%       Sets the maximun number of strides for the simulation to run before
%       exiting (to prevent endless searching if the simulation happens
%       upon a solution with a limit cycle over 2+ strides rather than one.
%
%   .searchStateVector:  states to allow to vary in the search.
%       Only required if searchMode = 1 or 2 (not 3)
%
%   .searchParamNames:  cell array of params to allow to vary in the search.
%       Only allowed if searchMode = 1 or 2 (not 3)
%
%   .limitCheckStateVector: a vector identifying the states to check 
%   for a limit cycle (for example, in the running model, forward motion 
%   is not checked, because this is the goal movement direction.
%
%   .limitCycleThreshold: a scalar determining the objective function
%       value below which the stride will be considered a limit cycle.
%       (a small number indicates a smaller difference between the
%      initial states and final states after one stride).
%
%   .stableEigThresh:  a scalar determining the value for the neutrally
%   stable eigenvalue.  Default = 1.0, but this allows a very slightly
%   higher number to be use becuase of numerical simulation inexactitude,
%   if the user wishes.
%
% Outputs:
% exitResult: (1) isLimitCycle true/false, (2) isStable true/false
% newModel: with new params and added fields
%    .limitCycleStates:  states for limit cycle, or NaNs if not found
%    .eigValArray:   Array of eigenValues if limit cycle was found.

exitResult = nan(2,1);
limitCycleStates = nan(length(model.initialStateGuess),1);
eigValArray = [];

% Parameter checking and default setting section
if ~exist('searchOptions','var') || isempty(searchOptions)
    % if no search options are included, use simple search
    % searchMode = 3
    searchOptions.searchMode = 3;
end

if ~isfield(searchOptions,'searchMode')
    searchOptions.searchMode = 3;
elseif isempty(searchOptions.searchMode)
    searchOptions.searchMode = 3;
end

% Check for searchMode specific settings:
%   First check that searchMode is set, use default if nec.
if searchOptions.searchMode == 3
    if ~isfield(searchOptions,'maxNumStrides')
        searchOptions.maxNumStrides = 20;
    elseif isempty(searchOptions.maxNumStrides)
        searchOptions.maxNumStrides = 20;
    end
    maxNumStrides = searchOptions.maxNumStrides;
elseif searchOptions.searchMode == 1 || searchOptions.searchMode == 2
    if ~isfield(searchOptions,'searchStateVector')
        error('Search parameter searchStateVector required for searchModes 1 & 2. See help in FindOneLimitCycle_Opt.m ')
    end
    nVarStates = length(searchOptions.searchStateVector);
    searchStateVector = searchOptions.searchStateVector;
    if isfield(searchOptions,'searchParamNames') && ~isempty(searchOptions.searchParamNames)
        searchParamNames = searchOptions.searchParamNames;
        nVarParams = length(searchParamNames);
        variedParamValues = nan(nVarParams,1);
        for i = 1: nVarParams
            variedParamValues(i) = model.params.(searchParamNames{i});
        end
    else
        nVarParams = 0;
        variedParamValues = [];
    end
else
    error('Invalid searchMode field in searchOptions structure. See help in FindOneLimitCycle_Opt.m')
end

searchMode = searchOptions.searchMode;

if ~isfield(searchOptions,'limitCheckStateVector')
    % Check all states by default
    searchOptions.limitCheckStateVector = [1:length(model.initialStateGuess)]';
elseif isempty(searchOptions.limitCheckStateVector)
    searchOptions.limitCheckStateVector = [1:length(model.initialStateGuess)]';
end

if ~isfield(searchOptions,'limitCycleThreshold')
    % Set to arbitrary small value
    searchOptions.limitCycleThreshold = 0.0000001;
elseif isempty(searchOptions.limitCycleThreshold)
    searchOptions.limitCycleThreshold = 0.0000001;
end

if ~isfield(searchOptions,'stableEigThresh')
    % Set to 1.0
    searchOptions.stableEigThresh = 1.0;
elseif isempty(searchOptions.stableEigThresh)
    searchOptions.stableEigThresh = 1.0;
end

limitCheckStateVector = searchOptions.limitCheckStateVector;
limitCycleThreshold = searchOptions.limitCycleThreshold;
stableEigThresh = searchOptions.stableEigThresh;

% Set up variables for search
% Set maximum number of times phase 1
% is executed to 1 (to run single stride)
model.timeParams.maxNumSpPhase(1) = 1;
model.timeParams.maxNumSpPhase(2) = 1;

isLimitCycle = false; %#ok<NASGU>
isLocallyStable = false;
hasFailed = false; %#ok<NASGU>

% Use fminun or fminseardch to search for a limit cycle
newModel = model;
newModel.limitCycleThreshold = limitCycleThreshold;

if searchMode == 1 || searchMode ==2
    variedStateValues = model.initialStateGuess(searchStateVector);
    variedValues = [variedStateValues; variedParamValues];

    funTol = (limitCycleThreshold./10); %#ok<NASGU>
    if searchMode == 1 % Use fminunc: uses gradients
        fminoptions =optimset('MaxFunEvals', 1000, 'FunValCheck', 'on','TolFun', funTol,'GradObj','on', 'LargeScale', 'off','TypicalX', variedValues);
        [newVariedValues,fval,searchExitFlag,outputFminUnc] =...
            fminunc(@(x) ObjectiveFunctionForLimitCycle(x,newModel,searchStateVector,limitCheckStateVector,searchParamNames),variedValues,fminoptions); %#ok<NASGU>
        msg = outputFminUnc.message;
    elseif searchMode == 2 % Use fminsearch: gradient free method
        fminoptions =optimset('FunValCheck', 'on','TolFun', funTol);
        [newVariedValues,fval,searchExitFlag,outputFminSearch] =...
            fminsearch(@(x) ObjectiveFunctionForLimitCycle(x,newModel,searchStateVector,limitCheckStateVector,searchParamNames),variedValues,fminoptions); %#ok<NASGU>
        msg = outputFminSearch.message;
    end

    % Set new states and parameters,
    newIniStateVals = newVariedValues(1:nVarStates);
    newParamValues = newVariedValues((nVarStates+1):(nVarStates+nVarParams));
    [newModel.params,newInitialStates] =...
        Set_ICsAndParams(newModel.params,newModel.initialStateGuess,searchParamNames,newParamValues,searchStateVector,newIniStateVals);
    dynamicModel =  feval(newModel.buildFunctionHandle, newModel.params);
    [finalStates,statesOverTime,exitCode] = RunOneStride(newInitialStates,dynamicModel, newModel.timeParams); %#ok<ASGLU>

elseif searchMode == 3
    dynamicModel =  feval(newModel.buildFunctionHandle, newModel.params);
    [simpleSearch_exitResult, simpleSearch_finalStates,fval,simpleSearch_msg,numStrides] = SimpleLimitCycleSearch(newModel.initialStateGuess, dynamicModel, newModel.timeParams, limitCheckStateVector, maxNumStrides, limitCycleThreshold); %#ok<NASGU>
    searchExitFlag = simpleSearch_exitResult(2);
    msg = simpleSearch_msg;
    disp('Simple search: number of strides:')
    disp(numStrides);
    if simpleSearch_exitResult(2) ==1
        newInitialStates = simpleSearch_finalStates;
    else
        %         disp(simpleSearch_msg);
        newInitialStates = newModel.initialStateGuess;
    end
    exitCode = simpleSearch_exitResult(1).*simpleSearch_exitResult(2);
    newModel.simpleSearchNumStrides = numStrides;
else
    error('Invalid search mode input into FindOneLimitCycle');
end

if  searchExitFlag == 1
    searchConverged = true;
    if fval >= limitCycleThreshold
        disp('Objective function minimised, but not to value lower than limitCycleThreshold: Check the results. ');
        isLimitCycle = false;
    else
        isLimitCycle = true;
    end
    limitCycleStates = newInitialStates;
    newModel.limitCycleStates = limitCycleStates;
%     disp('Limit cycle states: ');
%     disp(limitCycleStates);
    dynamicModel =  feval(newModel.buildFunctionHandle, newModel.params);
    dydx = partialderiv(limitCycleStates, dynamicModel, newModel.timeParams,limitCheckStateVector);
    % Check for local stability
    [v,d] = eig(dydx);
    eigValArray = abs(diag(d));
    newModel.eigValArray = eigValArray;
    if max(eigValArray) <= stableEigThresh
        isLocallyStable = true; %Locally Stable
        disp('Limit cycle is locally stable.');
    else
        isLocallyStable = false; %Not locally stable
        disp('Limit cycle is NOT locally stable.');
    end
else
    searchConverged = false;
    isLimitCycle = false;
    newModel.limitCycleStates = limitCycleStates;
    newModel.eigValArray = eigValArray;
%     disp('Initial state guess at termination:');
%     disp(newInitialStates);
end

if exitCode == -3
    hasFailed = false;
else
    hasFailed = true;
    disp('Model has failed: undesired terminal condition reached in simulation. Try a new initial condition guess.');
end

if  searchConverged && isLimitCycle
    exitResult(1) = 1; % limit cycle found
    exitResult(3) = 1; % searched converged
    if isLocallyStable
        exitResult(2) = 1; % is locally stable
    else
        exitResult(2) = 0; % not locally stable
    end
elseif  searchConverged && (~isLimitCycle)
    exitResult(1) = 0; % limit cycle not found
    exitResult(3) = 1; % searched converged
    if isLocallyStable
        exitResult(2) = 1; % is locally stable
    else
        exitResult(2) = 0; % not locally stable
    end
elseif hasFailed
    exitResult(1) = 0; % limit cycle not found, model failed (fell or other similation error)
    disp('Limit cycle not found: check simulation settings and initialStates guess');
else
    exitResult(1) = 0; % limit cycle not found, optimisation failed
    disp('Limit cycle not found: Check optimisation options,  exitResults, message output and initialStates guess.')
end
end

function [f,g] = ObjectiveFunctionForLimitCycle(variedValues,model,searchStateVector,limitCheckStateVector,searchParamNames)
% Set new states and parameters,
newICVals = variedValues(1:length(searchStateVector));
newParamVals = variedValues((length(searchStateVector)+1):(length(searchStateVector)+length(searchParamNames)));

[model.params,newInitialStates] =...
    Set_ICsAndParams(model.params,model.initialStateGuess,searchParamNames,newParamVals,searchStateVector,newICVals);

model.limitCycleStates = newInitialStates;

[f] = GetObjectiveFunctionValue(model,limitCheckStateVector);

if nargout > 1     % fun called with 2 output arguments
    [g] = objectiveFunctionPartialDerivative(newInitialStates,model,searchStateVector,limitCheckStateVector);
end
end

function [fVal] = GetObjectiveFunctionValue(model,limitCheckStateVector)
% Compute the function value at x
% (model.limitCycleState: the current candidate)
% [summaryData,exitCode,statesOverTime,timeVector,legforces]
[summaryData,exitCode] = SLIP_RunOneStrideSummary(model,'limitCycleStates', [0]);
statesOverTime = summaryData.statesOverTime;
initialStates = model.limitCycleStates;
finalStates = summaryData.finalStates;

% %constrain to desired gait parameters
curHalfAngle = deg2rad(summaryData.halfAngle);
 curSwingPeriod = summaryData.swingPeriod;
 %curStancePeriod = summaryData.stancePeriod;
% %  *(model.limitCycleThreshold)
 halfAngleDiffPenalty = ((((abs(model.setHalfAngle - curHalfAngle)).^2)./2));
 swingPeriodDiffPenalty = ((((abs(model.setSwingPeriod - curSwingPeriod)).^2)./2)./(summaryData.normFactors.stepPeriod));
 %stancePeriodDiffPenalty = ((((abs(model.setStancePeriod - curStancePeriod)).^2)./2)./(summaryData.normFactors.stepPeriod));
% 
meanStates = mean(statesOverTime(limitCheckStateVector,:),2);
limitStatesOverTime = statesOverTime(limitCheckStateVector,:);

for i = 1:length(limitCheckStateVector);
    normError(i) = (sum(((limitStatesOverTime(i,:)-meanStates(i)).^2),2)./abs(meanStates(i)));
    if normError(i) == 0
        normError(i) = model.limitCycleThreshold;
    end
    fi(i) = (((finalStates(limitCheckStateVector(i)) - initialStates(limitCheckStateVector(i))).^2)./normError(i));
end

% various objective functions:
%f = sum([fi]); %limit cycle only

% Limit cycle plus desired gait parametsrs
% f = sum([fi swingPeriodDiffPenalty meanVelocityDiffPenalty]);
 f = sum([fi swingPeriodDiffPenalty halfAngleDiffPenalty]);
%  f = sum([fi swingPeriodDiffPenalty stancePeriodDiffPenalty meanVelocityDiffPenalty]); %Desired gait parameters

fVal = f.^2;
if isnan(fVal)
    error('Objective function returns NaN');
end
end

function [dfdx] = objectiveFunctionPartialDerivative(x0,model,searchStateVector,limitCheckStateVector)
%
dx = 1e-3; % a small perturbation size, which can be any small value
% store the delta f's in a vector
% dy for a different perturbed element of x.
dfdx = zeros(length(x0(searchStateVector)),1); % make a vector of zeros
model.limitCycleStates = x0;
[fVal0] = GetObjectiveFunctionValue(model,limitCheckStateVector);
%[fVal] =
%GetObjectiveFunctionValue(model,limitCheckStateVector)

% perturb elements of x by a very small amount
for i = 1:length(searchStateVector)
    x = x0;
    x(searchStateVector(i)) = x(searchStateVector(i)) + dx; % perturb only the i'th element
    model.limitCycleStates = x;
    [fVal] = GetObjectiveFunctionValue(model,limitCheckStateVector);
    dfdx(i) = fVal - fVal0;
end

end
