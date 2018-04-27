function [gaitSummary,limitCycleStates,model,exitResult, LC_msg, fullSummaryData] = Example_MassSpringRunner
% Example that takes a set of model parameters and initial conditions, and
% searches for a limit cycle using the search options set within this
% function.
% Uses 'FindOneLimitCycle', 'GetDefault_ICsAndParams_MSR'
% and 'SLIP_RunOneStrideSummary' with associated subfunctions.

%Uncomment line below and at bottom to use the profiler
%profile on;

% Set initial states and parameters
% Obtain default model parameters and simulation time parameters for this model
% Open and edit this function to change the initial states, parameters and
% fixed variables
[model] = GetDefault_ICsAndParams_MSR([]);

[fullSummaryData,exitCode] = SLIP_RunOneStrideSummary(model,'initialStateGuess',[0]); %#ok<NASGU,NASGU>

% Find a limit cycle starting with the initial state guess
% Set up search options for finding limit cycle
searchOptions.searchStateVector = [2]; % Allow bounce height to vary
% Select parameters to search.
searchOptions.searchParamNames = {'apexLegTheta'}; %{'apexLegTheta' 'k'};% Allow apex leg angle to vary
%Specify states to check for limit cycle (exclude horizontal position)
searchOptions.limitCheckStateVector = [2 3 4]';
searchOptions.stableEigThresh = 1.001; % Slightly different from 1.0 to allow for small numerical errors.
searchOptions.limitCycleThreshold = 0.000000001; % Arbitrary small value: larger values will be further from a perfect limit cycle
searchOptions.searchMode = [2]; % Gradient-free search: fminsearch
% In general, fminsearch has been found to be faster than fminunc

[exitResult,model,LC_msg] = FindOneLimitCycle(model, searchOptions); %#ok<NASGU>

if exitResult(1) == 1
    limitCycleStates = model.limitCycleStates;
    model.defParams = model.params;
    model.initialStateGuess = limitCycleStates;
    % Animate the results and display summary data:
    animateMode = 1; %Plot movie?  1=Yes, 0 = No 
    [fullSummaryData,exitCode] = SLIP_RunOneStrideSummary(model, 'initialStateGuess', animateMode); %#ok<NASGU>
    
    gaitSummary.peakForce = fullSummaryData.peakGroundReactionForce./fullSummaryData.normFactors.force;
    gaitSummary.peakForAftForce = fullSummaryData.peakForAftForce./fullSummaryData.normFactors.force;
    gaitSummary.forwardVelocity = fullSummaryData.forwardVelocity./fullSummaryData.normFactors.velocity;
    gaitSummary.bounceHeight = fullSummaryData.bounceHeight./fullSummaryData.normFactors.length;
    gaitSummary.dutyFactor = fullSummaryData.dutyFactor;
    gaitSummary.halfAngle = fullSummaryData.halfAngle;
    gaitSummary.stancePeriod = fullSummaryData.stancePeriod./fullSummaryData.normFactors.stepPeriod;
    gaitSummary.swingPeriod = fullSummaryData.swingPeriod./fullSummaryData.normFactors.stepPeriod;
    gaitSummary.costOfTransport = fullSummaryData.costOfTransport;

 disp('Summary of gait data, in dimensionless quantities:')
    disp(gaitSummary)    
 disp('Model parameters')
    disp(model.params);
     disp('Limit cycle states')
    disp(model.limitCycleStates);
else
    disp('Limit cycle search result message:')
    disp(LC_msg);
    disp('initialStateGuess at search exit:')
    disp(model.limitCycleStates)
    limitCycleStates = model.limitCycleStates;
    fullSummaryData = [];
    gaitSummary = [];
end

%profile viewer;

end
