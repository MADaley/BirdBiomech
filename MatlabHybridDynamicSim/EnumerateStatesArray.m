function [initialConditionArray] =  EnumerateStatesArray(initialStatesBase, statesToVary, lows, highs, steps)

IClows  =initialStatesBase;
IChighs = initialStatesBase;
ICsteps = ones(size(initialStatesBase));

IClows(statesToVary) = lows;
IChighs(statesToVary) = highs;
ICsteps(statesToVary) = steps;

initialConditionArray = enumerateArgs(IClows,IChighs,ICsteps);
initialConditionArray = initialConditionArray';
