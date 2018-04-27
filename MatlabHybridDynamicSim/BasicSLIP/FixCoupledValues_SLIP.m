function [newParams,newInitialStates] = FixCoupledValues_SLIP(inParams,inInitialStates)
%Fixes coupled parameter and state values

newParams = inParams;
newInitialStates = inInitialStates;
% Calculate foot position based on leg length and apexLegTheta
% The minimal sets of states would use legTheta, but the choice was 
% made to use foot position for each of use in later plotting functions.

newInitialStates(5) = newInitialStates(1)-(newParams.Lnot.*cos(newParams.apexLegTheta));
newInitialStates(6) = newInitialStates(2)-(newParams.Lnot.*sin(newParams.apexLegTheta));
