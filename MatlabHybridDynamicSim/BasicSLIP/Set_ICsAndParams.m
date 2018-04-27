function [newParams,newInitialStates] = Set_ICsAndParams(params,initialStates,pNamesArray,newParamVals,stateIDVector,newIniStateVals)
%Sets specified parameter newParamVals.
% params: input parameter structure
% initialStates: input vector of initial states
% pNamesArray: cell array of parameter names to change: Can be empty
% newParamVals: vector containing values for each parameter- in the same order as
% listed in pNamesArray
% stateIDVector: vector containing indices into initialStates: Can be empty
% values: vector containing new values for each state-
%       in the same order as listed in stateIDVector

newParams = params;
newInitialStates = initialStates;

if exist('pNamesArray','var') && (~isempty(pNamesArray))
    nToChange = length(pNamesArray);
    if nToChange ~= length(newParamVals)
        error('Error: the number of parameters named in pNamesArray does not equal the number of values provided')
    end

    for i = 1:nToChange
        currentParam = pNamesArray{i};
        if isfield(newParams,currentParam)
            newParams.(pNamesArray{i}) = newParamVals(i);
        else
            errString = ['Error: Parameter name ' num2str(i)  ' (' currentParam ') does not exist in the param structure.' ];
            error(errString);
        end
    end
end

if exist('stateIDVector','var') &&  (~isempty(stateIDVector))
    nToChange = length(stateIDVector);
    if nToChange ~= length(newIniStateVals)
        error('Error: the number of states named in stateIDVector does not equal the number of values provided')
    end

    for i = 1:nToChange
        currentState = stateIDVector(i);
        newInitialStates(currentState) = newIniStateVals(i);
    end
end

 [newParams, newInitialStates,] = FixCoupledValues_SLIP(newParams,newInitialStates);
