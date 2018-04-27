function [goodLCArray, exitResultArray, limitCycleStateArray] = ...
    FindManyLimitCycles(modelFunHandle,initialStateGuessArray, paramsArray, timeParams, limitCheckStates, strideNumLimit, dirName)
%
%[goodLCArray, exitResultArray, limitCycleStateArray] = ...
%    FindManyLimitCycles(modelFunHandle,initialStateGuessArray, paramsArray, timeParams, limitCheckStates, strideNumLimit, dirName)
%
%
%Author: Monica Daley 2007
%
if ~exist('limitCheckStates','var')
    limitCheckStates = [2 3 4]';
end

if ~exist('strideNumLimit','var')
    strideNumLimit = 20;
end

if ~exist('dirName','var')
    dirName = matlabroot;
end

exitResultArray =cell(size(paramsArray));
limitCycleStateArray = cell(size(paramsArray));
goodLCArray = cell(size(paramsArray));

[exitResultArray{:}] = deal(ones(1,size(initialStateGuessArray,2)).*NaN);
[limitCycleStateArray{:}] = deal(ones(size(initialStateGuessArray)).*NaN);

saveStruct = struct('exitResultArray',exitResultArray,'limitCycleStateArray',limitCycleStateArray,'goodLCArray',goodLCArray);   
filename = [dirName 'FindLimitCycles_Temp.mat'];
timedSave = timer('TimerFcn',{@my_callback_fcn, filename, saveStruct}, 'Period', 120.0,'BusyMode','queue','ExecutionMode','fixedRate');

start(timedSave);
for j = 1:size(paramsArray,2)
    params = paramsArray{j};
    gVData.baseParams = params;
    for i = 1:gVData.gV_N
        [t_ICguess,params] = Get_ithValues_SLIP(i,gVData);
        dynamicModel =  modelFunHandle(params);
        [exitResultArray{j}(:,i), limitCycleStateArray{j}(:,i)] = FindOneLimitCycle(t_ICguess, dynamicModel, timeParams, limitCheckStates, strideNumLimit);
    end
        goodLCArray{j} = limitCycleStateArray{j}(:,exitResultArray{j}==1);
        curmsg = ['Done with params set ' num2str(j)];
        disp(curmsg)
end
stop(timedSave);
delete(timedSave);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function my_callback_fcn(timedSave, event, filename, saveStruct)
save(filename, 'saveStruct');
saveTime = datestr(event.Data.time);
msg = ['Temp file saved at: ' saveTime];
disp(msg)
