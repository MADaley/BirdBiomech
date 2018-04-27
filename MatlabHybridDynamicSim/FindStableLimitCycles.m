function [stableLCArray, stableEigValArray, exitResultArray, fullEigValArray] = ...
    FindStableLimitCycles(modelFunHandle, goodLCArray, paramsArray, timeParams, stableEigThresh, stabilityCheckStates, dirName)

if ~exist('stableEigThresh','var')
    stableEigThresh = 1.0';
end

if ~exist('dirName','var')
    dirName = matlabroot;
end

exitResultArray = cell(size(paramsArray));
fullEigValArray = cell(size(paramsArray));
stableLCArray = cell(size(paramsArray));
stableEigValArray = cell(size(paramsArray));

% [exitResultArray{:}] = deal(ones(1,size(goodLCArray,2)).*NaN);
% [fullEigValArray{:}] = deal(ones(size(goodLCArray)).*NaN);

saveStruct = struct('exitResultArray',exitResultArray,'fullEigValArray',...
                    fullEigValArray,'stableLCArray',stableLCArray, 'stableEigValArray', stableEigValArray);   
                
filename = [dirName 'FindStableLimitCycles_Temp.mat'];
timedSave = timer('TimerFcn',{@my_callback_fcn, filename, saveStruct}, 'Period', 120.0,'BusyMode','queue','ExecutionMode','fixedRate');

start(timedSave);
for j = 1:size(paramsArray,2)
    params = paramsArray{j};
    dynamicModel =  modelFunHandle(params);

    for i = 1:size(goodLCArray{j},2)
            A = partialderiv(goodLCArray{j}(:,i), dynamicModel, timeParams,stabilityCheckStates);
            [v,d] = eig(A);
            fullEigValArray{j}(:,i)=abs(diag(d));
            
            if max(fullEigValArray{j}(:,i)) <= stableEigThresh
                exitResultArray{j}(:,i) = 1; %Locally Stable
            else
                exitResultArray{j}(:,i) = -1; %Not locally stable
            end           
    end
        stableLCArray{j} = goodLCArray{j}(:,exitResultArray{j}==1);
        stableEigValArray{j} = fullEigValArray{j}(:,exitResultArray{j}==1);
        
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
