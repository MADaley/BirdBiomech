function dydx = partialderiv(x0, dynamicModel, timeParams,stabilityCheckStates)
% Evaluates the partial derivative dy/dx, where y is the 
% value returned by  dynamicModel. 
% Note that the partial derivative works even for vector-
% valued functions.  If x has n elements, and y has m elements,
% then dydx is an m x n matrix.
%From Dynamic Walking 2006
%Writen for locomotion simulations: Currently assumes that the first state is 
%is a position associated with the goal motion. 
%Modified by Monica Daley to use with Hybrid Dynamic Engine

%Set maximum number of times phase 1 is executed (to count strides)
timeParams.maxNumSpPhase(1) = 1;
timeParams.maxNumSpPhase(2) = 1;

x = x0;
% Run through one stride- assume that it has already been determined to be
% a limit cycle
[y0] = RunOneStride(x0,dynamicModel,timeParams); %evaluates the dynamicModel at x
y0(1)=0;

dx = 1e-7; % a small perturbation size, which can be any small value

% store the delta y's in a matrix, each column is the
% dy for a different perturbed element of x.
dy = zeros(length(y0(stabilityCheckStates)), length(x(stabilityCheckStates))); % make a matrix of zeros
% perturb elements of x by a very small amount
for i = 1:length(stabilityCheckStates)
  x = x0;
  x(stabilityCheckStates(i)) = x(stabilityCheckStates(i)) + dx; % perturb only the i'th element  
  [y] = RunOneStride(x,dynamicModel,timeParams); %evaluates the dynamicModel at x
  y(1)=0;
  dy(:,i) = y(stabilityCheckStates) - y0(stabilityCheckStates);
end

dydx = dy / dx; % return the matrix of partial derivatives