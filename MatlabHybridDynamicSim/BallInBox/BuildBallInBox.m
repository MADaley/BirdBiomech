function [dynamicModel] =  BuildBallInBox(params)

% % Something that describes transitions
dynamicModel.phaseTransitionMap  = ...
    {{{1, @(x)WallTransitionFunction(x,-params.cor,1)},
       {1, @(x)WallTransitionFunction(x,-params.cor,1)},
       {1, @(x)WallTransitionFunction(x,1,-params.cor)},
       {1, @(x)WallTransitionFunction(x,1,-params.cor)}}};

% Description of phases
dynamicModel.phases = ...
    {{@(t,x)BallisticPhase(t,x,params),  @(t,x)WallHitEvent(t,x,params)}};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newGlobalState] = WallTransitionFunction(oldGlobalState,xVelMultiplier,yVelMultiplier)

newGlobalState(1:2,:) = oldGlobalState(1:2,:);
newGlobalState(3,:) = oldGlobalState(3,:).*xVelMultiplier;
newGlobalState(4,:) = oldGlobalState(4,:).*yVelMultiplier;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [statedot] = BallisticPhase(t,state,params)
%Equations of motion for ballistic motion
g = params.g; %The only force acting on the system is gravity

%equations of motion
statedot(1) = state(3);
statedot(2) = state(4);
statedot(3) = 0;
statedot(4) = -g;
statedot = statedot';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [value,isterminal,direction] = WallHitEvent(t,state,params)
%Event function to determin when the ball hits one of the four walls.

value(1) = state(1) - params.r - params.wallXmin;
value(2) = state(1) + params.r - params.wallXmax;
value(3) = state(2) - params.r - params.wallYmin;
value(4) = state(2) + params.r - params.wallYmax;

isterminal = [1 1 1 1 ];
direction = [-1 +1 -1 +1];

