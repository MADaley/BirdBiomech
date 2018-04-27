function [dynamicModel] =  BuildMassSpringRunner(params)

% % the phase transition map for this particular system
dynamicModel.phaseTransitionMap = ...
    {{{2}, % If the previous phase was 1 with event 1, the next is 2
    {0}}, %Unless the CoM "fell" (event 2)(reached ground height)
     {{3}, % If the previous phase was 2 with event 1, the next is 3
     {1, @(x)ApexTransitionFunction(x,params)}, % If the previous phase was 2 with event 2 (neg. vert velocity), the next is 1 (skip phase 3)
     {0}},%Unless the CoM "fell"(event 3) (reached ground height)
     {{1, @(x)ApexTransitionFunction(x,params)},%If the previous phase was 3, the next is 1, with ApexTransitionFunction
     {0}}}; %Unless the CoM "fell" (reached ground height)

% Description of phases by their component functions (dynamic equations,
% event functions, coordinate axes translation functions
dynamicModel.phases = ...
    {{@(t,x)BallisticPhase1(t,x,params),  @(t,x)EndBallisticPhase1(t,x,params),  @(x)GlobalToBallistic(x,params),   @(x,data)BallisticToGlobal(x,data,params)},
    { @(t,x)StancePhase(t,x,params),     @(t,x)EndStancePhase(t,x,params),    @(x)GlobalToStance(x,params),   @(x,data)StanceToGlobal(x,data,params)},
    { @(t,x)BallisticPhase2(t,x,params),   @(t,x)EndBallisticPhase2(t,x,params),  @(x)GlobalToBallistic(x,params),   @(x,data)BallisticToGlobal(x,data,params)}};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newGlobalState] = ApexTransitionFunction(oldGlobalState,params)

newGlobalState(1:4,:) = oldGlobalState(1:4,:);
newGlobalState(5,:) = oldGlobalState(1,:)-(params.Lnot.*cos(params.apexLegTheta));
newGlobalState(6,:) = oldGlobalState(2,:)-(params.Lnot.*sin(params.apexLegTheta));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [globalState] =  BallisticToGlobal(localState,localToGlobalData,params)

%Translate theta of leg to Xfoot and Yfoot
Lnot = params.Lnot;
globalState(1:4,:) = localState(1:4,:);
globalState(5,:) = localState(1,:) - Lnot.*cos(localState(5,:));
globalState(6,:) = localState(2,:) - Lnot.*sin(localState(5,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [localState,localToGlobalData] =  GlobalToBallistic(globalState,params)

%Translate Xfoot and Yfoot to theta of leg
localToGlobalData = [];
localState(1:4,:) = globalState(1:4,:);
deltaXY = globalState(1:2,:) - globalState(5:6,:);
localState(5,:) = atan2(deltaXY(2,:),deltaXY(1,:));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [globalState] = StanceToGlobal(localState,absFootPosition,params)

%Move origin from foot back out to global coordinate system
footPosMatrix = repmat(absFootPosition,1,size(localState,2));
globalState(1:2,:) = localState(1:2,:) + footPosMatrix;
globalState(3:4,:) = localState(3:4,:);
globalState(5:6,:) =footPosMatrix;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [localState,absFootPosition] = GlobalToStance(globalState, params)

%Move origin to foot, the only states are associated with the body position
%and velocity
absFootPosition = globalState(5:6,1);
footPosMatrix = repmat(absFootPosition,1,size(globalState,2));
localState(1:2,:) = globalState(1:2,:) - footPosMatrix;
localState(3:4,:) = globalState(3:4,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [statedot] = BallisticPhase1(t,state,params)
%Equations of motion for swing phase, from apex
g = params.g; %The only force acting on the system is gravity

%equations of motion
statedot(1) = state(3);
statedot(2) = state(4);
statedot(3) = 0;
statedot(4) = -g;
statedot(5) = params.legVel; %radians/second
statedot = statedot';

%m=state(5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [statedot] = BallisticPhase2(t,state,params)
%Equations of motion for first part of swing phase
g = params.g; %The only force acting on the system is gravity

%equations of motion
statedot(1) = state(3);
statedot(2) = state(4);
statedot(3) = 0;
statedot(4) = -g;
statedot(5) = 0;
statedot = statedot';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [value,isterminal,direction] = EndBallisticPhase1(t,state,params)
%Event function for determining the end of ballistic phase 1
%Ballistic1 starts when the body is at its highest point during swing (apex)
%Ends when the foot contacts the ground
% Or when the body CoM reaches the ground (a fall)

Lnot = params.Lnot;
y = state(2); %body vertical position
footPosition = y - Lnot.*sin(state(5)); 

value(1) = footPosition;
value(2) = y;

isterminal = [1 1];
direction = [-1 -1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [value,isterminal,direction] = EndBallisticPhase2(t,state,params)
%Event function for determining when the 2nd ballistic phase ends
%Ballistic2 starts when the toe leaves the ground (end stance)
%Event 1: The body reaches its highest position (apex)
%Event 2: The body CoM reaches the ground (a fall)

y = state(2); %body vertical position
ydot = state(4);%body vertical velocity

value(1) = ydot;
value(2) = y;

isterminal = [1 1];
direction = [-1 -1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [statedot] = StancePhase(t,state,params)
% Equations of motion for stance phase
% For mass spring system, force is proportional to the length of the spring

% parameters needed for equations of motion
g = params.g; %gravity
k = params.k;%spring stiffness
m = params.m;%body mass in kg
Lnot = params.Lnot;%resting length of the spring

% Current leg angle relative to right hand horizontal
theta = atan2(state(2),state(1));

% Force is proportional to the change in length of the spring
% Don't allow the spring to be stretched beyond Lnot:
Lcurrent = min(((state(1)^2 +state(2)^2)^.5),Lnot);
% Lcurrent = ((state(1)^2 +state(2)^2)^.5);
deltaL = (Lnot - Lcurrent);

%equations of motion
statedot(1) = state(3);
statedot(2) = state(4);
statedot(3) = (k/m)*deltaL*cos(theta);
statedot(4) = (k/m)*deltaL*sin(theta) - g;
statedot = statedot';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [value,isterminal,direction]  = EndStancePhase(t,state,params)
% Event function for determining when stance phase ends
% Stance phase ends when the spring returns to resting length
% or the body hits the ground

currentLegLength = (state(1)^2 + state(2)^2)^.5;
Lnot = params.Lnot;

y = state(2);
isPositiveVertVel = sign(state(4))==1;

%If vertical velocity is positive, continue to next phase after event
%(event 1). If vertical velocity is negative, provide a different event
%value that triggers a different next phase. 

if isPositiveVertVel
value(1) = Lnot - currentLegLength; 
value(2) = 1;
else
value(1) = 1;
value(2) = Lnot - currentLegLength;
end

value(3) = y;

isterminal = [1 1 1];
direction = [-1 -1 -1];

