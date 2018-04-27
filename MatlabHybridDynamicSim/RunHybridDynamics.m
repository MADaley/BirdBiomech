function [statesOverTime, phasesOverTime, timeVector, transitions, currentPhaseIndex, exitCode, solverSteps] = RunHybridDynamics(initialStates, dynamicModel, timeParams, odeSolverH, odeOptPairs)
% [statesOverTime, phasesOverTime, timeVector, transitions] = RunHybridDynamics(initialStates, dynamicModel, timeParams)
%
% General hybrid dynamic simulator, that allows any number of "dynamic
% phases" to be strung together according to the associated event
% functions and phaseTransitionMap.
% 
% Author:  Monica Daley 2007
%  
% To run, the minimal inputs are:
%   initialStates - initial conditions in the global coordinate system
%   dynamicModel  - a structure describing the dynamic functions of the system
%
% ------------
% dynamicModel 
% ------------
% 
% dynamicModel is a structure containing one to two elements.
%
% dynamicModel.phases: A mandatory cell array of functions with the following structure:
% dynamicModel.phases = ...
%    {{@(t,x)DynamicPhase1(t,x,params), @(t,x)EndDynamicPhase1(t,x,params), @(x)GlobalToLocal1(x,params), @(x,data)LocalToGlobal1(x,data,params)},
%     {@(t,x)DynamicPhase2(t,x,params), @(t,x)EndDynamicPhase2(t,x,params), @(x)GlobalToLocal2(x,params), @(x,data)LocalToGlobal2(x,data,params)},
%     {@(t,x)DynamicPhase3(t,x,params), @(t,x)EndDynamicPhase3(t,x,params), @(x)GlobalToLocal3(x,params), @(x,data)LocalToGlobal3(x,data,params)}};
%  
% Each set of dynamic phase functions consists of at least:
%  (1)   a set of dynamic state equations. 
%  Optional additional functions:
%  (2)   the associated event functions.
%  (3-4) coordinate system translation functions
%        (globalToLocal and localToGlobal)
%  Valid number of functions for each phase: 1,2 or 4
%  
% dynamicModel.phaseTransitionMap is an optional field.  If included, it
% determines the mapping between dynamic phases according to (1) which phase the
% system was in when the event occured, (2) which event was triggered. Each
% combination of phase and event is mapped to the next phase with the phase
% transition map.  A transition function is included, when necessary, to reset
% states before the next phase or handle a collision event. If no
% phaseTransitionMap is included, the simulator runs through the phases in the
% order that they are defined in the dynamicModel.phases array of phase
% functions.
%
% See Example_MassSpringRunner and Example_BallInBox for a couple of simple
% phaseTransitionMaps.
%
% The phaseTransitionMap must have one entry for each phase, event
% combination and must at least specify the next event.  For example, in the
% simple MassSpringRunner example, there are 3 dynamic phases. Only one of these
% phases has a transition function, and there is only one event associated
% with each phase.  The phaseTransitionMap looks like this:
%
% phaseTransitionMap = ...
%  {{{2}},   % If the previous phase was 1, the next is 2
%   {{3}},   % If the previous phase was 2, the next is 3
%   {{1, @(x)ApexTransitionFunction(x,params)}}}; % If the previous phase was 3,
%              the next is 1, applying the ApexTransitionFunction in between.
% 
% If each phase is associated with multiple possible events (for example, if
% a fall check were included in each event function in the MassSpringRunner,
% then there must be an entry for each possible event for each phase.  In
% the above structure, there would be six entries instead of three. In the
% BallInBox example, there is only one dynamic phase, but 4 possible events
% (hitting each of the four walls). The phaseTransitionMap is:
%
% phaseTransitionMap = ...
%  {{{1, @(x)WallTransitionFunction(x,-params.cor,1)},
%    {1, @(x)WallTransitionFunction(x,-params.cor,1)},
%    {1, @(x)WallTransitionFunction(x,1,-params.cor)},
%    {1, @(x)WallTransitionFunction(x,1,-params.cor)}}};
%  
% ------------
% timeParams 
% ------------
%
% timeParams can contain the following (if none included, use empty matrix):
%
% timeParams.timeLimit: a set time limit for the simulation
%
% timeParams.timeStep: sets a specific time step for the output (for
%    plotting purposes only, this does not change the actual simulation timeStep
%    used by ode45)
%
% timeParams.maxNumPhases: set a specific number of phases to execute. This
%    can be used to cycle through a specific number of strides in a locomotor
%    simulation, for example, by setting the value to strides times the number
%    of phases per stride.
%
% timeParams.maxNumSpPhase: set a specific number of times for a single phase to
% to execute. This is a vector with two values:
%     maxNumSpPhase(1) = phase to count
%     maxNumSpPhase(2) = number of times to execute it.
%
% This can be used to set the simulation to stop after a specific number of strides,
% when the stride can sometimes contain different numbers of phases but are counted,
% by the repetition of a certain phase.  For example, in the mass-spring runner, ballistic
% phase 1 and ballistic phase 2 are counted as different phases (phase during negative
% vertical velocity and phase during positive vertical velocity). However, in some cases
% the second phase (positive vertical velocity) does not occur because the system isn't
% in a steady state condition.  In this case, a stride could be counted by the number of
% times phase1 is executed.
%  
% timeParams.maxCPUTime: set a time limit, in seconds, on the actual elapsed
%   computation time (not simulation time). For example, if you would like to
%   guarantee that the simulation does not run for longer than 1 minute, set
%   maxCPUTime to 60.0. This is accomplished by adding an addition event to each
%   simulation. maxCPUTime uses tic and toc.
%
% If no values are included timeParams, a default timeLimit (5s) and
% timeStep (0.01s) will be used.
%
% ------------
% odeSolverH
% ------------
%
% odeSolverH is a handle to an ode solver function. It must be a function
%  that takes exactly the same input options as ode45
%  
% ------------
% Outputs
% ------------
%
% [statesOverTime, phasesOverTime, timeVector, transitions, currentPhaseIndex, exitCode]
% where
% transitions = {phaseEndTime, phaseEndState, currentState, eventIndex};
%  
% exitCode:
%   0 = model failure
%  -1 = time limit reached
%  -2 = maxNumPhases reached
%  -3 = maxNumSpPhase reached
%  -4 = maxCPUTime reached
%
% See Example_MassSpringRunner and ExampleBallInBox and the associated
% functions for more details.


% Check the time parameters and determine conditions for stopping simulation
% User can specify a time limit or a maximum number of phase transitions. If
% no value is included, a default time limit will be used.
if ~isfield(timeParams,'timeLimit')
    timeParams.timeLimit = 5;
    disp(['Using default time limit of ' num2str(timeParams.timeLimit) ' seconds']);
end

if ~isfield(timeParams,'timeStep')
    timeParams.timeStep = 0.01;
    disp(['States output using default time step of ' num2str(timeParams.timeStep) ' seconds']);
end

% Check to see if a maximum number of phases has been set
noPhaseLimit = ~isfield(timeParams,'maxNumPhases');

% Check to see if a maximum number of a single phase has been set
noSinglePhaseLimit = ~isfield(timeParams,'maxNumSpPhase');

% Check if we are using a maximum actual elapsed time
noCPUTimeLimit = 1;
if isfield(timeParams,'maxCPUTime')
  noCPUTimeLimit = 0;
  tic;
end
  
% Check to see if a transition map was included
hasTransitionMap = isfield(dynamicModel, 'phaseTransitionMap');

% Number of phases in the set of functions included
numPhases = length(dynamicModel.phases);

currentPhaseIndex = 1;
currentState = initialStates;
phaseEndTime = 0;
timeVector=[];
statesOverTime=[];
phasesOverTime=[];
transitions={};
numCompletedPhases = 0;
numCountPhase = 0;
exitCode = 1;

if ~isfield(timeParams,'maxNumSpPhase')
    countPhase = 1;
else
    countPhase = timeParams.maxNumSpPhase(1);
end

if ~exist('odeSolverH','var')
    odeSolverH = @ode45;
elseif isempty(odeSolverH)
    odeSolverH = @ode45;
end

if ~exist('odeOptPairs','var')
    inOptions = odeset([]);
elseif isempty(odeOptPairs)
    inOptions = odeset([]);
end

% Continue simulation until a zero is returned for the next phase index, the
% time limit is reached, the maximum number of phase transitions is
% reached, or the maximum actual time has elapsed
while exitCode >0

    phaseStartTime = phaseEndTime;
    currentPhase = dynamicModel.phases{currentPhaseIndex};
    % Time vector for output, ode45 uses variable time steps
    timeSpan = [phaseStartTime timeParams.timeLimit];

    % check for the proper number of input functions, either 1, 2 or 4:
    % At least 1-2 must be provided:  a dynamic function and optional event function
    % If translation into a different coordinate systems is required,
    % two additional functions must be included.
    if ~(length(currentPhase)==1 || length(currentPhase)==2|| length(currentPhase)==4)
        error(['Incorrectly formatted input functions in phase ' num2str(currentPhaseIndex)]);
    end

    % Set the current phase dynamic functions, event functions, and
    % coordinate system translation functions, if required
    DynamicFunction = currentPhase{1};

    if length(currentPhase) >1
        EventFunction = currentPhase{2};
        newOptions = odeset('Events', EventFunction);
        odeOptions = odeset(inOptions,newOptions);
    else
        odeOptions = inOptions;
    end

    if length(currentPhase) >2
        GlobalToLocalFunction = currentPhase{3};
        LocalToGlobalFunction = currentPhase{4};
        [localState,localToGlobalData] = GlobalToLocalFunction(currentState);
    else
        localState = currentState;
    end
    
    % Augment the current event function with elapsed time checker
    if ~noCPUTimeLimit
      if length(currentPhase) > 1
        % Current event function already exists
        NewEventFunction = @(t,x)CPUTimeCheckEventWrap(t,x,EventFunction,timeParams.maxCPUTime, phaseStartTime);
      else
        NewEventFunction = @(t,x)CPUTimeCheckEvent(t,x,timeParams.maxCPUTime, phaseStartTime);
      end
      newOptions = odeset('Events', NewEventFunction);
      odeOptions = odeset(inOptions,newOptions);
    end

    % Run the simulation and output the solution as a structure
    currentSolution = feval(odeSolverH,DynamicFunction, timeSpan, localState, odeOptions);
    solverSteps = length(currentSolution.x);

    % Get end time
    if ~isfield(currentSolution,'xe')
        phaseEndTime = currentSolution.x(end);
    elseif isempty(currentSolution.xe)
        phaseEndTime = currentSolution.x(end);
    else
        phaseEndTime = currentSolution.xe(end);
    end

    % Create current time vector
    if isempty(timeVector)
        currentTimeSpan = (0:timeParams.timeStep:phaseEndTime)';
    else
        currentTimeSpan = ((timeVector(end)+timeParams.timeStep):timeParams.timeStep:phaseEndTime)';
    end

    % If events happen very quickly, the phase duration can be shorter than
    % the user input timestep.  The program needs to handle this but warn
    % the user.

    if isempty(currentTimeSpan)
        warning('Phase duration shorter than time step (%f). You may want to use a smaller time step. ', timeParams.timeStep); %#ok<WNTAG>
        % The user input timestep results in no output points for the
        % current phase, however, the end state and time were still updated
        % at a smaller time interval

        % Output states are empty matrices
        phaseOverTime = [];
        currentStatesOverTime = [];

        % Solve for the phaseEndState
        localphaseEndState = deval(currentSolution,phaseEndTime);

        % Map to global coordinate system
        if length(currentPhase) >2
            phaseEndState = LocalToGlobalFunction(localphaseEndState,localToGlobalData);
        else
            phaseEndState = localphaseEndState;
        end
    else % If there are output timesteps to solve for as well as the final state for the phase

        % Create current statesOverTime
        phaseOverTime=ones(1, length(currentTimeSpan))'*currentPhaseIndex;
        localStatesOverTime = deval(currentSolution,currentTimeSpan);
        localphaseEndState = deval(currentSolution,phaseEndTime);

        % Map to global coordinate system
        if length(currentPhase) >2
            currentStatesOverTime = LocalToGlobalFunction(localStatesOverTime,localToGlobalData);
            phaseEndState = LocalToGlobalFunction(localphaseEndState,localToGlobalData);
        else
            currentStatesOverTime = localStatesOverTime;
            phaseEndState = localphaseEndState;
        end

    end

    % Concatenate with all previous phase data
    timeVector = [timeVector ; currentTimeSpan]; %#ok<AGROW>
    statesOverTime = [statesOverTime  currentStatesOverTime]; %#ok<AGROW>
    phasesOverTime = [phasesOverTime ; phaseOverTime]; %#ok<AGROW>

    % Start setting up for the switch to the next phase
    currentState =  phaseEndState;
    
    % Check the elapsed time
    exceededCPUTime = ~noCPUTimeLimit && ...
        isfield(currentSolution,'ie') && ...
        ~isempty(currentSolution.ie) && ...
        currentSolution.ie(1) == 1;

    % Determine the next phase
    if phaseEndTime < timeParams.timeLimit && ~exceededCPUTime

        if isfield(currentSolution,'ie')

            eventIndex = currentSolution.ie(1);
            if ~noCPUTimeLimit
              eventIndex = eventIndex - 1; % Adjust for added time event
            end

            % Apply the transition function (collisions, resetting of states, etc.),
            % if applicable. Otherwise, just move onto the next phase.
            if hasTransitionMap
                % Move to the next phase as determined by the phase transition map
                try
                    if length (dynamicModel.phaseTransitionMap{currentPhaseIndex}{eventIndex})==2
                        TransitionFunction = dynamicModel.phaseTransitionMap{currentPhaseIndex}{eventIndex}{2};
                        currentState = TransitionFunction(currentState);
                    end
                    nextPhaseIndex = dynamicModel.phaseTransitionMap{currentPhaseIndex}{eventIndex}{1};
                catch
                    error(['Incorrectly formatted phase transiton map for phase, event: ' num2str(currentPhaseIndex) ',' num2str(eventIndex)]);
                end
            else
                % Default phase transition settings:
                % Cycle through phases in order, incrementing by one each time
                nextPhaseIndex = mod(currentPhaseIndex,numPhases)+1;
            end

            % Check that the next phase index is valid, throw an error if not
            if nextPhaseIndex < 0 || nextPhaseIndex > numPhases
                error('Incorrect next phase:%d from phase %d with event %d', nextPhaseIndex,  currentPhaseIndex, eventIndex);
            end

            % Save the transition information to an array that can be used to
            % analyze the results
            transitions{end+1}={phaseEndTime, phaseEndState, currentState, eventIndex}; %#ok<AGROW>
            currentPhaseIndex = nextPhaseIndex;
        else
            % simulation failure
            currentPhaseIndex = 0;
            error('Simulation terminated before time limit without event. Check ode settings');
        end
    end
    
    % Increment phase counts while loop check at the top
    numCompletedPhases = numCompletedPhases +1;
    if currentPhaseIndex==countPhase
        numCountPhase = numCountPhase +1;
    end

    % Set exitCode checks
    if currentPhaseIndex == 0
      exitCode = 0;
    elseif exceededCPUTime
      exitCode = -4; % Want this to take precedence over other exit codes
    elseif (phaseEndTime >= timeParams.timeLimit)
      exitCode = -1;
    elseif (~noPhaseLimit && numCompletedPhases >= timeParams.maxNumPhases)
      exitCode = -2;
    elseif (~noSinglePhaseLimit && numCountPhase >= timeParams.maxNumSpPhase(2))
      exitCode = -3;
    end

end % End while loop

function [value,isterminal,direction] = CPUTimeCheckEventWrap(t, x, OldEventFunction, maxCPUTime, startTime)
% Wrapper event function to add an overall elapsed time check (at the
% beginning).  Note that event functions always have to transition to 0 for it
% to trigger, so add a check that the evaluation time is at least some tolerance ahead of the start time.

tol = 1e-10;
[value,isterminal,direction] = feval(OldEventFunction, t, x);
value = [double(t < startTime+tol || toc <= maxCPUTime), value];
isterminal = [1, isterminal];
direction = [0, direction];

function [value,isterminal,direction] = CPUTimeCheckEvent(t, x, maxCPUTime, startTime)
% Event function to add an overall elapsed time check

tol = 1e-10;
value = double(t < startTime+tol || toc <= maxCPUTime);
isterminal = 1;
direction = 0;
  
  