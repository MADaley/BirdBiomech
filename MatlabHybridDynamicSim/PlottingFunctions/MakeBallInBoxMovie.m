function [progressMovie, handle] = MakeBallInBoxMovie(statesOverTime, params, timeParams)

xmin = params.wallXmin;
xmax = params.wallXmax;
ymin = params.wallYmin;
ymax = params.wallYmax;
ballSize = (params.r*2);

if params.movieTimeStep < timeParams.timeStep
    params.movieTimeStep = timeParams.timeStep
    warning('Move time step less than simulation time step. The larger time step was used.')
end

timeStepIndex = floor(params.movieTimeStep / timeParams.timeStep);
timeIndices = 1:timeStepIndex:size(statesOverTime,2);

handle = figure;
set(handle,'units','normalized');
set(handle,'Position',[0.05 0.05 0.45 0.5]);

for timeIndex = 1:length(timeIndices),
    i = timeIndices(timeIndex);
    hold off;
    plot([xmin xmax xmax xmin xmin ], [ymin ymin ymax ymax ymin], 'Color', [.5 .5 .5], 'LineWidth', 2);
    hold on;

%Rectangle can plot a rectangle, curved rectangle or ellipse of specifed
%demensions and curvature
    rectangle ('position', [(statesOverTime(1, i)-params.r), (statesOverTime(2, i)-params.r), ballSize, ballSize],...
                'curvature', [1, 1],...
                'FaceColor', 'c',...
                'EdgeColor', 'k',...
                'LineWidth', 0.2)

    plot([xmin xmax xmax xmin xmin ], [ymin ymin ymax ymax ymin], 'Color', [.5 .5 .5], 'LineWidth', 2);

    xlabel('X position [m]');
    ylabel('Y position [m]');
    axis equal;
    axis([xmin xmax ymin ymax]);

    progressMovie(timeIndex) = getframe(gcf);
end


