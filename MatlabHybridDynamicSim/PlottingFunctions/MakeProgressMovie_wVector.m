function [progressMovie, handle] = MakeProgressMovie_wVector(bodyStates,footStates, params, varargin)
%MakeProgressMovie Movie where foot progresses across screen
%   MakeProgressMovie(bodyStates, footStates, params,varargin)
% To control the output style, use
%   MakeProgressMovie(bodyStates, footStates, params, labelGraph, subPlotData)
%
% where labelGraph and subPlotData are optional. Set labelGraph to 0 if you
% want no labels on the progressGraph. If subPlotData is added, will create
% a graph of that data underneath the spring mass system, synced to the
% animation.

if nargin < 3
    labelGraph = 1;
else
    labelGraph = varargin{1};
end
if nargin < 4
    subPlotData = 0;
else
    subPlotData = varargin{2};
end

Lnot = params.Lnot;
widthBuffer = Lnot/2.0; % Extra padding on either side to make view look nice
heightBuffer = Lnot/3.0;
groundHeight = 0.0; % TODO: make this handle other ground shapes

xmin = min(bodyStates(1,:))-widthBuffer;
xmax = max(bodyStates(1,:))+widthBuffer;
ymin = min(footStates(2,:));
ymax = max(bodyStates(2,:))+heightBuffer;

timeStepIndex = floor(params.movieTimeStep / params.timeStep);
timeIndices = 1:timeStepIndex:size(bodyStates,2);

if ~(length(subPlotData))==1; %isscalar(subPlotData)
    datamaxX = max(subPlotData(3,:));
    dataminX = min(subPlotData(3,:));
    subplotXmin = dataminX - (datamaxX-dataminX)*0.15;
    subplotXmax = datamaxX + (datamaxX-dataminX)*0.15;

    datamaxY = max(subPlotData(4,:));
    dataminY = min(subPlotData(4,:));
    subplotYmin = dataminY - (datamaxY-dataminY)*0.15;
    subplotYmax = datamaxY + (datamaxY-dataminY)*0.15;
end

handle = figure;
set(handle,'units','normalized');
set(handle,'Position',[0 0.0364583 1 0.875]);
for timeIndex = 1:length(timeIndices),
    i = timeIndices(timeIndex);
    if ~(length(subPlotData))== 1;
        subplot(3,1,[1:2]);
    end
    hold off;
    plotSpring([bodyStates(1, i) bodyStates(2, i)], [footStates(1, i) footStates(2, i)],  Lnot*0.15, 'w', 'LineWidth', 2);
    % plot([allx_resampled(i) allFootStatesX_resampled(i)], [ally_resampled(i) allFootStatesY_resampled(i)]);
    hold on;
    plot(bodyStates(1, i), bodyStates(2, i), 'ro','LineWidth',2,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor','c',...
        'MarkerSize',30);
    %plotGround(constants.groundCoords);
    plot([xmin xmax], [0.0 0.0], 'Color', [.5 .5 .5], 'LineWidth', 2);

    xlabel('X position [m]');
    ylabel('Y position [m]');
    axis equal;
    axis([xmin xmax ymin ymax]);
    if ~labelGraph
        axis off;
    else
        box on
    end

    if ~(length(subPlotData))== 1;
        subhandle = subplot(3,1,3);
        %         plot((timeIndices(1:timeIndex)-1)*params.timeStep,
        %         subPlotData(:,timeIndices(1:timeIndex))', 'LineWidth', 2);
        if subPlotData(4,timeIndices(timeIndex))>0
            quiver(subPlotData(1,i),subPlotData(2,i),subPlotData(3,i),subPlotData(4,i),0,'y');
            axis([subplotXmin subplotXmax subplotYmin subplotYmax]);
        else
        end
        axis([subplotXmin subplotXmax subplotYmin subplotYmax]);
        % TODO: Put these options somewhere else more general
        set(subhandle, 'Color', get(handle,'Color')); %Make subplot background the same as figure background
        set(subhandle,'XTick',zeros(1,0)); % Get rid of x axis ticks
        set(subhandle,'YTick',zeros(1,0));
        %     xlabel('Time');
        ylabel('GRF');
        box off

    end
    progressMovie(timeIndex) = getframe(gcf);
end

movie2avi(progressMovie,[cd 'MassSpringProgressMovie.avi'],'FPS', 30, 'COMPRESSION', 'None');
end