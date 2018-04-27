function [progressMovie, handle] = MakeProgressMovie_wForces(bodyStates, footStates, time, params, varargin)
%MakeProgressMovie Movie where foot progresses across screen
%
%   MakeProgressMovie_wForces(bodyStates, footStates, time, params, varargin)
%
% To control the output style, use
%
%   MakeProgressMovie_wForces(bodyStates, footStates, time, params, labelGraph, subPlotData)
%
% where labelGraph and subPlotData are optional. Set labelGraph to 0 if you
% want no labels on the progressGraph. If subPlotData is added, will create
% a graph of that data underneath the spring mass system, synced to the
% animation.


colordef white;

if nargin < 5
    labelGraph = 1;
else
    labelGraph = varargin{1};
end
if nargin < 6
    subPlotData = 0;
else
    subPlotData = varargin{2};
end

if nargin < 7
    eventIndex = size(bodyStates,2) +1;
else
    eventIndex = varargin{3};
end

if isempty(eventIndex)
    eventIndex = size(bodyStates,2) +1;
end

Lnot = params.Lnot;
% Make sure that spring is not plotted as longer than Lnot during stance:
localState(1:2,:) = bodyStates(1:2,:) - footStates(1:2,:);
Lcurrent = min(((localState(1,:).^2 +localState(2,:).^2).^.5),Lnot);
theta = atan2(localState(2,:),localState(1,:));
newFootStates(1,:) = bodyStates(1,:) - Lcurrent.*cos(theta);
newFootStates(2,:) =  bodyStates(2,:) - Lcurrent.*sin(theta);

widthBuffer = Lnot/2.0; % Extra padding on either side to make view look nice
heightBuffer = Lnot/3.0;
groundHeight = 0.0; %#ok<NASGU> % TODO: make this handle other ground shapes

xmin = min(bodyStates(1,:))-widthBuffer;
xmax = max(bodyStates(1,:))+widthBuffer;
ymin = min(footStates(2,:));
ymax = abs(max(bodyStates(2,:)))+heightBuffer;

timeStepIndex = floor(params.movieTimeStep / params.timeStep);
timeIndices = 1:timeStepIndex:size(bodyStates,2);

if ~isscalar(subPlotData); %isscalar(subPlotData)
    subplotXmin = min(time);
    subplotXmax = max(time);
    datamaxY = max(max(subPlotData)); % 4*BW
    dataminY =  min(min(subPlotData)); % -1*BW
    subplotYmin = dataminY - (datamaxY-dataminY)*0.15;
    subplotYmax = datamaxY + (datamaxY-dataminY)*0.15;
end

progressMovie = VideoWriter('ProgressMovie.avi','Uncompressed AVI');
progressMovie.FrameRate = 5;
open(progressMovie);
% 
%progressMovie = avifile('ProgressMovie.avi','compression','none');

handle = figure;
% set(handle,'DoubleBuffer','on'); % flicker free animation

% Set the position so that is has exactly the right screen aspect ratio
% set(handle,'Position', [0 0 1280 800]);
% set(handle,'units','normalized');
% set(handle,'Position',[0 0.0364583 1 0.875]);
set(handle,'Color',[1 1 1]);

for timeIndex = 1:length(timeIndices),
    i = timeIndices(timeIndex);
    if ~isscalar(subPlotData);
        subplot(3,1,[1:2]);
    end
    hold on;
    
    if  footStates(2,i) <= 0.0001  % bodyStates(4,i) <= 0 || footStates(2,i) <= 0.001
    plotSpring([bodyStates(1, i) bodyStates(2, i)], [newFootStates(1, i) newFootStates(2, i)],  Lnot*0.15,'k');
    % 	plotSpring([bodyStates(1, i) bodyStates(2, i)], [newFootStates(1, i) newFootStates(2, i)],  Lnot*0.15, 'w', 'LineWidth', 2);
    end
    hold on;
    if i < eventIndex
        markerColor = 'c';
    else
        markerColor = 'r';
    end

    plot(bodyStates(1, i), bodyStates(2, i), 'ro','LineWidth',2,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',markerColor,...
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

    if ~isscalar(subPlotData);
        subhandle = subplot(3,1,3);
        %         plot((timeIndices(1:timeIndex)-1)*params.timeStep,
        %         subPlotData(:,timeIndices(1:timeIndex))', 'LineWidth', 2);
        %         if subPlotData(2,timeIndices(timeIndex))>0
        plot(time(timeIndices(1:timeIndex)), subPlotData(:,timeIndices(1:timeIndex))', 'k','LineWidth', 1.5);
        axis([subplotXmin subplotXmax subplotYmin subplotYmax]);
        set(subhandle, 'Color', get(handle,'Color')); %Make subplot background the same as figure background
%         set(subhandle,'XTick',zeros(1,0)); % Get rid of x axis ticks
%         set(subhandle,'YTick',zeros(1,0));
        xlabel('Time (s)');
        ylabel('GRF (N/mg))');
        box off

    end
    
  currFrame = getframe(handle); 
  writeVideo(progressMovie,currFrame);
end
  %movie2avi(progressMovie, 'progressMovie.avi', 'COMPRESSION', 'None');
  %progressMovie = close(progressMovie);
  close(progressMovie);
end