function [centerMovie, handle] = MakeMassSpringMovie(bodyStates, footStates, params)
% Movie where body stays centered
% TODO: make movie progress at constant horizontal velocity

Lnot = params.Lnot;
halfWindowWidth = Lnot;
%meanheight = mean(ally_resampled);
windowHeightMax = Lnot/2.0 + bodyStates(2,1);
windowHeightMin = -Lnot/8.0;

timeStepIndex = floor(params.movieTimeStep / params.timeStep);
timeIndices = 1:timeStepIndex:size(bodyStates,2);

handle = figure;
set(handle,'units','normalized');
set(handle,'Position',[0.05 0.05 0.45 0.5]);

for timeIndex = 1:length(timeIndices),
    i = timeIndices(timeIndex);
    hold off;
    plotSpring([bodyStates(1, i) bodyStates(2, i)], [footStates(1, i) footStates(2, i)],  Lnot*0.15);
    %plot([allx_resampled(i) allFootStatesX_resampled(i)], [ally_resampled(i) allFootStatesY_resampled(i)]);
    hold on;
    plot(bodyStates(1, i), bodyStates(2, i), 'ro','LineWidth',2,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[.49 1 .63],...
        'MarkerSize',15 );
    %plotGround(constants.groundCoords);
    xlabel('X position [m]');
    ylabel('Y position [m]');
    axis equal;
    axis([bodyStates(1, i)-halfWindowWidth bodyStates(1, i)+halfWindowWidth windowHeightMin windowHeightMax]);

    centerMovie(timeIndex) = getframe(gcf);
end

movie2avi(centerMovie,[cd '\MassSpringMovieCentered.avi'],'FPS', 30, 'COMPRESSION', 'None');

end
