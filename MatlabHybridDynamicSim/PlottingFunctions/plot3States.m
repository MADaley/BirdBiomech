function [handle] = plot3States(xVector,yVector,zVector,labels,inHandle,axLims)

if exist('inHandle', 'var')
    figure(inHandle)
    handle =inHandle;
else    
handle = figure;
end
hold all;
grid on;
view([-45,20])
xlabel(labels{1})
ylabel(labels{2})
zlabel(labels{3})

if exist('axLims','var')
xlim(axLims(1,:))
ylim(axLims(2,:))
zlim(axLims(3,:))
end
plot3(xVector,yVector,zVector,'x')
hold all;
