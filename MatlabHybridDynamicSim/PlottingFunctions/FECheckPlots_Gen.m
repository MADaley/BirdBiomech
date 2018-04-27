function [fh] = FECheckPlots_Gen(timeOneStride,groundForces, groundImpulse, energyStorage)

timeOneStride = OneStrideData.timeOneStride;
groundImpulse = OneStrideData.groundImpulse;
groundForces = OneStrideData.groundForces;
energyStorage = OneStrideData.energyStorage;

%check force and energy outputs
disp('Force output (dimensionless): J, Jangle, Fmax, Tc'); disp(groundImpulse);%Print impulse measures: J magnitude, J angle, Fmax and Duration of contact (to provide indication of shape)
%(divide absolute work by 2 to get spring energy storage. 
%For the simple mass-spring system, net change in energy (5) should always be zero 
%unless something went wrong with the model (because the model is a conservative system)

fh(1) = figure;
plot(timeOneStride,groundForces(1:2,:)')
legend('for-aft', 'vertical')
ylabel('Ground Forces (bw)')
title('Ground forces during stance')
xlabel('Time (s)')

fh(2) = figure;
 plot (timeOneStride,energyStorage)
 ylabel('Spring Energy Storage (J/mgL)')
 xlabel('Time (s)')
