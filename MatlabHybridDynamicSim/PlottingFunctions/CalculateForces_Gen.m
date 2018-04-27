function [summaryData, forces, energyStorage] = CalculateForces_Gen(statesOneStride,phasesOverTime,params,legInd)
% Calculates ground forces and leg spring energy fluctuations for SLIP model

bodystate=statesOneStride(1:2,:);
footstate=statesOneStride(legInd,:);
g=params.g;
k=params.k;
m=params.m;
Lnot=params.Lnot;
timeStep=params.timeStep;

if legInd(1) == 5
    isStance = (phasesOverTime==2)';
    isBallistic1 = (phasesOverTime==1)'; % phase corresponding to the last part of swing, from apex to toe down
    isBallistic2 = (phasesOverTime==3)'; % phase corresponding to the first part of swing, from  toe off to apex
elseif legInd(1) ==7
    isStance = (phasesOverTime==5)';
    isBallistic1 = (phasesOverTime==4)'; % phase corresponding to the last part of swing, from apex to toe down
    isBallistic2 = (phasesOverTime==6)'; % phase corresponding to the first part of swing, from  toe off to apex
end

ball1Period = (length((find(isBallistic1==1))).*timeStep);
ball2Period = (length((find(isBallistic2==1))).*timeStep);
% total swing period, assuming a two legged runner:
% ballistic2 + ballistic1 + swing + ballistic2 +ballistic 2
% or: 2*ballistic2 + 2*ballistic1 + stance
stancePeriod = (length((find(isStance==1))).*timeStep);%Duration of of contact
swingPeriod = 2*ball2Period + 2*ball1Period + stancePeriod;
stridePeriod = (stancePeriod + swingPeriod);
stepPeriod = stridePeriod/2;

stepLength = statesOneStride(1,end) - statesOneStride(1,1);
bounceHeight = max(bodystate(2,:));
vertDispTot = bounceHeight - min(bodystate(2,:));
vertDispStance = max(bodystate(2,isStance==1)) - min(bodystate(2,isStance==1));

meanVelocity(1) = mean(statesOneStride(3,:));
meanVelocity(2) = mean(statesOneStride(4,:));

newState=ones(2,size(bodystate,2));
newState(1,:) = newState(1,:).*(bodystate(1,:) - footstate(1,:));
newState(2,:) = newState(2,:).*(bodystate(2,:) - footstate(2,:));

theta=atan2(newState(2,:),newState(1,:));

toeDownIndex = find(isStance==1,1,'first');
toeOffIndex = find(isStance==1,1,'last');
beginStanceAngle = theta(toeDownIndex);
endStanceAngle = theta(toeOffIndex);

vVel_TD = statesOneStride(4,toeDownIndex);
hVel_TD = statesOneStride(3,toeDownIndex);
velMag = sqrt(vVel_TD.^2 + hVel_TD.^2);
velToLegAngle_atContact = atan2(vVel_TD,hVel_TD) + (pi-beginStanceAngle);
maxPredictedImpulse = m.*velMag.*cos(velToLegAngle_atContact);
maxPredictedEnergy = (1/2).*m.*(velMag.*cos(velToLegAngle_atContact)).^2;

%Force is proportional to the change in length of the spring
Lcurrent = min(((newState(1,:).^2 + newState(2,:).^2).^.5),Lnot);
deltaL = (Lnot - Lcurrent);
% deltaL = (Lnot - (newState(1,:).^2 + newState(2,:).^2).^.5);

forces=ones(3,size(bodystate,2))*NaN;
forces(1,:) = (deltaL.*cos(theta)).*k.*isStance; % fore-aft
forces(2,:) = (deltaL.*sin(theta)).*k.*isStance; % vertical
forces(3,:) = k.*deltaL.*isStance; % total ground reaction force
energyStorage=0.5.*(forces(3,:).*deltaL);

impulseNormFactor = (m*((g*Lnot)^.5));
forceNormFactor = (m*g);
freqNormFactor = (g/Lnot)^.5;
timeNormFactor = (Lnot/g)^.5;
energyNormFactor = (m*g*Lnot);
velocityNormFactor =   (Lnot*g)^.5;
klegNormFactor =   (m*g)./Lnot;

% Calculate summary values:
summaryData.msg = 'All summary values are SI units: divide by appropriate normFactors to convert to dimensionless quantities.';
summaryData.modelParams = params;
summaryData.initialStates = statesOneStride(:,1);
summaryData.finalStates = statesOneStride(:,end);
summaryData.nTimePoints = size(statesOneStride,2);
summaryData.legInd = legInd;
summaryData.normFactors.impulse = impulseNormFactor;
summaryData.normFactors.force = forceNormFactor;
summaryData.normFactors.energy = energyNormFactor;
summaryData.normFactors.velocity = velocityNormFactor;
summaryData.normFactors.mass = m;
summaryData.normFactors.length = Lnot;
summaryData.normFactors.g = g;
summaryData.normFactors.k = klegNormFactor;
summaryData.normFactors.stepPeriod = timeNormFactor;
summaryData.normFactors.halfStepPeriod = timeNormFactor/2;
summaryData.normFactors.stridePeriod = timeNormFactor*2;
summaryData.normFactors.freq = freqNormFactor;

summaryData.totalImpulse= ((sum(forces(3,:)).*timeStep)); %total impulse
summaryData.vertImpulse= ((sum(forces(2,:)).*timeStep)); %vertical  impulse
summaryData.foreAftImpulse= ((sum(abs(forces(1,:))).*timeStep)); %fore-aft impulse (absolute value)
summaryData.impulseAngleInDegrees = rad2deg(atan2((sum(forces(2,:).*timeStep)),(sum(forces(1,:).*timeStep)))); %impulse angle
summaryData.peakGroundReactionForce = (max(forces(3,:))); %Peak ground reaction force during stance in units of bw
summaryData.peakVertForce = (max(forces(2,:))); %Peak ground reaction force during stance in units of bw
summaryData.peakForAftForce = (max(forces(1,:))); %Peak ground reaction force during stance in units of bw

summaryData.forwardVelocity = meanVelocity(1);
summaryData.vertVelocity = meanVelocity(2);
summaryData.bounceHeight = bounceHeight;
summaryData.vertDispTot = vertDispTot;
summaryData.vertDispStance = vertDispStance;
summaryData.stancePeriod = stancePeriod; %  duration of stance half-cycle
summaryData.swingPeriod = swingPeriod;%  duration of swing half-cycle
summaryData.stepPeriod = stepPeriod; %  duration of step cycle
summaryData.stridePeriod = stridePeriod; %  duration of step cycle
summaryData.stanceFreq = (0.5./stancePeriod);% frequency of stance half-cycle
summaryData.swingFreq = (0.5./swingPeriod);%  frequency of swing half-cycle
summaryData.stepFreq = (1./stepPeriod);%  frequency of step cycle
summaryData.strideFreq = (1./stridePeriod); % stride frequency
summaryData.dutyFactor = (stancePeriod./stridePeriod);
summaryData.stepLength = stepLength;
summaryData.beginStanceAngle = rad2deg(beginStanceAngle);
summaryData.endStanceAngle = rad2deg(endStanceAngle);
summaryData.halfAngle = (summaryData.beginStanceAngle - summaryData.endStanceAngle)./2;
summaryData.vVel_TD = vVel_TD;
summaryData.hVel_TD = hVel_TD;
summaryData.velToLegAngle_atContact = velToLegAngle_atContact;
summaryData.maxPredictedImpulse = maxPredictedImpulse;
summaryData.maxPredictedEnergy = maxPredictedEnergy;
summaryData.totalLegEnergy = (max(energyStorage)); % total absolute energy done by the leg during stance
% calculate cost of transport: Joules of metabolic energy used
% (assuming only positive work costs, per meter*kilogram)
summaryData.costOfTransport = (summaryData.totalLegEnergy)./(m.*stepLength); % Joules of metabolic energy used (assuming only positive work costs, per meter*kilogram)
summaryData.beginStanceIndex =(find(isStance == 1,1,'first'));

%Normalize output forces to dimensionless values
forces = forces./forceNormFactor;
energyStorage = energyStorage./energyNormFactor;
