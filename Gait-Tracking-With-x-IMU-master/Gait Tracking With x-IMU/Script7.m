clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% -------------------------------------------------------------------------
% Select dataset (comment in/out)
filePath = 'Datasets/stairsAndCorridor';
startTime = 6;
stopTime = 26;

% -------------------------------------------------------------------------
% Import data
samplePeriod = 1/256;
xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
time = xIMUdata.CalInertialAndMagneticData.Time;
gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
clear('xIMUdata');

% -------------------------------------------------------------------------
% Manually frame data
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

% -------------------------------------------------------------------------
% Detect stationary periods
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);
acc_magFilt = abs(acc_magFilt);
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);
stationary = acc_magFilt < 0.05;

% -------------------------------------------------------------------------
% Compute orientation
quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);
initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end
for t = 1:length(time)
    if(stationary(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end

% -------------------------------------------------------------------------
% Compute translational accelerations
acc = quaternRotate([accX accY accZ], quaternConj(quat));
acc = acc * 9.81;
acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     
    end
end

% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end
vel = vel - velDrift;

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;   
end

% -------------------------------------------------------------------------
% Improved Navigation Instructions with More Tolerant Turn Detection
thresholdTurn = 100; % 회전 감지 임계값 높임 (도)
minStraightDuration = 9; % 최소 직진 지속 시간 늘림 (초)
minStraightDistance = 1; % 최소 직진 거리 늘림 (미터)
currentDirection = atan2d(vel(1,2), vel(1,1));
segmentDistance = 0;
lastTurnTime = time(1); % 마지막 회전 시간 초기화
pathInstructions = {};

for t = 2:length(time)
    segmentDistance = segmentDistance + norm(vel(t,:) * samplePeriod);
    elapsedTime = time(t) - lastTurnTime;
    directionChange = mod(atan2d(vel(t,2), vel(t,1)) - currentDirection + 180, 360) - 180;

    if abs(directionChange) > thresholdTurn && elapsedTime > minStraightDuration && segmentDistance > minStraightDistance
        if segmentDistance > 0.1 
            pathInstructions{end+1} = sprintf('직진 %.2f 미터', segmentDistance);
            if directionChange > 0
                pathInstructions{end+1} = '우회전';
            else
                pathInstructions{end+1} = '좌회전';
            end
        end
        segmentDistance = 0;
        lastTurnTime = time(t);
        currentDirection = atan2d(vel(t,2), vel(t,1));
    end
end

if segmentDistance > 0.1
    pathInstructions{end+1} = sprintf('직진 %.2f 미터', segmentDistance);
end

disp('네비게이션 지침:');
for i = 1:length(pathInstructions)
    disp(pathInstructions{i});
end

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
