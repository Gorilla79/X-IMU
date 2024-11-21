clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% 데이터셋 선택
filePath = 'Datasets/stairsAndCorridor';
startTime = 6;
stopTime = 26;

% 데이터 가져오기
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

% 데이터 프레이밍
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

% 정지 구간 감지
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);
acc_magFilt = abs(acc_magFilt);
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);
stationary = acc_magFilt < 0.05;

% 자세 계산
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

% 변환 가속도 계산
acc = quaternRotate([accX accY accZ], quaternConj(quat));
acc = acc * 9.81;
acc(:,3) = acc(:,3) - 9.81;  % 중력 제거

% 속도 적분
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % 정지 시 속도 0
    end
end

% 드리프트 제거
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end
vel = vel - velDrift;  % 드리프트 제거한 속도

% 위치 적분
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;
end

% 방향 및 높이 변화 감지
direction = atan2(vel(:,2), vel(:,1)); % 방향 계산
directionChangeIndices = find(diff(direction) > deg2rad(10)); % 방향 변화 인덱스
heightChangeIndices = find(diff(pos(:,3)) > 0.1 | diff(pos(:,3)) < -0.1); % 높이 변화 인덱스

% 변화 정보 출력
for i = 1:length(directionChangeIndices)
    index = directionChangeIndices(i);
    disp(['방향 변화 포인트: 시간 ', num2str(time(index)), '초, 거리 ', num2str(norm(pos(index, :))), '미터']);
end
for i = 1:length(heightChangeIndices)
    index = heightChangeIndices(i);
    heightChange = pos(index+1, 3) - pos(index, 3);
    if heightChange > 0
        disp(['상승: 시간 ', num2str(time(index)), '초, 높이 변화 ', num2str(heightChange), '미터']);
    else
        disp(['하강: 시간 ', num2str(time(index)), '초, 높이 변화 ', num2str(heightChange), '미터']);
    end
end
clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% 데이터셋 선택
filePath = 'Datasets/imu_test.csv';
startTime = 6;
stopTime = 26;

% 데이터 가져오기
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

% 데이터 프레이밍
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

% 정지 구간 감지
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);
acc_magFilt = abs(acc_magFilt);
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);
stationary = acc_magFilt < 0.05;

% 자세 계산
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

% 변환 가속도 계산
acc = quaternRotate([accX accY accZ], quaternConj(quat));
acc = acc * 9.81;
acc(:,3) = acc(:,3) - 9.81;  % 중력 제거

% 속도 적분
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % 정지 시 속도 0
    end
end

% 드리프트 제거
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end
vel = vel - velDrift;  % 드리프트 제거한 속도

% 위치 적분
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;
end

% 방향 및 높이 변화 감지
direction = atan2(vel(:,2), vel(:,1)); % 방향 계산
directionChangeIndices = find(diff(direction) > deg2rad(10)); % 방향 변화 인덱스
heightChangeIndices = find(diff(pos(:,3)) > 0.1 | diff(pos(:,3)) < -0.1); % 높이 변화 인덱스

% 변화 정보 출력
for i = 1:length(directionChangeIndices)
    index = directionChangeIndices(i);
    disp(['방향 변화 포인트: 시간 ', num2str(time(index)), '초, 거리 ', num2str(norm(pos(index, :))), '미터']);
end
for i = 1:length(heightChangeIndices)
    index = heightChangeIndices(i);
    heightChange = pos(index+1, 3) - pos(index, 3);
    if heightChange > 0
        disp(['상승: 시간 ', num2str(time(index)), '초, 높이 변화 ', num2str(heightChange), '미터']);
    else
        disp(['하강: 시간 ', num2str(time(index)), '초, 높이 변화 ', num2str(heightChange), '미터']);
    end
end

% -------------------------------------------------------------------------
% 3D 궤적 플로팅
posPlot = pos;
quatPlot = quat;
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; repmat(quatPlot(end, :), extraTime*(1/samplePeriod), 1)];

% 애니메이션 생성
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));