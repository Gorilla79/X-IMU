clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

% -------------------------------------------------------------------------
% 데이터셋 선택
filePath = 'Datasets/stairsAndCorridor';
startTime = 6;
stopTime = 26;

% -------------------------------------------------------------------------
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

% -------------------------------------------------------------------------
% 데이터 프레이밍
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

% -------------------------------------------------------------------------
% 정지 구간 감지

% 가속도계 크기 계산
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% 고역 필터 적용
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);

% 절대값 계산
acc_magFilt = abs(acc_magFilt);

% 저역 필터 적용
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% 임계값 감지
stationary = acc_magFilt < 0.05;

% -------------------------------------------------------------------------
% 자세 계산
quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% 초기 수렴
initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

% 전체 데이터에 대해서
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

% -------------------------------------------------------------------------
% 총 이동 거리 계산 및 한글로 출력
totalDistanceX = pos(end, 1) - pos(1, 1);
totalDistanceY = pos(end, 2) - pos(1, 2);
totalDistanceZ = pos(end, 3) - pos(1, 3);
disp(['X축 총 이동 거리: ', num2str(totalDistanceX), ' m']);
disp(['Y축 총 이동 거리: ', num2str(totalDistanceY), ' m']);
disp(['Z축 총 이동 거리: ', num2str(totalDistanceZ), ' m']);