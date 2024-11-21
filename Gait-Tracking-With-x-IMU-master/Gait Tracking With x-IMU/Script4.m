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
direction = atan2d(vel(:,2), vel(:,1)); % 방향 계산
currentDirection = direction(1);
totalDistance = 0;
segmentStart = 1;
directionChanges = [];

for t = 2:length(direction)
    totalDistance = totalDistance + norm(pos(t,:) - pos(t-1,:));
    directionChange = abs(direction(t) - currentDirection);
    
    if directionChange >= 90 || t == length(direction)
        segmentEnd = t;
        segmentDistance = norm(pos(segmentEnd,:) - pos(segmentStart,:));
        if directionChange >= 90
            turnType = '우회전';
        elseif directionChange <= -90
            turnType = '좌회전';
        else
            turnType = '직진';
        end
        fprintf('방향: %.2f도 - 거리: %.2f 미터 - 회전: %s\n', currentDirection, segmentDistance, turnType);
        currentDirection = direction(t);
        segmentStart = t;
        totalDistance = 0;
    end
end

% 3D 트랙과 방향 변화 시각화
figure('Position', [100, 100, 800, 600]);
plot3(pos(:,1), pos(:,2), pos(:,3), 'k.-'); % 3D 경로 플로팅
hold on;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D 이동 경로');

% 방향 변화 시각화
for i = 1:size(directionChanges, 1)
    % 방향 변화가 있는 위치를 표시
    dc_index = find(direction == directionChanges(i,1), 1);
    plot3(pos(dc_index,1), pos(dc_index,2), pos(dc_index,3), 'ro', 'MarkerSize', 8, 'LineWidth', 3);
end

% 결과 그래프 저장 (옵션)
% saveas(gcf, 'path_plot.png');

% 애니메이션 생성 (옵션)
% animation_fig = figure;
% for t = 1:length(time)
%     plot3(pos(1:t,1), pos(1:t,2), pos(1:t,3), 'k.-');
%     hold on;
%     plot3(pos(t,1), pos(t,2), pos(t,3), 'ro', 'MarkerSize', 8, 'LineWidth', 3);
%     hold off;
%     drawnow;
%     % 애니메이션 프레임을 저장할 수 있습니다.
% end

% 애니메이션을 비디오 파일로 저장 (옵션)
% v = VideoWriter('3DPathAnimation.avi');
% open(v);
% for t = 1:length(time)
%     frame = getframe(animation_fig);
%     writeVideo(v, frame);
% end
% close(v);