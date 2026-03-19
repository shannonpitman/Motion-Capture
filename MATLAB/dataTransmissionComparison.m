usbFile = load('usb_latency_log.mat');
udpFile = load('udp_latency_log.mat');

usbLog = usbFile.USBLogData;
udpLog = udpFile.udpLogData;

extractCam = @(logStruct, camPort) struct( ...
    'latencyMs', logStruct.latencyMs(logStruct.camPort == camPort), ...
    'jitterMs',  logStruct.jitterMs(logStruct.camPort == camPort), ...
    'camTick',   logStruct.cam_tick(logStruct.camPort == camPort), ...
    'matlabToc', logStruct.matlabToc(logStruct.camPort == camPort) ...
);

usbLatency = usbLog.latencyMs;
usbJitter = usbLog.jitterMs(~isnan(usbLog.jitterMs));

udpLatency = udpLog.latencyMs;
udpJitter = udpLog.jitterMs(~isnan(udpLog.jitterMs));

usbDuration = max(usbLog.matlabToc) - min(usbLog.matlabToc);
udbFPS = numel(usbLatency)/ usbDuration; %effective fps

udpDuration = max(udpLog.matlabToc) - min(udpLog.matlabToc);
udpFPS = numel(udpLatency) / udpDuration; 

%true fps
usbCamDt = diff(sort(usbLog.cam_tick));  % ms
udpCamDt = diff(sort(udpLog.cam_tick));
usbCamDt = usbCamDt(usbCamDt > 0 & usbCamDt < 200); % filter outliers
udpCamDt = udpCamDt(udpCamDt > 0 & udpCamDt < 200);
usbCamFPS = 1000 / median(usbCamDt);
udpCamFPS = 1000 / median(udpCamDt);

figure("Name", "Latency Comparison")
subplot(1,2,1)
histogram(usbLatency, 50, 'FaceColor', [1 0 0], 'FaceAlpha', 0.7);
hold on;
xline(median(usbLatency), 'k--', sprintf('median=%.1f ms', median(usbLatency)))
xlabel('Latency (ms)');
ylabel('Count');
subplot(1,2,2)
histogram(udpLatency, 50, 'FaceColor', [0 0 1], 'FaceAlpha', 0.7);
hold on;
xline(median(udpLatency), 'b--', sprintf('median=%.1f ms', median(udpLatency)));
xlabel('Latency (ms)');
ylabel('Count');
title('UDP Latency Distribution');
grid on;

figure('Name', 'Jitter Comparison');
subplot(1,2,1);
histogram(usbJitter, 50, 'FaceColor', [0.2 0.4 0.8], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
hold on;
xline(0, 'k-', 'LineWidth', 0.5);
xline(std(usbJitter), 'r--', sprintf('\\sigma=%.2f ms', std(usbJitter)), 'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
xline(-std(usbJitter), 'r--', 'LineWidth', 1.5);
xlabel('Jitter (ms)'); ylabel('Count');
title('USB Jitter Distribution');
grid on;
 
subplot(1,2,2);
histogram(udpJitter, 50, 'FaceColor', [0.8 0.4 0.2], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
hold on;
xline(0, 'k-', 'LineWidth', 0.5);
xline(std(udpJitter), 'r--', sprintf('\\sigma=%.2f ms', std(udpJitter)), 'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
xline(-std(udpJitter), 'r--', 'LineWidth', 1.5);
xlabel('Jitter (ms)'); ylabel('Count');
title('UDP Jitter Distribution');
grid on;
