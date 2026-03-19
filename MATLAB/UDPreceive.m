% UDPreceive.m
% Receives colour blob pixel coordinates from OpenMV RT1062 cameras over WiFi/UDP.
% Listens on a single UDP port and demultiplexes by cam_id in each packet.
% Parses 21-byte binary packets: uint8 cam_id + uint32 tick_ms + 8x uint16 uv.
%
% Latency is computed via one-shot sync: on the first packet from each camera,
% the camera tick and MATLAB toc are recorded as a reference pair.
%
% SETUP:
%   1. Save colourTrackUDP.py to each camera
%      (OpenMV IDE -> Tools -> Save Open Script to OpenMV Cam)
%   2. Set HOST_IP in each camera script to this PC's IP address
%   3. Set HOST_PORT to match LISTEN_PORT below
%   4. Ensure all cameras and this PC are on the same WiFi network
%   5. Disconnect OpenMV IDE before powering cameras (they auto-run); power
%   each camera up individually to prevent DHCP startup issues
%
%Powershell: ipconfig  (IPv4 Address under your WiFi adapter)

clear; clc; close all;

%% CONFIGURATION
numCams       = 3; % Number of cameras expected
num_features  = 4;
SENTINEL      = 65535; % uint16 NaN equivalent
PACKET_SIZE   = 21; % 1 + 4 + 16 = 21 bytes
RUN_DURATION  = 60; % seconds to run
featureNames  = {'Red', 'Green', 'Blue', 'Yellow'};

LISTEN_PORT   = 7007; % Must match HOST_PORT on cameras

%% OPEN UDP PORT
try
    u = udpport("byte", "LocalPort", LISTEN_PORT);
    fprintf("Listening on UDP port %d\n", LISTEN_PORT);
catch ME
    error("Failed to open UDP port %d: %s\n" + ...
          "Check that no other application is using this port.\n" + ...
          "If a previous MATLAB session left the port open, run:\n" + ...
          "  clear u; clear all;\n" + ...
          "or restart MATLAB.", ...
          LISTEN_PORT, ME.message);
end

% Flush any stale datagrams
flush(u);

%% STORAGE
% latestUV: numCams x num_features x 2 (u,v) — NaN if not detected
latestUV   = nan(numCams, num_features, 2);
latencyMs  = nan(numCams, 1);

packetCount = zeros(numCams, 1);
parseErrors = zeros(numCams, 1);
unknownCams = 0;

% One-shot sync references (set on first valid packet per camera)
syncCamTick  = nan(numCams, 1);
syncMatlabToc = nan(numCams, 1);

% Previous packet timing for jitter tracking
prevCamTick  = nan(numCams, 1);
prevMatlabToc = nan(numCams, 1);

% Camera ID -> internal index mapping (auto-discovered)
camIdMap = nan(numCams, 1);  % camIdMap(i) = cam_id for internal camPort i
nextSlot = 1;

% Logging for post-run analysis
maxLog = 50000;  % pre-allocate for performance
logIdx = 0;
udpLogData = struct( ...
    'camPort',       zeros(maxLog, 1), ...
    'cam_id',     zeros(maxLog, 1), ...
    'cam_tick',   zeros(maxLog, 1), ...
    'matlabToc',  zeros(maxLog, 1), ...
    'latencyMs',  zeros(maxLog, 1), ...
    'jitterMs',   nan(maxLog, 1), ...
    'uv',         nan(maxLog, num_features, 2) ...
);
%% LIVE DISPLAY
fprintf("\n--- UDP Streaming Started ---\n");
fprintf("Waiting for camera packets on port %d ...\n", LISTEN_PORT);
fprintf("Press Ctrl+C to stop\n\n");

tic;
%% MAIN LOOP
try
    while toc < RUN_DURATION
        % Read all available complete packets
        while u.NumBytesAvailable >= PACKET_SIZE
            rawBytes = read(u, PACKET_SIZE, "uint8");
            matlabToc = toc;
            cam_id = rawBytes(1);

            % Validate cam_id
            if cam_id < 1 || cam_id > 12
                parseErrors = parseErrors + 1;
                flush(u);
                continue;
            end

            % Map cam_id to internal index
            camPort = find(camIdMap == cam_id, 1);
            if isempty(camPort)
                if nextSlot > numCams
                    unknownCams = unknownCams + 1;
                    continue;  % All slots full, ignore extra cameras
                end
                camPort = nextSlot;
                camIdMap(camPort) = cam_id;
                nextSlot = nextSlot + 1;
                fprintf("  [DISCOVERED] Camera %d assigned to camPort %d\n", cam_id, camPort);
            end

            % Parse tick_ms (uint32, little-endian)
            cam_tick = double(typecast(uint8(rawBytes(2:5)), 'uint32'));

            % Parse feature uv data (8 x uint16, little-endian)
            uv_raw = typecast(uint8(rawBytes(6:21)), 'uint16');

            if numel(uv_raw) ~= 8
                parseErrors = parseErrors + 1;
                continue;
            end

            % One-shot sync
            if isnan(syncCamTick(camPort))
                syncCamTick(camPort)  = cam_tick;
                syncMatlabToc(camPort) = matlabToc;
                fprintf("  [SYNC] Camera %d synced: cam_tick=%d, matlab_toc=%.3f\n", ...
                    cam_id, cam_tick, matlabToc);
            end

            % Compute latency
            camElapsed_s    = (cam_tick - syncCamTick(camPort)) / 1000.0;
            matlabElapsed_s = matlabToc - syncMatlabToc(camPort);
            latency = (matlabElapsed_s - camElapsed_s) * 1000.0;  % ms

            % Frame-to-frame jitter
            jitterStr = '';
            jitter = NaN;
            if ~isnan(prevCamTick(camPort))
                camDelta    = (cam_tick - prevCamTick(camPort)) / 1000.0;
                matlabDelta = matlabToc - prevMatlabToc(camPort);
                jitter = (matlabDelta - camDelta) * 1000.0;
                jitterStr = sprintf('jit=%+5.1fms ', jitter);
            end
            prevCamTick(camPort)  = cam_tick;
            prevMatlabToc(camPort) = matlabToc;

            % Extract feature pixel coordinates 
            for feat = 1:num_features
                u_val = uv_raw((feat-1)*2 + 1);
                v_val = uv_raw((feat-1)*2 + 2);

                if u_val == SENTINEL || v_val == SENTINEL
                    latestUV(camPort, feat, :) = NaN;
                else
                    latestUV(camPort, feat, 1) = double(u_val);
                    latestUV(camPort, feat, 2) = double(v_val);
                end
            end

            latencyMs(camPort) = latency;
            packetCount(camPort) = packetCount(camPort) + 1;
            % Log this packet
            if logIdx < maxLog
                logIdx = logIdx + 1;
                udpLogData.camPort(logIdx)      = camPort;
                udpLogData.cam_id(logIdx)    = cam_id;
                udpLogData.cam_tick(logIdx)  = cam_tick;
                udpLogData.matlabToc(logIdx) = matlabToc;
                udpLogData.latencyMs(logIdx) = latency;
                if exist('jitter', 'var')
                    udpLogData.jitterMs(logIdx) = jitter;
                end
                udpLogData.uv(logIdx, :, :)  = latestUV(camPort, :, :);
            end
            % Display parsed packet
            fprintf("Cam %d | tick=%8d | lat=%+6.1fms | %s", cam_id, cam_tick, latency, jitterStr);
            for feat = 1:num_features
                uv_u = latestUV(camPort, feat, 1);
                uv_v = latestUV(camPort, feat, 2);
                if isnan(uv_u)
                    fprintf("%s:[-,-] ", featureNames{feat});
                else
                    fprintf("%s:[%4.0f,%4.0f] ", featureNames{feat}, uv_u, uv_v);
                end
            end
            fprintf("\n");
        end

        pause(0.0005);
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        rethrow(ME);
    end
    fprintf("\n--- Interrupted by user ---\n");
end

%% Results
% Trim log to actual size
udpLogData.camPort      = udpLogData.camPort(1:logIdx);
udpLogData.cam_id    = udpLogData.cam_id(1:logIdx);
udpLogData.cam_tick  = udpLogData.cam_tick(1:logIdx);
udpLogData.matlabToc = udpLogData.matlabToc(1:logIdx);
udpLogData.latencyMs = udpLogData.latencyMs(1:logIdx);
udpLogData.jitterMs  = udpLogData.jitterMs(1:logIdx);
udpLogData.uv        = udpLogData.uv(1:logIdx, :, :);

save('udp_latency_log.mat', 'udpLogData', 'syncCamTick', 'syncMatlabToc');
fprintf("Log saved to udp_latency_log.mat\n");

fprintf("\n--- UDP Streaming Complete ---\n");
elapsedTime = toc;

for i = 1:numCams
    if packetCount(i) > 0
        avgFPS = packetCount(i) / elapsedTime;
        fprintf("Camera %d (camPort %d): %d packets (%.1f fps), final latency=%.1fms\n", camIdMap(i), i, packetCount(i), avgFPS, latencyMs(i));
        camMask = udpLogData.camPort == i;
        camLat = udpLogData.latencyMs(camMask);
        camJit = udpLogData.jitterMs(camMask);
        camJit = camJit(~isnan(camJit));
        
        fprintf("  Latency mean: %+.1f ms, std: %.1f ms, min: %+.1f ms, max: %+.1f ms\n", ...
            mean(camLat), std(camLat), min(camLat), max(camLat));
        if ~isempty(camJit)
            fprintf("  Jitter mean: %+.1f ms, std: %.1f ms, |max|: %.1f ms\n", ...
                mean(camJit), std(camJit), max(abs(camJit)));
        end
    else
        fprintf("camPort %d: No packets received.\n", i);
    end
end
fprintf("Parse errors: %d | Unknown camera IDs dropped: %d\n", parseErrors, unknownCams);

% Close UDP port
delete(u);
clear u;
fprintf("UDP port closed.\n");
