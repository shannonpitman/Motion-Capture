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
%   5. Disconnect OpenMV IDE before powering cameras (they auto-run)
%
% FINDING YOUR PC's IP:
%   Windows:  ipconfig  (look for IPv4 Address under your WiFi adapter)

clear; clc; close all;

%% CONFIGURATION
numCams       = 2; % Number of cameras expected
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
camIdMap = nan(numCams, 1);     % camIdMap(i) = cam_id for internal slot i
nextSlot = 1;

% Logging for post-run analysis
maxLog = 50000;  % pre-allocate for performance
logIdx = 0;
logData = struct( ...
    'slot',       zeros(maxLog, 1), ...
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
            slot = find(camIdMap == cam_id, 1);
            if isempty(slot)
                if nextSlot > numCams
                    unknownCams = unknownCams + 1;
                    continue;  % All slots full, ignore extra cameras
                end
                slot = nextSlot;
                camIdMap(slot) = cam_id;
                nextSlot = nextSlot + 1;
                fprintf("  [DISCOVERED] Camera %d assigned to slot %d\n", cam_id, slot);
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
            if isnan(syncCamTick(slot))
                syncCamTick(slot)  = cam_tick;
                syncMatlabToc(slot) = matlabToc;
                fprintf("  [SYNC] Camera %d synced: cam_tick=%d, matlab_toc=%.3f\n", ...
                    cam_id, cam_tick, matlabToc);
            end

            % Compute latency
            camElapsed_s    = (cam_tick - syncCamTick(slot)) / 1000.0;
            matlabElapsed_s = matlabToc - syncMatlabToc(slot);
            latency = (matlabElapsed_s - camElapsed_s) * 1000.0;  % ms

            % Frame-to-frame jitter
            jitterStr = '';
            if ~isnan(prevCamTick(slot))
                camDelta    = (cam_tick - prevCamTick(slot)) / 1000.0;
                matlabDelta = matlabToc - prevMatlabToc(slot);
                jitter = (matlabDelta - camDelta) * 1000.0;
                jitterStr = sprintf('jit=%+5.1fms ', jitter);
            end
            prevCamTick(slot)  = cam_tick;
            prevMatlabToc(slot) = matlabToc;

            % Extract feature pixel coordinates 
            for feat = 1:num_features
                u_val = uv_raw((feat-1)*2 + 1);
                v_val = uv_raw((feat-1)*2 + 2);

                if u_val == SENTINEL || v_val == SENTINEL
                    latestUV(slot, feat, :) = NaN;
                else
                    latestUV(slot, feat, 1) = double(u_val);
                    latestUV(slot, feat, 2) = double(v_val);
                end
            end

            latencyMs(slot) = latency;
            packetCount(slot) = packetCount(slot) + 1;
            % Log this packet
            if logIdx < maxLog
                logIdx = logIdx + 1;
                logData.slot(logIdx)      = slot;
                logData.cam_id(logIdx)    = cam_id;
                logData.cam_tick(logIdx)  = cam_tick;
                logData.matlabToc(logIdx) = matlabToc;
                logData.latencyMs(logIdx) = latency;
                if exist('jitter', 'var')
                    logData.jitterMs(logIdx) = jitter;
                end
                logData.uv(logIdx, :, :)  = latestUV(slot, :, :);
            end
            % Display parsed packet
            fprintf("Cam %d | tick=%8d | lat=%+6.1fms | %s", cam_id, cam_tick, latency, jitterStr);
            for feat = 1:num_features
                uv_u = latestUV(slot, feat, 1);
                uv_v = latestUV(slot, feat, 2);
                if isnan(uv_u)
                    fprintf("%s:[----,----] ", featureNames{feat});
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
logData.slot      = logData.slot(1:logIdx);
logData.cam_id    = logData.cam_id(1:logIdx);
logData.cam_tick  = logData.cam_tick(1:logIdx);
logData.matlabToc = logData.matlabToc(1:logIdx);
logData.latencyMs = logData.latencyMs(1:logIdx);
logData.jitterMs  = logData.jitterMs(1:logIdx);
logData.uv        = logData.uv(1:logIdx, :, :);

save('udp_latency_log.mat', 'logData', 'syncCamTick', 'syncMatlabToc');
fprintf("Log saved to udp_latency_log.mat\n");

fprintf("\n--- UDP Streaming Complete ---\n");
elapsedTime = toc;

for i = 1:numCams
    if packetCount(i) > 0
        avgFPS = packetCount(i) / elapsedTime;
        fprintf("Camera %d (slot %d): %d packets (%.1f fps), final latency=%.1fms\n", camIdMap(i), i, packetCount(i), avgFPS, latencyMs(i));
        % Stats on corrected latency for this camera
        camMask = logData.slot == i;
        camCorrLat = logData.latencyMs(camMask);
        camJitter = logData.jitterMs(camMask);
        camJitter = camJitter(~isnan(camJitter));
    else
        fprintf("Slot %d: No packets received.\n", i);
    end
end
fprintf("Parse errors: %d | Unknown camera IDs dropped: %d\n", parseErrors, unknownCams);

% Close UDP port
delete(u);
clear u;
fprintf("UDP port closed.\n");
