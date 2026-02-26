% receiveUSB.m
% Receives colour blob pixel coordinates from OpenMV RT1062 cameras over USB.
% Parses 26-byte uint16 binary packets with RTC timestamps.
% Computes measurement latency by comparing camera RTC time to MATLAB clock.
%
% Run this BEFORE starting the EKF to validate the data pipeline.
%
% IMPORTANT: 
%   1. Save the tracking script to the camera first
%      (OpenMV IDE -> Tools -> Save Open Script to OpenMV Cam)
%   2. Disconnect the OpenMV IDE before running this script
%   3. The camera will auto-run the saved script on power-up

clear; clc; close all;

%% Configuration
numCams = 2;
num_features = 4;
SENTINEL = 65535;               % uint16 NaN equivalent
PACKET_SIZE = 26;               % 13 x uint16 = 26 bytes
RUN_DURATION = 60;              % seconds to run
featureNames = {'Red', 'Green', 'Blue', 'Yellow'};

% Serial port names — update these for your system
% Windows: check Device Manager -> Ports (COM & LPT)
%   Look for "USB Serial Device" after plugging in each camera
% Mac: ls /dev/tty.usbmodem*
% Linux: ls /dev/ttyACM*
portNames = ["COM3", "COM4"];   % <-- UPDATE THESE
BAUD_RATE = 115200;

%% Open Serial Ports
serialPorts = cell(1, numCams);
for i = 1:numCams
    try
        serialPorts{i} = serialport(portNames(i), BAUD_RATE);
        configureTerminator(serialPorts{i}, "LF");
        flush(serialPorts{i});
        fprintf("Camera %d connected on %s\n", i, portNames(i));
    catch ME
        error("Failed to open %s: %s\nCheck:\n" + ...
            "  1. OpenMV IDE is disconnected\n" + ...
            "  2. Camera is plugged in (script saved to cam with Tools -> Save Open Script)\n" + ...
            "  3. Port name is correct (check Device Manager)", ...
            portNames(i), ME.message);
    end
end

% Flush stale data after connection
pause(0.5);
for i = 1:numCams
    flush(serialPorts{i});
end

%% Storage
% latestUV: numCams x num_features x 2 (u,v) — NaN if not detected
latestUV = nan(numCams, num_features, 2);
camTimestamp = NaT(numCams, 1);     % Camera RTC timestamp as datetime
timeReceived = NaT(numCams, 1);     % MATLAB receive time as datetime
latencyMs = nan(numCams, 1);        % Estimated latency in milliseconds

packetCount = zeros(numCams, 1);
parseErrors = zeros(numCams, 1);

%% Live Display Setup
fprintf("\n--- Streaming Started ---\n");
fprintf("Press Ctrl+C to stop\n\n");

tic;
today = datetime('today');  % Used to build full datetime from RTC h:m:s

%% Main Receive Loop
try
    while toc < RUN_DURATION
        for camPort = 1:numCams
            sp = serialPorts{camPort};

            % Check if a full packet is available
            if sp.NumBytesAvailable >= PACKET_SIZE
                % Read exactly one packet
                rawBytes = read(sp, PACKET_SIZE, "uint8");
                matlabNow = datetime('now');  % Record arrival time immediately

                % Parse: 13 x uint16, little-endian
                data = typecast(uint8(rawBytes), 'uint16');

                if numel(data) ~= 13
                    parseErrors(camPort) = parseErrors(camPort) + 1;
                    continue;
                end

                % Validate cam_id (basic sanity check for byte alignment)
                cam_id = data(1);
                if cam_id < 1 || cam_id > 12
                    % Likely misaligned — flush and resync
                    parseErrors(camPort) = parseErrors(camPort) + 1;
                    flush(sp);
                    continue;
                end

                % Extract RTC timestamp
                cam_hour = double(data(2));
                cam_min = double(data(3));
                cam_sec = double(data(4));
                cam_ms = double(data(5));

                % Validate time fields
                if cam_hour > 23 || cam_min > 59 || cam_sec > 59 || cam_ms > 999
                    parseErrors(camPort) = parseErrors(camPort) + 1;
                    flush(sp);
                    continue;
                end

                % Build camera timestamp as datetime
                camTime = today + hours(cam_hour) + minutes(cam_min) + ...
                    seconds(cam_sec) + milliseconds(cam_ms);

                % Compute latency (camera capture -> MATLAB receive)
                latency = milliseconds(matlabNow - camTime);

                % Extract feature pixel coordinates
                uv_data = data(6:13);  % [u_r, v_r, u_g, v_g, u_b, v_b, u_y, v_y]

                for feat = 1:num_features
                    u_val = uv_data((feat-1)*2 + 1);
                    v_val = uv_data((feat-1)*2 + 2);

                    if u_val == SENTINEL || v_val == SENTINEL
                        latestUV(camPort, feat, :) = NaN;
                    else
                        latestUV(camPort, feat, 1) = double(u_val);
                        latestUV(camPort, feat, 2) = double(v_val);
                    end
                end

                % Store timing data
                camTimestamp(camPort) = camTime;
                timeReceived(camPort) = matlabNow;
                latencyMs(camPort) = latency;
                packetCount(camPort) = packetCount(camPort) + 1;

                % Display parsed packet
                fprintf("Cam %d | %02d:%02d:%02d.%03d | lat=%5.0fms | ", ...
                    cam_id, cam_hour, cam_min, cam_sec, cam_ms, latency);
                for feat = 1:num_features
                    u = latestUV(camPort, feat, 1);
                    v = latestUV(camPort, feat, 2);
                    if isnan(u)
                        fprintf("%s:[----,----] ", featureNames{feat});
                    else
                        fprintf("%s:[%4.0f,%4.0f] ", featureNames{feat}, u, v);
                    end
                end
                fprintf("\n");
            end
        end

        % Small pause to avoid busy-waiting
        pause(0.0005);
    end
catch ME
    if ~strcmp(ME.identifier, 'MATLAB:interrupt')
        rethrow(ME);
    end
    fprintf("\n--- Interrupted by user ---\n");
end

%% Summary & Cleanup
fprintf("\n--- Streaming Complete ---\n");
elapsedTime = toc;
for i = 1:numCams
    if packetCount(i) > 0
        avgFPS = packetCount(i) / elapsedTime;
        fprintf("Camera %d: %d packets (%.1f fps), %d parse errors\n", ...
            i, packetCount(i), avgFPS, parseErrors(i));
    else
        fprintf("Camera %d: No packets received. Check connection.\n", i);
    end
end

fprintf("\nNOTE on latency values:\n");
fprintf("  Latency accuracy depends on the camera RTC being set correctly.\n");
fprintf("  If latency is large/negative, re-sync the RTC init time in the\n");
fprintf("  OpenMV script to match your PC clock, then re-save to camera.\n");

% Close serial ports
for i = 1:numCams
    delete(serialPorts{i});
end
clear serialPorts;
fprintf("Serial ports closed.\n");