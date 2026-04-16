clear; clc; close all;

%% CONFIGURATION
TEST_LABEL    = 'red_static_daylight';   % describe this run
RUN_DURATION  = 120;                     % seconds
LISTEN_PORT   = 5005;                    % must match HOST_PORT in .py script
featureNames  = {'Red', 'Green', 'Blue', 'Yellow'};
numFeatures   = numel(featureNames);

%% OPEN UDP PORT
try %[output:group:3faf078e]
    u = udpport("byte", "LocalPort", LISTEN_PORT);
    configureTerminator(u, "LF");
    flush(u);
    fprintf("Listening on UDP port %d\n", LISTEN_PORT); %[output:7c200484]
catch ME
    error("Failed to open UDP port %d: %s\n" + ...
          "If a previous session left it open, run: clear u; clear all;", ...
          LISTEN_PORT, ME.message);
end %[output:group:3faf078e]

maxLog = 200000;   % ~55 min at 60 Hz; generous
log = struct( ...
    'cam_id',    zeros(maxLog, 1), ...
    'cam_tick',  zeros(maxLog, 1), ...
    'matlabToc', zeros(maxLog, 1), ...
    'count',     zeros(maxLog, numFeatures), ...
    'u',         nan(maxLog, numFeatures), ...
    'v',         nan(maxLog, numFeatures), ...
    'pixels',    zeros(maxLog, numFeatures), ...
    'density',   zeros(maxLog, numFeatures), ...
    'bboxW',     zeros(maxLog, numFeatures), ...
    'bboxH',     zeros(maxLog, numFeatures));

idx = 0;
headerInfo = '';

fprintf("Waiting for packets... (Ctrl+C to stop early)\n"); %[output:1f53e3c7]
tStart = tic;


while toc(tStart) < RUN_DURATION %[output:group:5bfb7b8f]
    if u.NumBytesAvailable == 0
        pause(0.001);
        continue;
    end

    try
        line = readline(u);   % string, terminator stripped
    catch
        continue;
    end

    if strlength(line) == 0
        continue;
    end

    matlabToc = toc(tStart);

    % Handle one-off header packet
    if startsWith(line, "HEADER")
        headerInfo = char(line);
        fprintf("Received header: %s\n", headerInfo);
        continue;
    end

    % Parse CSV data line
    parts = split(line, ",");
    % Expected: 2 + 7*numFeatures = 30 fields
    if numel(parts) ~= 2 + 7*numFeatures
        fprintf("Skipped malformed packet (%d fields): %s\n", ...
                numel(parts), line);
        continue;
    end

    cam_id   = str2double(parts(1));
    cam_tick = str2double(parts(2));

    idx = idx + 1;
    if idx > maxLog
        warning("Log full; stopping.");
        break;
    end

    log.cam_id(idx)    = cam_id;
    log.cam_tick(idx)  = cam_tick;
    log.matlabToc(idx) = matlabToc;

    for feat = 1:numFeatures
        base = 2 + (feat-1)*7;   % offset into parts
        log.count(idx, feat)   = str2double(parts(base + 1));
        log.u(idx, feat)       = str2double(parts(base + 2));  % NaN if "NaN"
        log.v(idx, feat)       = str2double(parts(base + 3));
        log.pixels(idx, feat)  = str2double(parts(base + 4));
        log.density(idx, feat) = str2double(parts(base + 5));
        log.bboxW(idx, feat)   = str2double(parts(base + 6));
        log.bboxH(idx, feat)   = str2double(parts(base + 7));
    end

    % Live summary line, shown every 30 packets to avoid flooding
    if mod(idx, 30) == 0
        summary = sprintf("t=%5.1fs | cam=%d | ", matlabToc, cam_id);
        for feat = 1:numFeatures
            if log.count(idx, feat) > 0
                summary = summary + sprintf("%s:n=%d,pix=%d ", ...
                    featureNames{feat}, log.count(idx, feat), ...
                    log.pixels(idx, feat));
            else
                summary = summary + sprintf("%s:- ", featureNames{feat});
            end
        end
        fprintf("%s\n", summary); %[output:7ad768dc]
    end
end %[output:group:5bfb7b8f]

fieldsToTrim = fieldnames(log);
for k = 1:numel(fieldsToTrim)
    log.(fieldsToTrim{k}) = log.(fieldsToTrim{k})(1:idx, :);
end

meta = struct( ...
    'test_label', TEST_LABEL, ...
    'run_duration', RUN_DURATION, ...
    'listen_port', LISTEN_PORT, ...
    'num_packets', idx, ...
    'header_info', headerInfo, ...
    'feature_names', {featureNames}, ...
    'timestamp', datestr(now, 'yyyymmdd_HHMMSS'));

fname = sprintf("depth_test_%s_%s.mat", TEST_LABEL, meta.timestamp);
save(fname, 'log', 'meta');

fprintf("\nDone. %d packets logged over %.1f s -> %s\n", ...
        idx, toc(tStart), fname);

clear u;

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":55.9}
%---
%[output:7c200484]
%   data: {"dataType":"text","outputData":{"text":"Listening on UDP port 5005\n","truncated":false}}
%---
%[output:1f53e3c7]
%   data: {"dataType":"text","outputData":{"text":"Waiting for packets... (Ctrl+C to stop early)\n","truncated":false}}
%---
%[output:7ad768dc]
%   data: {"dataType":"text","outputData":{"text":"t=  0.3s | cam=1 | Red:- Green:n=4,pix=3495 Blue:- Yellow:n=3,pix=3495 \nt=  0.3s | cam=1 | Red:- Green:n=6,pix=3140 Blue:- Yellow:n=1,pix=3140 \nt=  0.4s | cam=1 | Red:- Green:n=3,pix=3553 Blue:- Yellow:n=1,pix=3553 \nt=  1.1s | cam=1 | Red:- Green:n=2,pix=1320 Blue:- Yellow:n=2,pix=1320 \nt=  2.1s | cam=1 | Red:- Green:n=6,pix=3247 Blue:- Yellow:n=1,pix=3247 \nt=  3.2s | cam=1 | Red:- Green:n=4,pix=3456 Blue:- Yellow:n=1,pix=3456 \nt=  4.2s | cam=1 | Red:- Green:n=5,pix=2201 Blue:- Yellow:n=1,pix=175 \nt=  5.3s | cam=1 | Red:- Green:n=6,pix=580 Blue:- Yellow:n=1,pix=255 \nt=  6.4s | cam=1 | Red:- Green:n=5,pix=360 Blue:- Yellow:n=3,pix=390 \nt=  9.2s | cam=1 | Red:- Green:n=4,pix=3856 Blue:- Yellow:n=1,pix=3856 \nt= 11.3s | cam=1 | Red:- Green:n=9,pix=1279 Blue:- Yellow:n=3,pix=1279 \nt= 11.3s | cam=1 | Red:- Green:n=7,pix=3023 Blue:- Yellow:n=1,pix=3023 \nt= 11.3s | cam=1 | Red:- Green:n=7,pix=2983 Blue:- Yellow:n=1,pix=2983 \nt= 13.0s | cam=1 | Red:- Green:n=7,pix=2899 Blue:- Yellow:n=1,pix=2899 \nt= 13.7s | cam=1 | Red:- Green:n=3,pix=4677 Blue:- Yellow:n=1,pix=4677 \nt= 14.2s | cam=1 | Red:- Green:n=6,pix=2283 Blue:- Yellow:n=4,pix=1197 \nt= 15.3s | cam=1 | Red:- Green:n=7,pix=1899 Blue:- Yellow:n=3,pix=1192 \nt= 16.5s | cam=1 | Red:- Green:n=9,pix=1740 Blue:- Yellow:n=4,pix=1072 \nt= 19.0s | cam=1 | Red:- Green:n=4,pix=5085 Blue:- Yellow:n=1,pix=5085 \nt= 20.7s | cam=1 | Red:- Green:n=4,pix=4522 Blue:- Yellow:n=1,pix=4522 \nt= 20.7s | cam=1 | Red:- Green:n=6,pix=2891 Blue:- Yellow:n=1,pix=2891 \nt= 20.8s | cam=1 | Red:- Green:n=4,pix=3070 Blue:- Yellow:n=2,pix=3070 \nt= 22.0s | cam=1 | Red:- Green:n=6,pix=2684 Blue:- Yellow:n=2,pix=2684 \nt= 22.9s | cam=1 | Red:- Green:n=4,pix=3243 Blue:- Yellow:n=4,pix=3243 \nt= 24.0s | cam=1 | Red:- Green:n=5,pix=3151 Blue:- Yellow:n=1,pix=3151 \nt= 25.1s | cam=1 | Red:- Green:n=4,pix=4535 Blue:- Yellow:n=1,pix=4535 \nt= 26.1s | cam=1 | Red:- Green:n=4,pix=4567 Blue:- Yellow:n=1,pix=4567 \nt= 27.2s | cam=1 | Red:- Green:n=3,pix=4396 Blue:- Yellow:n=1,pix=4396 \nt= 28.5s | cam=1 | Red:- Green:n=3,pix=4042 Blue:- Yellow:n=1,pix=4042 \nt= 29.4s | cam=1 | Red:- Green:n=4,pix=4489 Blue:- Yellow:n=2,pix=4489 \nt= 31.0s | cam=1 | Red:- Green:n=5,pix=4429 Blue:- Yellow:n=1,pix=4429 \nt= 31.5s | cam=1 | Red:- Green:n=4,pix=4414 Blue:- Yellow:n=2,pix=4414 \nt= 32.5s | cam=1 | Red:- Green:n=3,pix=4056 Blue:- Yellow:n=1,pix=4056 \nt= 33.6s | cam=1 | Red:- Green:n=7,pix=1395 Blue:- Yellow:n=3,pix=1395 \nt= 34.7s | cam=1 | Red:- Green:n=3,pix=4358 Blue:- Yellow:n=2,pix=4358 \nt= 35.8s | cam=1 | Red:- Green:n=3,pix=3919 Blue:- Yellow:n=1,pix=3919 \nt= 36.9s | cam=1 | Red:- Green:n=5,pix=3309 Blue:- Yellow:n=1,pix=3309 \nt= 38.9s | cam=1 | Red:- Green:n=8,pix=1602 Blue:- Yellow:n=3,pix=1294 \nt= 41.8s | cam=1 | Red:- Green:n=3,pix=3144 Blue:- Yellow:n=1,pix=3144 \nt= 42.5s | cam=1 | Red:- Green:n=7,pix=2779 Blue:- Yellow:n=2,pix=2779 \nt= 42.5s | cam=1 | Red:- Green:n=3,pix=2365 Blue:- Yellow:n=1,pix=2365 \nt= 42.5s | cam=1 | Red:- Green:n=5,pix=1593 Blue:- Yellow:n=2,pix=1593 \nt= 43.6s | cam=1 | Red:- Green:n=5,pix=554 Blue:- Yellow:n=1,pix=248 \nt= 44.7s | cam=1 | Red:n=1,pix=155 Green:n=5,pix=3165 Blue:- Yellow:n=2,pix=3165 \nt= 45.8s | cam=1 | Red:- Green:n=4,pix=404 Blue:- Yellow:n=5,pix=1632 \nt= 46.9s | cam=1 | Red:- Green:n=5,pix=2712 Blue:- Yellow:n=1,pix=2712 \nt= 48.0s | cam=1 | Red:- Green:n=8,pix=2100 Blue:- Yellow:n=1,pix=2100 \nt= 49.2s | cam=1 | Red:- Green:n=6,pix=2844 Blue:- Yellow:n=1,pix=2844 \nt= 51.1s | cam=1 | Red:- Green:n=5,pix=2632 Blue:- Yellow:n=1,pix=2632 \nt= 51.4s | cam=1 | Red:- Green:n=3,pix=5038 Blue:- Yellow:n=1,pix=5038 \nt= 52.5s | cam=1 | Red:- Green:n=7,pix=2999 Blue:- Yellow:n=1,pix=2999 \nt= 53.7s | cam=1 | Red:- Green:n=4,pix=5293 Blue:- Yellow:n=1,pix=5293 \nt= 54.6s | cam=1 | Red:- Green:n=8,pix=1230 Blue:- Yellow:n=3,pix=1144 \nt= 55.7s | cam=1 | Red:- Green:n=3,pix=4881 Blue:- Yellow:n=1,pix=4881 \nt= 56.8s | cam=1 | Red:- Green:n=5,pix=3359 Blue:- Yellow:n=1,pix=3359 \nt= 58.1s | cam=1 | Red:- Green:n=4,pix=4612 Blue:- Yellow:n=1,pix=4612 \nt= 58.9s | cam=1 | Red:- Green:n=3,pix=4997 Blue:- Yellow:n=1,pix=4997 \nt= 60.8s | cam=1 | Red:- Green:n=6,pix=1326 Blue:- Yellow:n=3,pix=1326 \nt= 62.3s | cam=1 | Red:- Green:n=8,pix=1371 Blue:- Yellow:n=4,pix=1371 \nt= 62.3s | cam=1 | Red:- Green:n=5,pix=3112 Blue:- Yellow:n=3,pix=3112 \nt= 63.2s | cam=1 | Red:- Green:n=3,pix=4602 Blue:- Yellow:n=1,pix=4602 \nt= 64.2s | cam=1 | Red:- Green:n=3,pix=4573 Blue:- Yellow:n=1,pix=4573 \nt= 65.3s | cam=1 | Red:- Green:n=7,pix=2928 Blue:- Yellow:n=1,pix=2928 \nt= 66.4s | cam=1 | Red:- Green:n=8,pix=1473 Blue:- Yellow:n=3,pix=1473 \nt= 67.5s | cam=1 | Red:- Green:n=3,pix=5350 Blue:- Yellow:n=1,pix=5350 \nt= 68.7s | cam=1 | Red:- Green:n=5,pix=2733 Blue:- Yellow:n=1,pix=2733 \nt= 69.9s | cam=1 | Red:- Green:n=4,pix=2858 Blue:- Yellow:n=1,pix=2858 \nt= 70.8s | cam=1 | Red:- Green:n=4,pix=1610 Blue:- Yellow:n=3,pix=1610 \nt= 72.0s | cam=1 | Red:- Green:n=4,pix=3980 Blue:- Yellow:n=2,pix=3980 \nt= 73.1s | cam=1 | Red:- Green:n=6,pix=2441 Blue:- Yellow:n=3,pix=2441 \nt= 74.2s | cam=1 | Red:- Green:n=7,pix=2424 Blue:- Yellow:n=1,pix=2424 \nt= 75.4s | cam=1 | Red:- Green:n=3,pix=4958 Blue:- Yellow:n=1,pix=4958 \nt= 76.5s | cam=1 | Red:- Green:n=7,pix=2093 Blue:- Yellow:n=2,pix=1348 \nt= 77.6s | cam=1 | Red:- Green:n=6,pix=2877 Blue:- Yellow:n=1,pix=2877 \nt= 78.7s | cam=1 | Red:- Green:n=4,pix=5635 Blue:- Yellow:n=1,pix=5635 \nt= 80.9s | cam=1 | Red:- Green:n=6,pix=3045 Blue:- Yellow:n=1,pix=3045 \nt= 82.4s | cam=1 | Red:- Green:n=6,pix=3241 Blue:- Yellow:n=1,pix=3241 \nt= 82.8s | cam=1 | Red:- Green:n=6,pix=1500 Blue:- Yellow:n=3,pix=1500 \nt= 83.4s | cam=1 | Red:- Green:n=6,pix=3224 Blue:- Yellow:n=1,pix=3224 \nt= 84.0s | cam=1 | Red:- Green:n=4,pix=4125 Blue:- Yellow:n=1,pix=4125 \nt= 85.1s | cam=1 | Red:- Green:n=3,pix=4274 Blue:- Yellow:n=2,pix=4274 \nt= 86.2s | cam=1 | Red:- Green:n=4,pix=4511 Blue:- Yellow:n=2,pix=4511 \nt= 87.3s | cam=1 | Red:- Green:n=5,pix=3109 Blue:- Yellow:n=1,pix=3109 \nt= 88.4s | cam=1 | Red:- Green:n=3,pix=3302 Blue:- Yellow:n=1,pix=3302 \nt= 89.4s | cam=1 | Red:- Green:n=3,pix=5951 Blue:- Yellow:n=1,pix=5951 \nt= 91.5s | cam=1 | Red:- Green:n=3,pix=4765 Blue:- Yellow:n=1,pix=4765 \nt= 91.7s | cam=1 | Red:- Green:n=5,pix=3922 Blue:- Yellow:n=2,pix=3922 \nt= 93.0s | cam=1 | Red:- Green:n=6,pix=2307 Blue:- Yellow:n=3,pix=2307 \nt= 94.9s | cam=1 | Red:- Green:n=3,pix=2251 Blue:- Yellow:n=4,pix=2251 \nt= 95.1s | cam=1 | Red:- Green:n=3,pix=482 Blue:- Yellow:n=7,pix=1212 \nt= 96.2s | cam=1 | Red:- Green:n=3,pix=3771 Blue:- Yellow:n=1,pix=3771 \n","truncated":false}}
%---
