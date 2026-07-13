clear; clc; close all;

%CONFIG
TEST_LABEL    = 'all_dynamic_florescent'; % describe the run (Colour|static/dynamic|daylight\syntheticlight)
RUN_DURATION  = 120; % seconds
LISTEN_PORT   = 5005; % openMV script -> port to listen to
featureNames  = {'Red', 'Green', 'Blue', 'Yellow'};
numFeatures   = numel(featureNames);

%UDP PORT
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

maxLog = 200000;   
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


while toc(tStart) < RUN_DURATION %[output:group:771c4f55]
    if u.NumBytesAvailable == 0
        pause(0.001);
        continue;
    end

    try
        line = readline(u); 
    catch
        continue;
    end

    if strlength(line) == 0
        continue;
    end

    matlabToc = toc(tStart);

    if startsWith(line, "HEADER")
        headerInfo = char(line);
        fprintf("Received header: %s\n", headerInfo);
        continue;
    end


    parts = split(line, ",");
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

    %Summary line every 30 packets 
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
        fprintf("%s\n", summary); %[output:20563cff]
    end
end %[output:group:771c4f55]

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

fprintf("\nDone. %d packets logged over %.1f s -> %s\n", ... %[output:group:4f2216a7] %[output:10b41443]
        idx, toc(tStart), fname); %[output:group:4f2216a7] %[output:10b41443]

clear u;

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":16.3}
%---
%[output:7c200484]
%   data: {"dataType":"text","outputData":{"text":"Listening on UDP port 5005\n","truncated":false}}
%---
%[output:1f53e3c7]
%   data: {"dataType":"text","outputData":{"text":"Waiting for packets... (Ctrl+C to stop early)\n","truncated":false}}
%---
%[output:20563cff]
%   data: {"dataType":"text","outputData":{"text":"t=  0.7s | cam=1 | Red:n=1,pix=210 Green:- Blue:- Yellow:- \nt=  1.6s | cam=1 | Red:n=2,pix=979 Green:- Blue:n=1,pix=253 Yellow:- \nt=  2.6s | cam=1 | Red:n=2,pix=378 Green:- Blue:n=1,pix=104 Yellow:- \nt=  3.5s | cam=1 | Red:n=2,pix=396 Green:n=1,pix=108 Blue:n=1,pix=169 Yellow:- \nt=  4.4s | cam=1 | Red:n=2,pix=581 Green:n=1,pix=139 Blue:n=1,pix=149 Yellow:- \nt=  5.3s | cam=1 | Red:n=2,pix=187 Green:- Blue:n=1,pix=143 Yellow:- \nt=  6.3s | cam=1 | Red:n=2,pix=158 Green:n=1,pix=128 Blue:n=1,pix=115 Yellow:n=1,pix=114 \nt=  7.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=  8.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=  8.9s | cam=1 | Red:n=2,pix=313 Green:- Blue:n=1,pix=116 Yellow:- \nt=  9.9s | cam=1 | Red:n=2,pix=379 Green:- Blue:n=1,pix=111 Yellow:- \nt= 10.8s | cam=1 | Red:n=2,pix=567 Green:- Blue:n=1,pix=160 Yellow:- \nt= 11.8s | cam=1 | Red:n=1,pix=100 Green:- Blue:n=1,pix=108 Yellow:- \nt= 12.6s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 13.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 14.4s | cam=1 | Red:n=2,pix=1276 Green:- Blue:- Yellow:- \nt= 15.3s | cam=1 | Red:n=1,pix=128 Green:- Blue:- Yellow:- \nt= 16.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 17.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 17.8s | cam=1 | Red:n=1,pix=181 Green:- Blue:- Yellow:- \nt= 18.7s | cam=1 | Red:n=1,pix=253 Green:- Blue:- Yellow:- \nt= 19.6s | cam=1 | Red:n=2,pix=501 Green:- Blue:n=2,pix=104 Yellow:- \nt= 20.5s | cam=1 | Red:n=1,pix=499 Green:n=1,pix=103 Blue:- Yellow:- \nt= 21.5s | cam=1 | Red:n=1,pix=419 Green:- Blue:- Yellow:- \nt= 22.4s | cam=1 | Red:n=1,pix=255 Green:- Blue:- Yellow:- \nt= 23.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 24.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 24.9s | cam=1 | Red:n=1,pix=253 Green:- Blue:- Yellow:- \nt= 25.8s | cam=1 | Red:n=2,pix=758 Green:- Blue:- Yellow:- \nt= 26.7s | cam=1 | Red:n=2,pix=733 Green:- Blue:- Yellow:- \nt= 27.6s | cam=1 | Red:n=2,pix=465 Green:- Blue:- Yellow:- \nt= 28.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 29.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 30.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 30.9s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 31.7s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 32.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 33.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 34.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 35.0s | cam=1 | Red:n=1,pix=139 Green:- Blue:- Yellow:- \nt= 35.9s | cam=1 | Red:n=1,pix=258 Green:- Blue:- Yellow:- \nt= 36.8s | cam=1 | Red:n=2,pix=536 Green:- Blue:n=1,pix=101 Yellow:- \nt= 37.6s | cam=1 | Red:n=1,pix=562 Green:- Blue:- Yellow:- \nt= 38.6s | cam=1 | Red:n=2,pix=969 Green:n=1,pix=129 Blue:n=1,pix=129 Yellow:n=1,pix=127 \nt= 39.5s | cam=1 | Red:n=2,pix=877 Green:n=1,pix=118 Blue:n=1,pix=135 Yellow:n=1,pix=121 \nt= 40.5s | cam=1 | Red:n=2,pix=853 Green:n=1,pix=154 Blue:- Yellow:n=1,pix=116 \nt= 41.4s | cam=1 | Red:n=2,pix=667 Green:- Blue:n=1,pix=142 Yellow:- \nt= 42.3s | cam=1 | Red:n=2,pix=542 Green:- Blue:n=1,pix=122 Yellow:- \nt= 43.2s | cam=1 | Red:n=3,pix=595 Green:- Blue:n=1,pix=102 Yellow:- \nt= 44.2s | cam=1 | Red:n=1,pix=1460 Green:- Blue:- Yellow:- \nt= 45.1s | cam=1 | Red:n=1,pix=166 Green:- Blue:- Yellow:- \nt= 45.9s | cam=1 | Red:n=1,pix=201 Green:- Blue:- Yellow:- \nt= 46.8s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 47.6s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 48.5s | cam=1 | Red:n=2,pix=178 Green:- Blue:- Yellow:- \nt= 49.4s | cam=1 | Red:n=1,pix=116 Green:- Blue:- Yellow:- \nt= 50.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 51.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 52.0s | cam=1 | Red:n=1,pix=113 Green:- Blue:n=2,pix=121 Yellow:- \nt= 52.9s | cam=1 | Red:n=1,pix=115 Green:- Blue:n=1,pix=109 Yellow:- \nt= 53.8s | cam=1 | Red:n=1,pix=101 Green:- Blue:- Yellow:- \nt= 54.7s | cam=1 | Red:n=1,pix=111 Green:- Blue:n=1,pix=103 Yellow:- \nt= 55.6s | cam=1 | Red:n=1,pix=239 Green:- Blue:- Yellow:- \nt= 56.5s | cam=1 | Red:n=1,pix=119 Green:- Blue:- Yellow:- \nt= 57.3s | cam=1 | Red:n=1,pix=149 Green:- Blue:- Yellow:- \nt= 58.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 59.1s | cam=1 | Red:n=1,pix=145 Green:- Blue:n=1,pix=145 Yellow:- \nt= 60.1s | cam=1 | Red:- Green:n=1,pix=106 Blue:n=1,pix=120 Yellow:- \nt= 61.0s | cam=1 | Red:- Green:- Blue:n=1,pix=110 Yellow:- \nt= 61.8s | cam=1 | Red:n=1,pix=112 Green:- Blue:- Yellow:- \nt= 62.7s | cam=1 | Red:- Green:- Blue:n=1,pix=101 Yellow:- \nt= 63.6s | cam=1 | Red:n=1,pix=122 Green:n=1,pix=108 Blue:n=1,pix=159 Yellow:- \nt= 64.4s | cam=1 | Red:n=1,pix=125 Green:- Blue:n=1,pix=129 Yellow:- \nt= 65.3s | cam=1 | Red:n=1,pix=109 Green:n=1,pix=105 Blue:n=1,pix=115 Yellow:- \nt= 66.2s | cam=1 | Red:n=1,pix=103 Green:- Blue:n=1,pix=107 Yellow:- \nt= 67.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 67.9s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 68.7s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 69.6s | cam=1 | Red:n=1,pix=116 Green:- Blue:n=1,pix=111 Yellow:- \nt= 70.3s | cam=1 | Red:n=2,pix=163 Green:- Blue:n=1,pix=109 Yellow:- \nt= 71.0s | cam=1 | Red:n=2,pix=171 Green:- Blue:n=1,pix=107 Yellow:- \nt= 71.9s | cam=1 | Red:n=1,pix=107 Green:- Blue:- Yellow:- \nt= 72.8s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 73.7s | cam=1 | Red:n=2,pix=177 Green:n=1,pix=143 Blue:n=1,pix=145 Yellow:n=1,pix=112 \nt= 74.6s | cam=1 | Red:n=2,pix=299 Green:n=1,pix=117 Blue:n=1,pix=112 Yellow:- \nt= 75.6s | cam=1 | Red:n=2,pix=136 Green:- Blue:n=1,pix=131 Yellow:- \nt= 76.5s | cam=1 | Red:n=1,pix=119 Green:n=1,pix=112 Blue:n=1,pix=116 Yellow:- \nt= 77.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 78.2s | cam=1 | Red:n=1,pix=110 Green:- Blue:- Yellow:- \nt= 79.1s | cam=1 | Red:n=1,pix=104 Green:- Blue:n=1,pix=103 Yellow:- \nt= 80.0s | cam=1 | Red:n=1,pix=184 Green:- Blue:n=1,pix=108 Yellow:- \nt= 81.0s | cam=1 | Red:n=1,pix=159 Green:- Blue:- Yellow:- \nt= 81.9s | cam=1 | Red:n=1,pix=156 Green:- Blue:- Yellow:- \nt= 82.7s | cam=1 | Red:- Green:- Blue:n=1,pix=120 Yellow:- \nt= 83.6s | cam=1 | Red:n=1,pix=109 Green:- Blue:n=1,pix=102 Yellow:- \nt= 84.5s | cam=1 | Red:n=2,pix=175 Green:n=1,pix=123 Blue:- Yellow:- \nt= 85.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 86.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 87.2s | cam=1 | Red:n=3,pix=147 Green:n=1,pix=120 Blue:- Yellow:- \nt= 88.1s | cam=1 | Red:n=2,pix=187 Green:n=1,pix=101 Blue:n=1,pix=103 Yellow:- \nt= 89.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 89.9s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 90.7s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 91.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 92.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 93.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 94.1s | cam=1 | Red:n=1,pix=636 Green:- Blue:- Yellow:- \nt= 95.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 95.8s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 96.6s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 97.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 98.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt= 99.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=100.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=100.9s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=101.7s | cam=1 | Red:n=1,pix=105 Green:- Blue:- Yellow:- \nt=102.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=103.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=104.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=105.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=105.9s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=106.7s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=107.6s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=108.4s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=109.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=110.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=111.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=111.9s | cam=1 | Red:n=1,pix=101 Green:- Blue:- Yellow:- \nt=112.7s | cam=1 | Red:n=1,pix=130 Green:- Blue:- Yellow:- \nt=113.6s | cam=1 | Red:n=1,pix=102 Green:- Blue:- Yellow:- \nt=114.5s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=115.3s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=116.2s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=117.1s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=118.0s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=118.8s | cam=1 | Red:- Green:- Blue:- Yellow:- \nt=119.6s | cam=1 | Red:- Green:- Blue:- Yellow:- \n","truncated":false}}
%---
%[output:10b41443]
%   data: {"dataType":"text","outputData":{"text":"\nDone. 4122 packets logged over 120.1 s -> depth_test_all_dynamic_florescent_20260423_201957.mat\n","truncated":false}}
%---
