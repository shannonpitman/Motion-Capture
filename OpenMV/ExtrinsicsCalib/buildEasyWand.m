function out = buildEasyWand(logIn, mode, opts)
%BUILDEASYWAND  Combine all camera streams into easyWand6 input files.
%
%   buildEasyWand('wandRun.mat', 'wand')
%   buildEasyWand('axisRun.mat', 'axis')
%   buildEasyWand(log, 'wand', 'PulseHz', 3, 'OutDir', 'easyWandIn')
%
% Packets arrive independently from each camera. This groups them into strobe
% pulses and lays them out in easyWand6's column order.
%
% Grouping is by HOST ARRIVAL TIME, not by the camera clocks - those are
% free-running and unsynchronised. It works because every camera that catches
% a given pulse reports it within one frame period, while consecutive pulses
% are a whole pulse period apart. REQUIRES frame period < pulse period, with
% margin. Check the diagnostics this prints.
%
% mode 'wand' writes:
%   wandPoints.csv      4*N cols, one row per pulse. pt1 = marker A,
%                       pt2 = marker C. Point-major: all cameras' pt1, then
%                       all cameras' pt2.
%   unpairedPoints.csv  2*N cols, one row per marker observation (4 per pulse).
%
% mode 'axis' writes:
%   axisPoints.csv      2*N cols, 3 rows: A = origin, C = +X, D = +Y.
%                       Averaged over the whole static recording.
%
% Shannon Pitman

arguments
    logIn
    mode (1,:) char {mustBeMember(mode,{'wand','axis'})}
    opts.PulseHz  (1,1) double = 3
    opts.MinCams  (1,1) double = 2
    opts.OutDir   (1,:) char   = 'easyWandIn'
    opts.Verbose  (1,1) logical = true
end

if ischar(logIn) || isstring(logIn)
    log = load(char(logIn));
else
    log = logIn;
end

N  = log.numCams;
M  = log.numMarkers;
if M ~= 4
    error('buildEasyWand:markers','expected 4 markers, log has %d', M);
end
IA = 1; IC = 3; ID = 4;                    % A,B,C,D order from order_markers

if ~exist(opts.OutDir,'dir'), mkdir(opts.OutDir); end

keep = log.nValid > 0;
if ~any(keep)
    error('buildEasyWand:empty','no packets carry detections');
end
tH  = log.tHost(keep);
cam = double(log.camId(keep));
uv  = log.uv(keep,:);
sg  = log.sigma(keep,:);

[tH, ord] = sort(tH);
cam = cam(ord);  uv = uv(ord,:);  sg = sg(ord,:);

% --- group into pulses -----------------------------------------------------
gap  = 0.5 / opts.PulseHz;
brk  = [true; diff(tH) > gap];
pIdx = cumsum(brk);
nP   = pIdx(end);

if opts.Verbose
    d = diff(tH);
    fprintf('Pulse grouping: %d pulses from %d detections (gap = %.3f s)\n', ...
            nP, numel(tH), gap);
    ds = sort(d);
    p90 = ds(max(1, ceil(0.90*numel(ds))));
    fprintf('  inter-packet gaps: median %.4f s, 90th pct %.4f s\n', ...
            median(d), p90);
    fprintf('  --> within-pulse spread must stay well under %.3f s\n', gap);
end

% --- fold into [pulse x camera x marker] ----------------------------------
U = nan(nP, N, M);  V = nan(nP, N, M);  S = inf(nP, N);
for i = 1:numel(tH)
    c = cam(i);
    if c < 1 || c > N, continue; end
    p = pIdx(i);
    s = mean(sg(i,:),'omitnan');
    if s < S(p,c)                          % duplicate camera in one pulse ->
        S(p,c) = s;                        % keep the lower-sigma detection
        for m = 1:M
            U(p,c,m) = uv(i, 2*m-1);
            V(p,c,m) = uv(i, 2*m);
        end
    end
end

camsPer = squeeze(sum(~isnan(U(:,:,IA)), 2));
good    = camsPer >= opts.MinCams;

if opts.Verbose
    fprintf('  cameras per pulse: ');
    for k = 0:N, fprintf('%d:%d ', k, sum(camsPer==k)); end
    fprintf('\n  %d/%d pulses have >= %d cameras\n', ...
            sum(good), nP, opts.MinCams);
end
if ~any(good)
    error('buildEasyWand:noRows','no pulse was seen by %d+ cameras', opts.MinCams);
end

out = struct('numPulses', nP, 'numUsable', sum(good), 'camsPerPulse', camsPer);

switch mode
% ---------------------------------------------------------------------------
case 'wand'
    P = find(good);
    W = nan(numel(P), 4*N);                % pt1 all cams, then pt2 all cams
    for k = 1:numel(P)
        p = P(k);
        for c = 1:N
            W(k, 2*c-1)       = U(p,c,IA);   W(k, 2*c)       = V(p,c,IA);
            W(k, 2*N + 2*c-1) = U(p,c,IC);   W(k, 2*N + 2*c) = V(p,c,IC);
        end
    end
    f1 = fullfile(opts.OutDir,'wandPoints.csv');
    writeNaNCsv(f1, W);

    Up = nan(numel(P)*M, 2*N);             % every marker as a loose point
    r = 0;
    for k = 1:numel(P)
        p = P(k);
        for m = 1:M
            r = r + 1;
            for c = 1:N
                Up(r, 2*c-1) = U(p,c,m);  Up(r, 2*c) = V(p,c,m);
            end
        end
    end
    f2 = fullfile(opts.OutDir,'unpairedPoints.csv');
    writeNaNCsv(f2, Up);

    out.wandFile     = f1;
    out.unpairedFile = f2;
    fprintf('\nWrote %s  (%d rows x %d cols)\n', f1, size(W,1), size(W,2));
    fprintf('Wrote %s  (%d rows x %d cols)\n', f2, size(Up,1), size(Up,2));
    fprintf(['\neasyWand6: paired points = wandPoints.csv, ' ...
             'inter-point distance = |AC| from OptiTrack.\n']);

% ---------------------------------------------------------------------------
case 'axis'
    A = nan(3, 2*N);
    src = [IA IC ID];                      % origin, +X, +Y
    for c = 1:N
        for j = 1:3
            m  = src(j);
            uu = U(good,c,m);  vv = V(good,c,m);
            ok = ~isnan(uu) & ~isnan(vv);     % joint mask - never filter apart
            uu = uu(ok);  vv = vv(ok);
            if numel(uu) >= 1
                A(j, 2*c-1) = mean(uu);  A(j, 2*c) = mean(vv);
                if opts.Verbose && numel(uu) > 1
                    fprintf('  cam%d marker%d: n=%3d  sd = %.3f, %.3f px\n', ...
                            c, m, numel(uu), std(uu), std(vv));
                end
            end
        end
    end
    f1 = fullfile(opts.OutDir,'axisPoints.csv');
    writeNaNCsv(f1, A);
    out.axisFile = f1;
    fprintf('\nWrote %s  (3 rows x %d cols)\n', f1, 2*N);
    fprintf('  row 1 = A (origin)   row 2 = C (+X)   row 3 = D (+Y)\n');
    fprintf(['  Rig must have been STATIONARY. If the sd above exceeds ' ...
             '~1 px, re-record.\n']);
end
end

% ---------------------------------------------------------------------------
function writeNaNCsv(fname, M)
% easyWand wants bare numbers with NaN for unseen, no header.
fid = fopen(fname,'w');
if fid < 0, error('cannot open %s', fname); end
c = onCleanup(@() fclose(fid));
for i = 1:size(M,1)
    row = M(i,:);
    s = cell(1, numel(row));
    for j = 1:numel(row)
        if isnan(row(j)), s{j} = 'NaN'; else, s{j} = sprintf('%.4f', row(j)); end
    end
    fprintf(fid, '%s\n', strjoin(s, ','));
end
end
