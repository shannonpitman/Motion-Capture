function log = collectCalib(outFile, durationSec, opts)
%COLLECTCALIB  Capture strobed marker packets from the OpenMV cameras.
%
%   log = collectCalib('wandRun.mat', 120)
%   log = collectCalib('axisRun.mat', 10, 'Port', 7008, 'NumCams', 7)
%
% Listens on a UDP port for packets from extrinsicsCalib.py, decodes them and
% saves a log. Run this once per recording:
%   - one long run while sweeping the wand   -> buildEasyWand(..., 'wand')
%   - one short run with the rig held static -> buildEasyWand(..., 'axis')
%
% Each packet carries all four markers already ordered A,B,C,D by the camera,
% so no cross-camera matching is needed here.
%
% Shannon Pitman

arguments
    outFile     (1,:) char
    durationSec (1,1) double = 120
    opts.Port      (1,1) double = 7008
    opts.NumCams   (1,1) double = 7
    opts.NumMarkers(1,1) double = 4
    opts.Verbose   (1,1) logical = true
end

MAGIC      = hex2dec('4C');
HDR_BYTES  = 10;
MRK_BYTES  = 13;
pktBytes   = HDR_BYTES + MRK_BYTES * opts.NumMarkers;

u = udpport("datagram", "LocalPort", opts.Port, ...
            "EnablePortSharing", false, "Timeout", 1);
cleanup = onCleanup(@() delete(u));   %#ok<NASGU>

cap = 200000;
log = struct();
log.tHost   = zeros(cap,1);                        % host arrival, seconds
log.camId   = zeros(cap,1,'uint8');
log.seq     = zeros(cap,1,'uint16');
log.tUs     = zeros(cap,1,'uint32');
log.nValid  = zeros(cap,1,'uint8');
log.uv      = nan(cap, 2*opts.NumMarkers);         % [uA vA uB vB uC vC uD vD]
log.sigma   = nan(cap, opts.NumMarkers);
log.flags   = zeros(cap, opts.NumMarkers,'uint8');

n = 0;  nBad = 0;  seen = false(1, opts.NumCams);
t0 = tic;  lastReport = 0;

if opts.Verbose
    fprintf('Listening on UDP %d for %g s (packet = %d bytes)...\n', ...
            opts.Port, durationSec, pktBytes);
end

while toc(t0) < durationSec
    k = u.NumDatagramsAvailable;
    if k == 0
        pause(0.005);
    else
        dg = read(u, k, "uint8");
        tNow = toc(t0);
        for i = 1:numel(dg)
            b = uint8(dg(i).Data(:)).';
            if numel(b) ~= pktBytes || b(1) ~= MAGIC
                nBad = nBad + 1;  continue
            end
            n = n + 1;
            if n > cap, error('collectCalib:full','log capacity exceeded'); end

            log.tHost(n)  = tNow;
            log.camId(n)  = b(2);
            log.nValid(n) = b(3);
            log.seq(n)    = typecast(b(5:6),  'uint16');
            log.tUs(n)    = typecast(b(7:10), 'uint32');

            if b(2) >= 1 && b(2) <= opts.NumCams, seen(b(2)) = true; end

            if b(3) > 0
                for m = 1:opts.NumMarkers
                    o = HDR_BYTES + (m-1)*MRK_BYTES;
                    log.uv(n, 2*m-1)  = double(typecast(b(o+1 :o+4 ),'single'));
                    log.uv(n, 2*m  )  = double(typecast(b(o+5 :o+8 ),'single'));
                    log.sigma(n, m)   = double(typecast(b(o+9 :o+12),'single'));
                    log.flags(n, m)   = b(o+13);
                end
            end
        end
    end

    if opts.Verbose && toc(t0) - lastReport >= 2
        lastReport = toc(t0);
        fprintf('  %5.1f s  packets=%6d  hits=%5d  cams seen: %s\n', ...
                lastReport, n, sum(log.nValid(1:n) > 0), ...
                mat2str(find(seen)));
    end
end

% trim
f = fieldnames(log);
for i = 1:numel(f), log.(f{i}) = log.(f{i})(1:n, :); end

log.numCams    = opts.NumCams;
log.numMarkers = opts.NumMarkers;
log.duration   = toc(t0);

missing = find(~seen);
if ~isempty(missing)
    warning('collectCalib:missingCams', ...
            'No packets from camera(s): %s', mat2str(missing));
end
if nBad > 0
    fprintf('Discarded %d malformed datagrams.\n', nBad);
end

fprintf(['Captured %d packets, %d with detections, from %d/%d cameras ' ...
         'in %.1f s.\n'], n, sum(log.nValid > 0), sum(seen), ...
         opts.NumCams, log.duration);

if nargin >= 1 && ~isempty(outFile)
    save(outFile, '-struct', 'log');
    fprintf('Saved %s\n', outFile);
end
end
