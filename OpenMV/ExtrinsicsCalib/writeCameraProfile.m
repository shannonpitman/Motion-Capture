function fname = writeCameraProfile(opts)
%WRITECAMERAPROFILE  Write the easyWand6 camera profile for 7 OpenMV cameras.
%
%   writeCameraProfile()
%   writeCameraProfile('FocalLengthMm', 2.8, 'NumCams', 7)
%   writeCameraProfile('FocalPx', [2000 2000 1990 2000 2010 2000 2000])
%
% One row per camera, 12 space-separated fields:
%   id  f_px  W  H  cx  cy  aspect  skew  d2  d4  t1  t2
%
% CRITICAL: extrinsicsCalib.py emits SENSOR pixel coordinates, so W and H here
% must be the full sensor array (2592 x 1944 on the OV5640), NOT the VGA
% framesize you capture at. f_px must likewise be in sensor pixels:
%
%       f_px = FocalLengthMm / PixelPitchMm
%
% The OV5640 pixel pitch is 1.4 um, so a 2.8 mm lens gives f_px = 2000.
%
% Pass FocalPx directly if you have per-camera values from an intrinsic
% calibration - that always beats the nominal lens figure.
%
% Shannon Pitman

arguments
    opts.NumCams       (1,1) double = 7
    opts.SensorW       (1,1) double = 2592
    opts.SensorH       (1,1) double = 1944
    opts.PixelPitchMm  (1,1) double = 0.0014
    opts.FocalLengthMm (1,1) double = 2.8
    opts.FocalPx             double = []
    opts.OutDir        (1,:) char   = 'easyWandIn'
    opts.FileName      (1,:) char   = 'cameraProfile.txt'
end

N = opts.NumCams;

if isempty(opts.FocalPx)
    fpx = repmat(opts.FocalLengthMm / opts.PixelPitchMm, 1, N);
    fprintf(['Using nominal f = %.0f px for all %d cameras ' ...
             '(%.2f mm / %.4f mm).\n'], fpx(1), N, ...
             opts.FocalLengthMm, opts.PixelPitchMm);
    fprintf('Replace with measured per-camera values when you have them.\n');
elseif isscalar(opts.FocalPx)
    fpx = repmat(opts.FocalPx, 1, N);
else
    if numel(opts.FocalPx) ~= N
        error('writeCameraProfile:size', ...
              'FocalPx has %d entries, NumCams is %d', numel(opts.FocalPx), N);
    end
    fpx = opts.FocalPx(:).';
end

cx = opts.SensorW / 2;
cy = opts.SensorH / 2;

if ~exist(opts.OutDir,'dir'), mkdir(opts.OutDir); end
fname = fullfile(opts.OutDir, opts.FileName);

fid = fopen(fname,'w');
if fid < 0, error('cannot open %s', fname); end
c = onCleanup(@() fclose(fid));

for i = 1:N
    % id f_px W H cx cy aspect skew d2 d4 t1 t2
    fprintf(fid, '%d %.0f %d %d %.0f %.0f 1 0 0 0 0 0\n', ...
            i, fpx(i), opts.SensorW, opts.SensorH, cx, cy);
end

fprintf('Wrote %s (%d cameras, %dx%d sensor)\n', ...
        fname, N, opts.SensorW, opts.SensorH);
end
