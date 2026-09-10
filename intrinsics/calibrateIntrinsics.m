% Checkerboard intrinsic calibration for OpenMV RT1062 camera
% set SQUARE_MM run.

clear; clc; close all;

CAM_ID = 2;
SQUARE_MM = 40.0; % checkerboard square
MAX_ERR_PX = 0.5;    % drop any image whose mean reprojection error exceeds this
MIN_IMAGES = 12;     % never prune below this many
NUM_RADIAL = 3;      % 2 or 3
EST_TANGENTIAL = true;

here = fileparts(mfilename('fullpath'));
imgDir = fullfile(here, 'images', sprintf('cam%d', CAM_ID));
outFile = fullfile(here, sprintf('intrinsics_cam%d.mat', CAM_ID));

d = [dir(fullfile(imgDir,'*.pgm')); dir(fullfile(imgDir,'*.bmp')); dir(fullfile(imgDir,'*.png'))];
assert(~isempty(d), 'No images in %s', imgDir);
files = fullfile({d.folder}, {d.name});
fprintf('%d images found in %s\n', numel(files), imgDir);

%% Detect corners
[imagePoints, boardSize, used] = detectCheckerboardPoints(files);
files = files(used);
fprintf('Board detected in %d/%d images, boardSize = [%d %d] (%d corners)\n', numel(files), numel(d), boardSize(1), boardSize(2), prod(boardSize-1));
assert(numel(files) >= MIN_IMAGES, 'Too few usable images.');

worldPoints = generateCheckerboardPoints(boardSize, SQUARE_MM);
I = imread(files{1});
imageSize = [size(I,1) size(I,2)];

% The tracker reports centroids in SENSOR pixels (2592x1944). Check no
% downscaling happened
SENSOR = [1944 2592];
ratio  = SENSOR(2) / imageSize(2);
if abs(ratio - 1) > 0.01
    warning(['Frames are %dx%d, i.e. the sensor DOWNSCALED by %.3f.\n' ...
             'These intrinsics are in image pixels, NOT sensor pixels.\n' ...
             'Multiply fx, fy, cx, cy by %.3f before use with kalmanFilter.py.\n' ...
             'k and p are normalised and do not scale.\n' ...
             'Recapture with FULL_RES = true in captureCalib.py to avoid this.'], ...
            imageSize(2), imageSize(1), ratio, ratio);
end

%% Estimate + prune outliers
for pass = 1:4
    [params, used, estErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
        'ImageSize', imageSize, 'WorldUnits', 'mm', ...
        'EstimateSkew', false, ...
        'EstimateTangentialDistortion', EST_TANGENTIAL, ...
        'NumRadialDistortionCoefficients', NUM_RADIAL);
    imagePoints = imagePoints(:,:,used);  files = files(used);

    errPerImage = squeeze(mean(sqrt(sum(params.ReprojectionErrors.^2, 2)), 1));
    bad = errPerImage > MAX_ERR_PX;
    if ~any(bad) || nnz(~bad) < MIN_IMAGES, break; end
    fprintf('pass %d: dropping %d image(s) over %.2f px\n', pass, nnz(bad), MAX_ERR_PX);
    imagePoints = imagePoints(:,:,~bad);  files = files(~bad);
end

%% Report
if isprop(params,'K'), K = params.K; else, K = params.IntrinsicMatrix'; end
fl = params.FocalLength;  pp = params.PrincipalPoint;
fprintf('\nImages used        : %d\n', numel(files));
fprintf('Overall RMS error  : %.4f px\n', params.MeanReprojectionError);
fprintf('fx, fy             : %.2f +/- %.2f , %.2f +/- %.2f px\n', ...
    fl(1), estErrors.IntrinsicsErrors.FocalLengthError(1), ...
    fl(2), estErrors.IntrinsicsErrors.FocalLengthError(2));
fprintf('cx, cy             : %.2f +/- %.2f , %.2f +/- %.2f px\n', ...
    pp(1), estErrors.IntrinsicsErrors.PrincipalPointError(1), ...
    pp(2), estErrors.IntrinsicsErrors.PrincipalPointError(2));
fprintf('radial   k         : %s\n', mat2str(params.RadialDistortion, 5));
fprintf('tangential p       : %s\n', mat2str(params.TangentialDistortion, 5));
fprintf('pixel frame        : %dx%d, sensor ratio %.3f%s\n', ...
    imageSize(2), imageSize(1), ratio, ...
    string(repmat(' (SENSOR PIXELS - no scaling needed)',1,abs(ratio-1)<=0.01)));
fprintf('FOV (h, v)         : %.1f deg , %.1f deg\n', ...
    2*atand(imageSize(2)/(2*fl(1))), 2*atand(imageSize(1)/(2*fl(2))));

figure; showReprojectionErrors(params);
figure; showExtrinsics(params, 'CameraCentric');
figure; imshowpair(I, undistortImage(I, params), 'montage'); title('raw | undistorted');

% coverage: where the corners actually landed
figure; hold on; axis ij equal; xlim([0 imageSize(2)]); ylim([0 imageSize(1)]);
plot(reshape(imagePoints(:,1,:),[],1), reshape(imagePoints(:,2,:),[],1), '.');
title('corner coverage - want the whole frame, corners included');

%% Save
intr = struct();
intr.camId        = CAM_ID;
intr.K            = K;
intr.focalLength  = fl;
intr.principalPoint = pp;
intr.radialDistortion = params.RadialDistortion;
intr.tangentialDistortion = params.TangentialDistortion;
intr.imageSize    = imageSize;
intr.squareSizeMm = SQUARE_MM;
intr.rmsError     = params.MeanReprojectionError;
intr.numImages    = numel(files);
intr.files        = files;
intr.date         = datetime('now');
intr.intrinsics   = params.Intrinsics;   % cameraIntrinsics object for undistortPoints/triangulate
cameraParams      = params;
estimationErrors  = estErrors;

save(outFile, 'intr', 'cameraParams', 'estimationErrors');
fprintf('\nSaved %s\n', outFile);
