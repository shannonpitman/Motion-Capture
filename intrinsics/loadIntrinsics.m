function [intr, cameraParams] = loadIntrinsics(camId, folder)
% [intr, cameraParams] = loadIntrinsics(1)
%   intr.K, intr.focalLength, intr.principalPoint, intr.radialDistortion,
%   intr.tangentialDistortion, intr.imageSize, intr.rmsError
%   intr.intrinsics  -> cameraIntrinsics object (undistortPoints, triangulate, ...)

if nargin < 2, folder = fileparts(mfilename('fullpath')); end
f = fullfile(folder, sprintf('intrinsics_cam%d.mat', camId));
assert(isfile(f), 'No intrinsics for camera %d at %s', camId, f);

S = load(f);
intr = S.intr;
cameraParams = S.cameraParams;
fprintf('cam %d: f = [%.1f %.1f] px, c = [%.1f %.1f], RMS %.3f px, %d images (%s)\n', ...
    camId, intr.focalLength, intr.principalPoint, intr.rmsError, intr.numImages, ...
    string(intr.date, 'yyyy-MM-dd'));
end
