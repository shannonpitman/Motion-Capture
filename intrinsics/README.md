# Intrinsic calibration - OpenMV RT1062

    captureCalib.py       runs on the cam, writes .pgm frames to /sd/calib_camN
    images/camN/          copy the frames here
    calibrateIntrinsics.m set SQUARE_MM + CAM_ID, run
    loadIntrinsics.m      [intr, cameraParams] = loadIntrinsics(1);

## Measuring the board

Do not trust the nominal square size. A laser printer is typically 0.2-0.5%
off, and that error goes straight into your scale.

Measure the span of **10 squares** with calipers (corner of square 1 to the far
corner of square 10) and divide by 10. Do it in both directions; if they differ
by more than ~0.1%, the print is anisotropic - measure both and use the mean,
and note it in your thesis.

Scale error does not affect fx, fy, cx, cy (those are dimensionless in pixels).
It scales translations - so it does matter for your stereo baseline and for any
metric 3D output.

Tape the sheet to float glass or a foam board. A 1 mm bow at 300 mm is a bigger
error source than anything else in this procedure.

## How many photos, where, at what angle

Target: **40 captured, 25-30 surviving the prune**. Below ~20 the distortion
terms get noisy; above ~40 you gain almost nothing.

1. **Distances** - 3 bands. Near (board fills ~80% of frame), mid (~50%), far
   (~30%). Roughly 15 / 15 / 10 shots. Near shots pin the distortion, far shots
   pin the focal length.
2. **Angles** - tilt the board 30-45 deg about the vertical axis, then the
   horizontal axis, both signs. Never leave it square-on to the sensor for more
   than ~5 of the shots: a set of frontal-parallel views is degenerate and fx/fy
   will not separate. Also roll it +/-30 deg in-plane.
3. **Coverage** - mentally split the frame into a 3x3 grid and put the board
   centre in each cell at least 3 times. The corner cells matter most; k1/k2/k3
   are only constrained by data far from the principal point. The coverage plot
   at the end of the MATLAB script tells you if you missed a region - reshoot
   rather than accept a hole.
4. **Sharpness** - blur biases corners inward. Brace the board against
   something, keep it still for the full red-LED period, and check a few frames
   at 100% before you calibrate.
5. **Illumination** - IR spotlight from behind the camera, off-axis by ~20 deg
   so the specular hotspot from the paper lands outside the frame. Glossy paper
   will kill you here; matte is much better. Watch the printed `mean`/`max`: you
   want max around 200-240 with no flat white patch over the board.

Checkerboard vs ChArUco: use the **plain checkerboard** for intrinsics.
MATLAB's `detectCheckerboardPoints` gives saturated-subpixel corners and needs
the whole board visible, which is exactly what you want here. ChArUco earns its
keep when the board must run off the frame edge or be partly occluded - if you
end up needing that, set `'PartialDetections', true` in the detect call instead.

## What MATLAB is actually doing

`detectCheckerboardPoints(files)`
  Finds saddle points: filters for the X-junction signature, grows the grid,
  then refines each corner to subpixel by fitting the local intensity gradients
  (the corner is where the image gradient is orthogonal to the edge direction
  in a least-squares sense). Returns `imagePoints` [numCorners x 2 x numImages]
  and `boardSize`. Any image where the full grid was not found is dropped via
  `imagesUsed`.

`generateCheckerboardPoints(boardSize, squareSize)`
  The model: the ideal board in its own frame, Z = 0, corners at integer
  multiples of `squareSize`. This is the only place your measured mm enters.

`estimateCameraParameters(imagePoints, worldPoints, ...)`
  Zhang's method, in three stages.

  1. **Homographies.** Because the board is planar (Z = 0), the projection
     collapses to `s*[u v 1]' = H*[X Y 1]'` with `H = K*[r1 r2 t]`. One H is fit
     per image by DLT on that image's corner correspondences.
  2. **Closed-form K.** r1 and r2 are columns of a rotation matrix, so they are
     orthonormal. Writing `B = K^-T K^-1` those two constraints
     (`h1' B h2 = 0`, `h1' B h1 = h2' B h2`) are linear in the 6 unknowns of the
     symmetric B. Each image gives 2 equations, so 3+ views in general position
     solve it; B is factored (Cholesky) into K, giving fx, fy, cx, cy. With
     `'EstimateSkew', false` the skew term is forced to 0. Then per-image
     extrinsics fall out: `r1 = K^-1 h1 / s`, `r2 = K^-1 h2 / s`,
     `r3 = r1 x r2`, `t = K^-1 h3 / s`. This stage assumes zero distortion,
     which is why it is only an initialisation.
  3. **Nonlinear refinement.** All parameters - fx, fy, cx, cy, k1..k3, p1, p2,
     and the 6 extrinsics per image - are refined together by Levenberg-
     Marquardt, minimising the total squared reprojection error
     `sum ||m_ij - project(K, dist, R_i, t_i, M_j)||^2`, with the distortion
     model applied in normalised coordinates before K:
       `x_d = x(1 + k1 r^2 + k2 r^4 + k3 r^6) + [2 p1 x y + p2(r^2 + 2x^2)]`
     Distortion starts at 0 and is estimated inside this optimisation.
     `MeanReprojectionError` is the RMS of the residual at convergence.

  Returns `imagesUsed` (images the optimiser rejected) and `estimationErrors` -
  standard errors taken from the covariance `sigma^2 * (J'J)^-1` of the final
  Jacobian. If `k3`'s standard error is the same size as `k3` itself, drop to
  `NUM_RADIAL = 2`; an unsupported parameter makes the fit worse, not better.

The prune loop in `calibrateIntrinsics.m` refits after removing images above
`MAX_ERR_PX`. This is not cosmetic - one blurred or bowed board drags the
distortion terms for every image.

## Sanity checks before you trust the result

- RMS below ~0.25 px on a VGA sensor. Above 0.5 px, something is wrong
  (blur, board flatness, wrong square size, mixed resolutions).
- cx, cy within ~30 px of (320, 240), and their standard errors well under the
  offset itself.
- fx and fy within ~1% of each other unless you know the pixels are not square.
- Errors evenly spread across images in `showReprojectionErrors` - one tall bar
  means a bad frame slipped through.
- The undistorted montage: straight edges in the scene must be straight.

## Important

Calibrate at the same 1:1 VGA readout the tracker uses. Intrinsics are in
pixels, so they are only valid for the resolution they were measured at - which
is exactly why `MAX_RATIO = 1.0` is pinned in `kalmanFilter.py`. Do all three
cameras separately; do not share one set of intrinsics across cams.
