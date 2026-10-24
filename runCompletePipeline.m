function [drivableMask, details] = runCompletePipeline(imageFile, options)
% RUNCOMPLETEPIPELINE Full perception stack combining road surface + obstacles + divider
%
% Syntax:
%   drivableMask = runCompletePipeline(imageFile)
%   [drivableMask, details] = runCompletePipeline(imageFile, options)
%
% Options:
%   options.useDeepLearningRoad - Use trained DeepLabv3+ net (default: true)
%   options.addDivider          - Add manual center divider (default: false)
%   options.cropDashboard       - Crop bottom dashboard (default: false)
%   options.cropRatio           - Dashboard crop ratio (default: 0.65)
%   options.showPlots           - Show standalone figures (default: false)

if nargin < 2
    options = struct();
end
if ~isfield(options, 'useDeepLearningRoad'), options.useDeepLearningRoad = true; end
if ~isfield(options, 'addDivider'),          options.addDivider = false; end
if ~isfield(options, 'cropDashboard'),       options.cropDashboard = false; end
if ~isfield(options, 'cropRatio'),           options.cropRatio = 0.65; end
if ~isfield(options, 'showPlots'),           options.showPlots = false; end

%% Step 1: Load image
if ischar(imageFile) || isstring(imageFile)
    I = imread(imageFile);
else
    I = imageFile;
end

%% Step 2: Optional dashboard crop
if options.cropDashboard
    h = size(I, 1);
    cropHeight = round(h * options.cropRatio);
    I = I(1:cropHeight, :, :);
end

[H, W, ~] = size(I);

%% Step 3: Road-surface estimation (Deep Learning or HSV fallback)
roadSurfaceMask = false(H, W);
roadSource = "HSV Filter";

if options.useDeepLearningRoad && exist('detectUnmarkedRoad', 'file') == 2
    try
        res = detectUnmarkedRoad(I);
        if isfield(res, 'drivableMask')
            roadSurfaceMask = res.drivableMask;
        elseif isfield(res, 'roadMask')
            roadSurfaceMask = res.roadMask;
        end
        roadSource = "DeepLabv3+ ResNet-18 (GPU Trained)";
    catch
        % Fall back to HSV color thresholding
        options.useDeepLearningRoad = false;
    end
end

if ~options.useDeepLearningRoad || ~any(roadSurfaceMask(:))
    hsvImg = rgb2hsv(I);
    S = hsvImg(:,:,2);
    V = hsvImg(:,:,3);
    roadSurfaceMask = (S < 0.25) & (V > 0.15) & (V < 0.75);
    roadSurfaceMask = imopen(roadSurfaceMask, strel('disk', 3));
    roadSurfaceMask = imclose(roadSurfaceMask, strel('disk', 5));
    roadSource = "HSV Color Space Segmentation";
end

%% Step 4: Obstacle detection (Indian Road Deep Learning Detector)
[obstacleFreeMask, bboxes, labels, obstacleMask] = runDetectionPipeline(I, false);

%% Step 5: Combine road-surface minus dynamic obstacles into final drivable corridor
drivableMask = double(roadSurfaceMask);
drivableMask(obstacleMask) = 0;

%% Package details
details = struct();
details.image = I;
details.roadSurfaceMask = roadSurfaceMask;
details.roadSource = roadSource;
details.bboxes = bboxes;
details.labels = labels;
details.obstacleMask = obstacleMask;
details.drivableMask = drivableMask;

%% Step 7: Optional plotting
if options.showPlots
    figure('Name', 'Complete Pipeline Result');
    imagesc(drivableMask);
    colormap([0.8 0.1 0.1; 0.2 0.7 0.2]);
    set(gca, 'YDir', 'reverse');
    axis equal;
    axis([0.5 W+0.5 0.5 H+0.5]);
    title("Complete Drivable Mask (" + roadSource + ")");
end
end
