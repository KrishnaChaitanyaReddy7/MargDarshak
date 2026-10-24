%% demo_detectUnmarkedRoad.m
% Perception pipeline demonstration for marked and unmarked roads.
%
% This script demonstrates:
%   1. Automatic input resolution normalization for deep learning (handles any size, from 320x227 up to 24MP photos).
%   2. Deep learning semantic road segmentation (DeepLabv3+ with ResNet backbone).
%   3. Hierarchical lane detection:
%      - If painted lane markings exist: tracks ego lane lines (Green) & center (Cyan).
%      - If unmarked road: extracts physical road margins via RANSAC (Yellow) & center corridor (Cyan).
%
% Compatible with Person 3 (integration) and Person 4 (motion planner).

clc; close all;

%% 1. Choose Image to Test
% You can change testFile to any image you want to evaluate:
% - 'training/road.jpg'       -> 24 MP photo of an unmarked street
% - 'training/test_image.jpg' -> Marked highway scene with lane markings
testFile = 'training/road.jpg';

scriptDir = fileparts(mfilename('fullpath'));
candidatePaths = {testFile, ...
                  fullfile(scriptDir, testFile), ...
                  fullfile(scriptDir, 'Video_Project_1_frames', testFile), ...
                  fullfile(scriptDir, 'video project frames', testFile), ...
                  fullfile(scriptDir, 'training', 'road.jpg'), ...
                  fullfile(scriptDir, 'road.jpg'), ...
                  'road.jpg'};

foundPath = '';
for k = 1:numel(candidatePaths)
    if exist(candidatePaths{k}, 'file')
        foundPath = candidatePaths{k};
        break;
    end
end

if isempty(foundPath)
    error('Test image not found. Checked candidate paths relative to %s', scriptDir);
end
testFile = foundPath;

fprintf('====================================================\n');
fprintf('  AUTONOMOUS ROAD PERCEPTION & BOUNDARY DETECTION   \n');
fprintf('====================================================\n');
fprintf('Loading test image: %s\n', testFile);
frame = imread(testFile);
[origH, origW, ~] = size(frame);
fprintf('Image Resolution: %d x %d pixels\n', origW, origH);

%% 2. Run Perception Pipeline
fprintf('\nRunning Deep Learning road segmentation and boundary extraction...\n');
tic;
result = detectUnmarkedRoad(frame);
execTime = toc;

%% 3. Print Results Summary
fprintf('\n------------------ PERCEPTION RESULTS ------------------\n');
fprintf('Processing Time:       %.2f seconds\n', execTime);
fprintf('Unmarked Road?:        %s\n', mat2str(result.isUnmarkedRoad));
fprintf('Perception Mode:       %s\n', result.laneType);
fprintf('Road Coverage:         %.2f%% of image\n', ...
    100 * sum(result.drivableMask(:)) / numel(result.drivableMask));

if ~result.isUnmarkedRoad
    fprintf('Status:                Marked Lane Following Active (Green Lane Lines)\n');
else
    fprintf('Status:                Unmarked Road Corridor Active (Yellow Boundaries)\n');
end
fprintf('--------------------------------------------------------\n');

%% 4. Display Visual Results (3-Panel Display: Raw Image | Drivable Mask | Perception Overlay)
hFig = figure('Name', 'Road Perception: Raw Image | Road Mask | Boundary Overlay', ...
              'NumberTitle', 'off', 'Position', [40, 80, 1650, 520]);

% Panel 1: Raw Camera Input (Original Image)
subplot(1, 3, 1);
rawDisplay = imresize(frame, [500, 700]);
imshow(rawDisplay);
title(sprintf('1. Raw Camera Input\n(%d x %d px)', origW, origH), ...
    'FontSize', 12, 'FontWeight', 'bold');

% Panel 2: Deep Learning Drivable Road Mask
subplot(1, 3, 2);
maskDisplay = imresize(uint8(result.drivableMask) * 255, [500, 700]);
imshow(maskDisplay);
title(sprintf('2. Deep Learning Road Mask\n(%.1f%% Drivable Surface)', ...
    100 * sum(result.drivableMask(:)) / numel(result.drivableMask)), ...
    'FontSize', 12, 'FontWeight', 'bold');

% Panel 3: Autonomous Perception & Trajectory Overlay
subplot(1, 3, 3);
overlayDisplay = imresize(result.overlayImage, [500, 700]);
imshow(overlayDisplay);
if ~result.isUnmarkedRoad
    title(sprintf('3. Perception: %s\n(Painted Lanes: Green | Center: Cyan)', result.laneType), ...
        'FontSize', 12, 'FontWeight', 'bold');
else
    title(sprintf('3. Perception: %s\n(Road Corridor: Yellow | Center: Cyan)', result.laneType), ...
        'FontSize', 12, 'FontWeight', 'bold');
end

fprintf('\nFigure displayed with 3-panel view (Raw Image, Drivable Mask, Perception Overlay).\n');
