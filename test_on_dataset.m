%% test_on_dataset.m
% Test the Deep Learning perception pipeline on images from the IDD dataset.
%
% Usage:
%   1. Simply run this script in MATLAB:
%        test_on_dataset
%      By default, it picks an image from the validation set and displays the results.
%   2. To test a specific image number (1 to 204), change imgIndex below.
%   3. To test a random image each time, set imgIndex = 'random'.

clc; close all;

%% 1. Choose Image Index or Custom Image
% Options:
%   imgIndex = 'random';   % Picks a random real-world scene each time
%   imgIndex = 25;         % Specific image number (1 to 204)
imgIndex = 'random'; 

%% 2. Locate Dataset
dataDir = fullfile(fileparts(mfilename('fullpath')), 'training', 'idd20k_lite', 'leftImg8bit', 'val');
if ~exist(dataDir, 'dir')
    dataDir = fullfile(fileparts(mfilename('fullpath')), 'idd-lite', 'idd20k_lite', 'leftImg8bit', 'val');
end

valFiles = dir(fullfile(dataDir, '**', '*_image.jpg'));
totalImgs = numel(valFiles);

if totalImgs == 0
    error('No dataset images found in %s', dataDir);
end

if ischar(imgIndex) && strcmpi(imgIndex, 'random')
    selectedIdx = randi(totalImgs);
else
    selectedIdx = min(max(1, imgIndex), totalImgs);
end

selectedFile = fullfile(valFiles(selectedIdx).folder, valFiles(selectedIdx).name);

fprintf('=======================================================\n');
fprintf('       IDD DATASET PERCEPTION PIPELINE TEST           \n');
fprintf('=======================================================\n');
fprintf('Testing Image #%d of %d\n', selectedIdx, totalImgs);
fprintf('File: %s\n', selectedFile);

%% 3. Run Perception Pipeline
frame = imread(selectedFile);
[origH, origW, ~] = size(frame);

tic;
result = detectUnmarkedRoad(frame);
execTime = toc;

%% 4. Print Summary
fprintf('\n----------------- PERCEPTION SUMMARY -----------------\n');
fprintf('Execution Time:        %.2f seconds\n', execTime);
fprintf('Image Resolution:      %d x %d pixels\n', origW, origH);
fprintf('Unmarked Road?:        %s\n', mat2str(result.isUnmarkedRoad));
fprintf('Perception Mode:       %s\n', result.laneType);
fprintf('Road Surface Coverage: %.1f%%\n', ...
    100 * sum(result.drivableMask(:)) / numel(result.drivableMask));

if result.isUnmarkedRoad
    fprintf('Navigation Mode:       Unmarked Road Corridor Guidance Active\n');
else
    fprintf('Navigation Mode:       Painted Lane Line Following Active\n');
end
fprintf('------------------------------------------------------\n');

%% 5. Display Visual Overlay (3-Panel Display)
hFig = figure('Name', sprintf('Dataset Test: Image #%d (%s)', selectedIdx, result.laneType), ...
              'NumberTitle', 'off', 'Position', [40, 80, 1650, 520]);

% Panel 1: Raw Camera Input (Original Image)
subplot(1, 3, 1);
rawDisplay = imresize(frame, [500, 700]);
imshow(rawDisplay);
title(sprintf('1. Raw Image #%d\n(%d x %d px)', selectedIdx, origW, origH), ...
    'FontSize', 12, 'FontWeight', 'bold');

% Panel 2: Deep Learning Drivable Mask
subplot(1, 3, 2);
maskDisplay = imresize(uint8(result.drivableMask) * 255, [500, 700]);
imshow(maskDisplay);
title(sprintf('2. DeepLabv3+ Road Mask\n(%.1f%% Drivable Area)', ...
    100 * sum(result.drivableMask(:)) / numel(result.drivableMask)), ...
    'FontSize', 12, 'FontWeight', 'bold');

% Panel 3: Autonomous Perception Overlay
subplot(1, 3, 3);
overlayDisplay = imresize(result.overlayImage, [500, 700]);
imshow(overlayDisplay);
if result.isUnmarkedRoad
    title(sprintf('3. Perception: %s\n(Corridor: Yellow | Trajectory: Cyan)', result.laneType), ...
        'FontSize', 12, 'FontWeight', 'bold');
else
    title(sprintf('3. Perception: %s\n(Lanes: Green | Trajectory: Cyan)', result.laneType), ...
        'FontSize', 12, 'FontWeight', 'bold');
end

fprintf('\nResult figure displayed with 3-panel view! Run again to see another random scene.\n');
