%% train_obstacleSegmentationNet.m
% GPU Training Script for Autonomous Obstacle Segmentation & Detection
% Target Classes: Pedestrians, Two-Wheelers, Auto-Rickshaws, Vehicles, Road Obstacles
% Network: DeepLabv3+ with ResNet-18 Backbone
% Dataset: IDD-Lite (1,403 Train, 204 Val)
% Hardware: NVIDIA GeForce RTX 5050 Laptop GPU (8.5 GB VRAM)

clc; clear; close all;

fprintf('=======================================================\n');
fprintf('  GPU TRAINING: INDIAN ROAD OBSTACLE DETECTION NET     \n');
fprintf('=======================================================\n');

%% 1. Verify GPU Availability
gpu = gpuDevice;
fprintf('Active GPU: %s (%.2f GB VRAM Available)\n', gpu.Name, gpu.TotalMemory / 1e9);

%% 2. Configure Paths & Data
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end

dataRoot = fullfile(scriptDir, 'training', 'idd20k_lite');
if ~exist(dataRoot, 'dir')
    dataRoot = fullfile(scriptDir, 'idd-lite', 'idd20k_lite');
end

imgDirTrain = fullfile(dataRoot, 'leftImg8bit', 'train');
lblDirTrain = fullfile(dataRoot, 'obstacle_labels', 'train');

imgDirVal = fullfile(dataRoot, 'leftImg8bit', 'val');
lblDirVal = fullfile(dataRoot, 'obstacle_labels', 'val');

classNames = {'background', 'pedestrian', 'vehicle', 'road_obstacle'};
pixelLabelIDs = [0, 1, 2, 3];
numClasses = numel(classNames);

fprintf('\nLoading Training & Validation Datastores...\n');
imdsTrain = imageDatastore(imgDirTrain, 'IncludeSubfolders', true, 'FileExtensions', '.jpg');
pxdsTrain = pixelLabelDatastore(lblDirTrain, classNames, pixelLabelIDs, 'IncludeSubfolders', true, 'FileExtensions', '.png');

imdsVal = imageDatastore(imgDirVal, 'IncludeSubfolders', true, 'FileExtensions', '.jpg');
pxdsVal = pixelLabelDatastore(lblDirVal, classNames, pixelLabelIDs, 'IncludeSubfolders', true, 'FileExtensions', '.png');

fprintf('Training samples:   %d pairs\n', numel(imdsTrain.Files));
fprintf('Validation samples: %d pairs\n', numel(imdsVal.Files));

%% 3. Calculate Class Weights for Balanced Multi-Class Learning
fprintf('Computing class pixel distribution...\n');
tbl = countEachLabel(pxdsTrain);
disp(tbl);
totalPixels = sum(tbl.PixelCount);
freq = tbl.PixelCount / totalPixels;
% Balanced inverse-frequency weighting
classWeights = median(freq) ./ (freq + 1e-4);
classWeights = classWeights / min(classWeights);
% Cap maximum weight to 10 for training stability
classWeights = min(classWeights, 10.0);
fprintf('\nCalculated Class Weights:\n');
for c = 1:numClasses
    fprintf('  %-15s : %.2f\n', classNames{c}, classWeights(c));
end

%% 4. Data Augmentation
inputSize = [227, 320, 3];
augmenter = imageDataAugmenter('RandXReflection', true);

warning('off', 'vision:pixelLabelImageDatastore:willBeRemoved');
pximdsTrain = pixelLabelImageDatastore(imdsTrain, pxdsTrain, ...
    'DataAugmentation', augmenter, ...
    'OutputSize', inputSize);

pximdsVal = pixelLabelImageDatastore(imdsVal, pxdsVal, ...
    'OutputSize', inputSize);

%% 5. Construct DeepLabv3+ Architecture
fprintf('\nConstructing DeepLabv3+ network with ResNet-18 backbone...\n');
net = deeplabv3plus(inputSize, numClasses, 'resnet18');

%% 6. Training Options for RTX 5050 GPU
batchSize = 16;
numEpochs = 15;
initialLR = 2e-4;

options = trainingOptions('adam', ...
    'InitialLearnRate', initialLR, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropPeriod', 6, ...
    'LearnRateDropFactor', 0.5, ...
    'MaxEpochs', numEpochs, ...
    'MiniBatchSize', batchSize, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', pximdsVal, ...
    'ValidationFrequency', 40, ...
    'GradientThreshold', 5, ...
    'Verbose', true, ...
    'VerboseFrequency', 20, ...
    'Plots', 'none');

%% 7. Execute GPU Training
fprintf('\n>>> Starting GPU Training on %s...\n', gpu.Name);
fprintf('>>> Batch Size: %d | Epochs: %d | Classes: %d\n\n', batchSize, numEpochs, numClasses);

tTrainStart = tic;
[trainedNet, trainInfo] = trainnet(pximdsTrain, net, 'crossentropy', options);
trainDuration = toc(tTrainStart);

fprintf('\n=======================================================\n');
fprintf('  TRAINING COMPLETED in %.2f minutes (%.1f s)\n', trainDuration/60, trainDuration);
fprintf('=======================================================\n');

%% 8. Save Trained Obstacle Model
saveTargets = {
    fullfile(scriptDir, 'obstacleSegmentationNet.mat'), ...
    fullfile(scriptDir, 'training', 'obstacleSegmentationNet.mat')
};

for k = 1:numel(saveTargets)
    saveDir = fileparts(saveTargets{k});
    if ~exist(saveDir, 'dir'), mkdir(saveDir); end
    save(saveTargets{k}, 'trainedNet', 'classNames', 'inputSize', 'classWeights', 'trainInfo', '-v7.3');
    fprintf('Exported trained model -> %s\n', saveTargets{k});
end

fprintf('\nIndian Road Obstacle Model ready for autonomous inference!\n');
