%% train_roadSegmentationNet.m
% GPU Training Script for Autonomous Road Semantic Segmentation
% Network: DeepLabv3+ with ResNet-18 Backbone
% Dataset: IDD-Lite (1,403 Train, 204 Val)
% Hardware: NVIDIA GeForce RTX 5050 Laptop GPU (8.5 GB VRAM)

clc; clear; close all;

fprintf('=======================================================\n');
fprintf('  DEEP LEARNING GPU TRAINING: ROAD SEGMENTATION NET     \n');
fprintf('=======================================================\n');

%% 1. Verify GPU Availability
gpu = gpuDevice;
fprintf('Active GPU: %s (%.2f GB VRAM Available)\n', gpu.Name, gpu.TotalMemory / 1e9);

%% 2. Configure Paths & Data
scriptDir = fileparts(mfilename('fullpath'));
if exist(fullfile(scriptDir, 'idd20k_lite'), 'dir')
    dataRoot = fullfile(scriptDir, 'idd20k_lite');
elseif exist(fullfile(scriptDir, 'training', 'idd20k_lite'), 'dir')
    dataRoot = fullfile(scriptDir, 'training', 'idd20k_lite');
else
    dataRoot = 'E:\SIH\training\idd20k_lite';
end

imgDirTrain = fullfile(dataRoot, 'leftImg8bit', 'train');
lblDirTrain = fullfile(dataRoot, 'binary_labels', 'train');

imgDirVal = fullfile(dataRoot, 'leftImg8bit', 'val');
lblDirVal = fullfile(dataRoot, 'binary_labels', 'val');

classNames = {'notRoad', 'road'};
pixelLabelIDs = [0, 1];

fprintf('\nLoading Training & Validation Datastores...\n');
imdsTrain = imageDatastore(imgDirTrain, 'IncludeSubfolders', true, 'FileExtensions', '.jpg');
pxdsTrain = pixelLabelDatastore(lblDirTrain, classNames, pixelLabelIDs, 'IncludeSubfolders', true, 'FileExtensions', '.png');

imdsVal = imageDatastore(imgDirVal, 'IncludeSubfolders', true, 'FileExtensions', '.jpg');
pxdsVal = pixelLabelDatastore(lblDirVal, classNames, pixelLabelIDs, 'IncludeSubfolders', true, 'FileExtensions', '.png');

fprintf('Training samples:   %d pairs\n', numel(imdsTrain.Files));
fprintf('Validation samples: %d pairs\n', numel(imdsVal.Files));

%% 3. Calculate Class Weights for Balanced Learning
tbl = countEachLabel(pxdsTrain);
totalPixels = sum(tbl.PixelCount);
freq = tbl.PixelCount / totalPixels;
classWeights = median(freq) ./ freq;
classWeights = classWeights / min(classWeights);
fprintf('\nClass Weighting (Road vs NotRoad): [%.2f, %.2f]\n', classWeights(1), classWeights(2));

%% 4. Data Augmentation
inputSize = [227, 320, 3];
augmenter = imageDataAugmenter(...
    'RandXReflection', true);

warning('off', 'vision:pixelLabelImageDatastore:willBeRemoved');

pximdsTrain = pixelLabelImageDatastore(imdsTrain, pxdsTrain, ...
    'DataAugmentation', augmenter, ...
    'OutputSize', inputSize(1:2));

pximdsVal = pixelLabelImageDatastore(imdsVal, pxdsVal, ...
    'OutputSize', inputSize(1:2));

%% 5. Build DeepLabv3+ Architecture (ResNet-18 Backbone)
fprintf('\nConstructing DeepLabv3+ with ResNet-18 backbone...\n');
numClasses = numel(classNames);
net = deeplabv3plus(inputSize, numClasses, 'resnet18');

%% 6. Training Options for RTX 5050 GPU
miniBatchSize = 16;
numEpochs = 15;

options = trainingOptions('adam', ...
    'InitialLearnRate', 2e-4, ...
    'GradientThreshold', 5, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropPeriod', 6, ...
    'LearnRateDropFactor', 0.3, ...
    'L2Regularization', 1e-4, ...
    'MaxEpochs', numEpochs, ...
    'MiniBatchSize', miniBatchSize, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', pximdsVal, ...
    'ValidationFrequency', 45, ...
    'ValidationPatience', 4, ...
    'Verbose', true, ...
    'VerboseFrequency', 15, ...
    'Plots', 'training-progress', ...
    'ExecutionEnvironment', 'gpu');

%% 7. Kick Off Training
fprintf('\n>>> Starting GPU Training on %s...\n', gpu.Name);
fprintf('>>> Batch Size: %d | Epochs: %d\n', miniBatchSize, numEpochs);

tic;
[trainedNet, trainInfo] = trainnet(pximdsTrain, net, 'crossentropy', options);
trainDuration = toc;

fprintf('\n>>> Training completed in %.2f minutes!\n', trainDuration / 60);

%% 8. Save Model
modelSavePath = fullfile(dataRoot, 'roadSegmentationNet.mat');
rootSavePath  = fullfile(fileparts(dataRoot), 'roadSegmentationNet.mat');

net = trainedNet; %#ok<NASGU>
save(modelSavePath, 'net', '-v7.3');
save(rootSavePath, 'net', '-v7.3');
fprintf('Saved trained model to:\n  - %s\n  - %s\n', modelSavePath, rootSavePath);

%% 9. Evaluate on Validation Set
fprintf('\nEvaluating segmentation metrics on validation set...\n');
valWriteDir = fullfile(tempdir, 'val_eval_results');
if ~exist(valWriteDir, 'dir')
    mkdir(valWriteDir);
end

pxdsResults = semanticseg(imdsVal, trainedNet, ...
    'MiniBatchSize', miniBatchSize, ...
    'WriteLocation', valWriteDir, ...
    'Classes', classNames, ...
    'ExecutionEnvironment', 'gpu');

metrics = evaluateSemanticSegmentation(pxdsResults, pxdsVal, 'Verbose', false);
disp(metrics.DataSetMetrics);
disp(metrics.ClassMetrics);

fprintf('=======================================================\n');
fprintf('  GPU TRAINING & VALIDATION COMPLETED SUCCESSFULLY!    \n');
fprintf('=======================================================\n');
