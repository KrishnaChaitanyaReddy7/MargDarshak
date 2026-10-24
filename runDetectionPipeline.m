function [drivableMask, bboxes, labels, obstacleMask] = runDetectionPipeline(imageFile, showPlots)
% RUNDETECTIONPIPELINE Deep Learning Indian Road Obstacle Detection
% Trained on IDD-Lite with NVIDIA RTX 5050 GPU
% Detects: Pedestrians, Two-Wheelers, Auto-Rickshaws, Vehicles, Road Obstacles
% Note: Divider detection has been completely removed as requested.
%
% Inputs:
%   imageFile    - Image file path or RGB image matrix
%   showPlots    - (Optional) Boolean flag to show standalone figure (default: false)
%
% Outputs:
%   drivableMask - 2D binary mask (1 = clear space, 0 = obstacle)
%   bboxes       - N-by-4 matrix of [x, y, w, h] bounding boxes
%   labels       - String array of detected class labels
%   obstacleMask - 2D logical mask of all detected obstacles (1 = obstacle, 0 = clear)

if nargin < 2
    showPlots = false;
end

%% 1. Load Image
if ischar(imageFile) || isstring(imageFile)
    I = imread(imageFile);
else
    I = imageFile;
    imageFile = 'Input Scene';
end

[H, W, ~] = size(I);

%% 2. Load Trained Indian Obstacle Segmentation Model
persistent netModel;
if isempty(netModel) || (isfield(netModel, 'classNames') && numel(netModel.classNames) ~= 4)
    modelPaths = {
        'obstacleSegmentationNet.mat', ...
        fullfile('training', 'obstacleSegmentationNet.mat'), ...
        fullfile(fileparts(mfilename('fullpath')), 'obstacleSegmentationNet.mat'), ...
        fullfile(fileparts(mfilename('fullpath')), 'training', 'obstacleSegmentationNet.mat')
    };
    for m = 1:numel(modelPaths)
        if exist(modelPaths{m}, 'file')
            loaded = load(modelPaths{m});
            netModel = loaded;
            break;
        end
    end
end

bboxes = [];
labels = string([]);
obstacleMask = false(H, W);

if ~isempty(netModel)
    % ---------------------------------------------------------------------
    % GPU INFERENCE: DeepLabv3+ Multi-Class Indian Obstacle Detector
    % ---------------------------------------------------------------------
    targetSize = [227, 320];
    if isfield(netModel, 'inputSize')
        targetSize = netModel.inputSize(1:2);
    end
    
    imgScaled = imresize(I, targetSize);
    
    try
        predicted = semanticseg(imgScaled, netModel.trainedNet, 'Classes', netModel.classNames);
        predFull = imresize(predicted, [H, W], 'nearest');
        
        % Supported obstacle classes from Indian Driving Dataset
        validObstacles = ["pedestrian", "vehicle", "road_obstacle", "two_wheeler", "auto_rickshaw"];
        targetClasses = intersect(string(netModel.classNames), validObstacles, 'stable');
        
        minArea = max(50, round(H * W * 0.0018)); % Adaptive noise suppression
        
        for c = 1:numel(targetClasses)
            clsName = targetClasses(c);
            clsBinary = (predFull == clsName);
            clsBinary = bwareaopen(clsBinary, minArea);
            
            % Extract bounding boxes with ADAS spatial constraints
            stats = regionprops(clsBinary, 'BoundingBox', 'Area', 'Centroid', 'PixelIdxList');
            for k = 1:numel(stats)
                bb = stats(k).BoundingBox;
                cent = stats(k).Centroid;
                
                % 1. Horizon & Sky filter: Reject objects in the upper sky (upper 32% of frame)
                if cent(2) < (H * 0.32), continue; end
                
                % 2. Scale filter: Reject massive background/terrain regions (> 50% image dimension)
                if bb(3) > (W * 0.50) || bb(4) > (H * 0.50), continue; end
                
                if stats(k).Area >= minArea
                    bboxes = [bboxes; bb]; %#ok<AGROW>
                    labels = [labels; clsName]; %#ok<AGROW>
                    
                    % Accumulate only validated obstacle pixels
                    obstacleMask(stats(k).PixelIdxList) = true;
                end
            end
        end
    catch ME
        warning('runDetectionPipeline:InferenceError', ...
            'Deep learning obstacle inference failed: %s.', ME.message);
    end
else
    %% Fallback: Built-in ACF Detectors ONLY if deep learning model is missing
    try
        vDet = vehicleDetectorACF('front-rear-view');
        [vBoxes, vScores] = detect(vDet, I);
        if ~isempty(vBoxes)
            keep = vScores > 15;
            for k = 1:size(vBoxes, 1)
                if keep(k)
                    bboxes = [bboxes; vBoxes(k, :)]; %#ok<AGROW>
                    labels = [labels; "vehicle"]; %#ok<AGROW>
                    bx = max(1, round(vBoxes(k, 1)));
                    by = max(1, round(vBoxes(k, 2)));
                    bw = round(vBoxes(k, 3));
                    bh = round(vBoxes(k, 4));
                    obstacleMask(by:min(H, by+bh), bx:min(W, bx+bw)) = true;
                end
            end
        end
    catch
    end
    
    try
        pDet = peopleDetectorACF();
        [pBoxes, pScores] = detect(pDet, I);
        if ~isempty(pBoxes)
            keep = pScores > 0.8;
            for k = 1:size(pBoxes, 1)
                if keep(k)
                    bboxes = [bboxes; pBoxes(k, :)]; %#ok<AGROW>
                    labels = [labels; "pedestrian"]; %#ok<AGROW>
                    bx = max(1, round(pBoxes(k, 1)));
                    by = max(1, round(pBoxes(k, 2)));
                    bw = round(pBoxes(k, 3));
                    bh = round(pBoxes(k, 4));
                    obstacleMask(by:min(H, by+bh), bx:min(W, bx+bw)) = true;
                end
            end
        end
    catch
    end
end

%% 3. Construct Drivable Obstacle-Free Mask
drivableMask = double(~obstacleMask);

%% 4. Display Detections if Requested
if showPlots
    figure('Name', sprintf('Obstacle Detection: %s', string(imageFile)));
    if ~isempty(bboxes)
        annotated = insertObjectAnnotation(I, 'rectangle', bboxes, labels, ...
            'Color', 'cyan', 'TextColor', 'black', 'LineWidth', 2);
    else
        annotated = I;
    end
    imshow(annotated);
    title(sprintf('Indian Road Obstacles Detected: %d', size(bboxes, 1)));
end
end
