function export_for_sim(frameIndices, outDir)
% EXPORT_FOR_SIM Processes video frames through DeepLabv3+ perception
% and exports road polygons & obstacle bounding boxes to JSON for sim replay.
%
% Usage:
%   export_for_sim              % Exports all available frames to public/replay
%   export_for_sim(15)          % Exports frame 15 only
%   export_for_sim(1:10)        % Exports frames 1 to 10
%   export_for_sim([], outDir)  % Custom output directory

    scriptDir = fileparts(mfilename('fullpath'));
    rootDir = fileparts(scriptDir);
    if isempty(rootDir), rootDir = pwd; end
    
    addpath(rootDir);
    addpath(fullfile(rootDir, 'training'));

    if nargin < 2 || isempty(outDir)
        outDir = fullfile(rootDir, 'public', 'replay');
    end

    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    % Locate video frames
    candidates = {
        fullfile(rootDir, 'Video_Project_1_frames'), ...
        fullfile(rootDir, 'public', 'replay', 'frames'), ...
        fullfile(rootDir, 'video project frames'), ...
        'C:\Users\nkcr2\Downloads\Video_Project_1_frames'
    };

    frameDir = '';
    for c = 1:numel(candidates)
        if exist(candidates{c}, 'dir')
            frameDir = candidates{c};
            break;
        end
    end

    if isempty(frameDir)
        error('Video_Project_1_frames folder not found.');
    end

    frameFiles = dir(fullfile(frameDir, '*.jpg'));
    if isempty(frameFiles), frameFiles = dir(fullfile(frameDir, '*.png')); end
    [~, sortIdx] = sort({frameFiles.name});
    frameFiles = frameFiles(sortIdx);

    totalFrames = numel(frameFiles);
    fprintf('Found %d frames in %s\n', totalFrames, frameDir);

    if nargin < 1 || isempty(frameIndices)
        frameIndices = 1:totalFrames;
    else
        frameIndices = intersect(frameIndices, 1:totalFrames);
    end

    % Pre-load models once for high-throughput batch inference
    fprintf('Loading segmentation networks...\n');
    roadNet = [];
    roadNetPaths = {
        fullfile(rootDir, 'roadSegmentationNet.mat'), ...
        fullfile(rootDir, 'training', 'roadSegmentationNet.mat')
    };
    for p = 1:numel(roadNetPaths)
        if exist(roadNetPaths{p}, 'file')
            s = load(roadNetPaths{p});
            if isfield(s, 'trainedNet')
                roadNet = s.trainedNet;
            elseif isfield(s, 'net')
                roadNet = s.net;
            end
            fprintf('Loaded road segmentation network from %s\n', roadNetPaths{p});
            break;
        end
    end

    obsNet = [];
    obsNetClassNames = [];
    obsNetPaths = {
        fullfile(rootDir, 'obstacleSegmentationNet.mat'), ...
        fullfile(rootDir, 'training', 'obstacleSegmentationNet.mat')
    };
    for p = 1:numel(obsNetPaths)
        if exist(obsNetPaths{p}, 'file')
            s = load(obsNetPaths{p});
            if isfield(s, 'trainedNet')
                obsNet = s.trainedNet;
                obsNetClassNames = s.classNames;
            end
            fprintf('Loaded obstacle segmentation network from %s\n', obsNetPaths{p});
            break;
        end
    end

    exportedFiles = {};

    % Process frames
    for idx = frameIndices
        fname = frameFiles(idx).name;
        fpath = fullfile(frameDir, fname);
        img = imread(fpath);
        [H, W, ~] = size(img);

        fprintf('Processing frame %d/%d (%s)... ', idx, totalFrames, fname);

        % 1. Extract Road Polygon(s)
        roadMask = false(H, W);
        if ~isempty(roadNet)
            try
                inputSize = [227, 320];
                if isprop(roadNet, 'Layers') && isa(roadNet.Layers(1), 'nnet.cnn.layer.ImageInputLayer')
                    inputSize = roadNet.Layers(1).InputSize(1:2);
                end
                scaled = imresize(img, inputSize);
                pred = semanticseg(scaled, roadNet, 'Classes', ["notRoad", "road"]);
                predFull = imresize(pred == "road", [H, W], 'nearest');
                roadMask = bwareaopen(predFull, max(300, round(H * W * 0.005)));
                roadMask = imfill(roadMask, 'holes');
            catch ME
                warning('Road segmentation failed on frame %d: %s', idx, ME.message);
            end
        end

        % Fallback if deep learning road mask is empty
        if ~any(roadMask(:))
            hsv = rgb2hsv(img);
            roadMask = (hsv(:,:,2) < 0.25) & (hsv(:,:,3) > 0.15) & (hsv(:,:,3) < 0.75);
            roadMask = imclose(roadMask, strel('disk', 7));
            roadMask = imfill(roadMask, 'holes');
            roadMask = bwareaopen(roadMask, 500);
        end

        % Extract boundaries of largest drivable road area
        roadMaskClean = bwareafilt(logical(roadMask), 1); % Keep dominant road component
        if ~any(roadMaskClean(:))
            roadMaskClean = roadMask;
        end

        B = bwboundaries(roadMaskClean, 'noholes');
        roadPolys = {};
        for b = 1:numel(B)
            poly = B{b}; % [row, col] -> [y, x]
            polyXY = [poly(:, 2), poly(:, 1)]; % [x, y]

            % Simplify polygon to <= 60 points
            simplified = simplifyPolygon(polyXY, 60);
            if size(simplified, 1) >= 3
                roadPolys{end+1} = simplified; %#ok<AGROW>
            end
        end

        % Format road polygon: if 1 polygon, export as [N x 2] points array
        if numel(roadPolys) == 1
            roadData = roadPolys{1};
        elseif numel(roadPolys) > 1
            roadData = roadPolys;
        else
            % Default fallback corridor if segmentation found nothing
            roadData = [
                round(W*0.1), H;
                round(W*0.2), round(H*0.7);
                round(W*0.4), round(H*0.4);
                round(W*0.6), round(H*0.4);
                round(W*0.8), round(H*0.7);
                round(W*0.9), H
            ];
        end

        % 2. Extract Obstacles
        obstacles = [];
        if ~isempty(obsNet) && ~isempty(obsNetClassNames)
            try
                targetSize = [227, 320];
                scaledObs = imresize(img, targetSize);
                predObs = semanticseg(scaledObs, obsNet, 'Classes', obsNetClassNames);
                predObsFull = imresize(predObs, [H, W], 'nearest');

                validObstacles = ["pedestrian", "vehicle", "road_obstacle", "two_wheeler", "auto_rickshaw"];
                targetClasses = intersect(string(obsNetClassNames), validObstacles, 'stable');
                minArea = max(50, round(H * W * 0.0018));

                for c = 1:numel(targetClasses)
                    cName = targetClasses(c);
                    cBinary = (predObsFull == cName);
                    cBinary = bwareaopen(cBinary, minArea);
                    stats = regionprops(cBinary, 'BoundingBox', 'Area', 'Centroid');

                    for k = 1:numel(stats)
                        bb = stats(k).BoundingBox; % [x, y, w, h]
                        cent = stats(k).Centroid;

                        % Reject horizon / sky noise (upper 32%)
                        if cent(2) < (H * 0.32), continue; end
                        % Reject massive full-screen artifacts
                        if bb(3) > (W * 0.50) || bb(4) > (H * 0.50), continue; end
                        if stats(k).Area < minArea, continue; end

                        mappedCls = mapToObstacleClass(cName);
                        x1 = max(0, round(bb(1)));
                        y1 = max(0, round(bb(2)));
                        x2 = min(W, round(bb(1) + bb(3)));
                        y2 = min(H, round(bb(2) + bb(4)));

                        obsItem = struct('cls', char(mappedCls), 'bbox', [x1, y1, x2, y2]);
                        if isempty(obstacles)
                            obstacles = obsItem;
                        else
                            obstacles(end+1) = obsItem; %#ok<AGROW>
                        end
                    end
                end
            catch ME
                warning('Obstacle detection failed on frame %d: %s', idx, ME.message);
            end
        end

        % Build JSON struct
        frameStruct = struct();
        frameStruct.frame = idx;
        frameStruct.timestamp = round((idx - 1) * (1 / 15), 3);
        frameStruct.image = sprintf('frames/%s', fname);
        frameStruct.road = roadData;
        if isempty(obstacles)
            frameStruct.obstacles = cell(0, 1);
        else
            frameStruct.obstacles = obstacles;
        end

        outJsonName = sprintf('frame_%03d.json', idx);
        outJsonPath = fullfile(outDir, outJsonName);
        jsonStr = jsonencode(frameStruct, 'PrettyPrint', true);

        fid = fopen(outJsonPath, 'w');
        if fid ~= -1
            fwrite(fid, jsonStr, 'char');
            fclose(fid);
            exportedFiles{end+1} = outJsonName; %#ok<AGROW>
            fprintf('Saved %s (obs: %d)\n', outJsonName, numel(obstacles));
        else
            fprintf('Failed to write %s\n', outJsonPath);
        end
    end

    % 3. Save index.json
    indexStruct = struct();
    indexStruct.fps = 15;
    indexStruct.width = 1920;
    indexStruct.height = 1080;
    indexStruct.camera = struct(...
        'height', 1.4, ...
        'horizonRatio', 0.32, ...
        'fx', 1000, ...
        'fy', 1000 ...
    );
    indexStruct.totalFrames = numel(exportedFiles);
    indexStruct.frames = exportedFiles;

    indexJsonPath = fullfile(outDir, 'index.json');
    fid = fopen(indexJsonPath, 'w');
    if fid ~= -1
        fwrite(fid, jsonencode(indexStruct, 'PrettyPrint', true), 'char');
        fclose(fid);
        fprintf('Updated %s successfully.\n', indexJsonPath);
    end
end

% =========================================================================
% HELPER: Map detected label to strict ObstacleClass
% 'pedestrian'|'two_wheeler'|'auto'|'car'|'truck'|'cow'|'debris'
% =========================================================================
function mapped = mapToObstacleClass(labelStr)
    lbl = lower(char(labelStr));
    switch lbl
        case {'pedestrian', 'person', 'walker', 'people'}
            mapped = 'pedestrian';
        case {'two_wheeler', 'bicycle', 'motorcycle', 'bike', 'scooter'}
            mapped = 'two_wheeler';
        case {'auto_rickshaw', 'auto', 'rickshaw', 'autorickshaw', 'tuk_tuk'}
            mapped = 'auto';
        case {'vehicle', 'car', 'sedan', 'suv', 'jeep', 'van'}
            mapped = 'car';
        case {'truck', 'bus', 'lorry', 'heavy_vehicle'}
            mapped = 'truck';
        case {'cow', 'cattle', 'animal', 'dog'}
            mapped = 'cow';
        otherwise
            mapped = 'debris';
    end
end

% =========================================================================
% HELPER: Simplify polygon to <= maxPoints
% =========================================================================
function outPts = simplifyPolygon(inPts, maxPoints)
    if size(inPts, 1) <= maxPoints
        outPts = inPts;
        return;
    end

    % Try reducepoly first
    try
        tol = 0.005;
        p = reducepoly(inPts, tol);
        while size(p, 1) > maxPoints && tol < 0.1
            tol = tol * 1.5;
            p = reducepoly(inPts, tol);
        end
        if size(p, 1) <= maxPoints && size(p, 1) >= 3
            outPts = p;
            return;
        end
    catch
    end

    % Equidistant index decimation fallback
    N = size(inPts, 1);
    idx = round(linspace(1, N, maxPoints));
    outPts = inPts(idx, :);
end
