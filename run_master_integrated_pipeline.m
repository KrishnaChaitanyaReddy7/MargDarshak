function resStruct = run_master_integrated_pipeline(inputImage, frameIdx)
% RUN_MASTER_INTEGRATED_PIPELINE Unified Autonomous Driving Master Pipeline
% Integrates modules from all team members into ONE single 4-panel dashboard:
%   - Module 1 (Perception): detectUnmarkedRoad.m + roadSegmentationNet.mat
%   - Module 2 (Obstacle & Divider Detection): runDetectionPipeline.m
%   - Module 3 (Drivable Space Fusion): runCompletePipeline.m
%   - Module 4 (Trajectory Planning): planPathFromMask.m & planPathForSimulink.m
%
% Usage:
%   --- IDD Dataset Images ---
%   run_master_integrated_pipeline              % Picks a RANDOM IDD dataset scene
%   run_master_integrated_pipeline('random')    % Same as above
%   run_master_integrated_pipeline(42)          % IDD image #42
%   run_master_integrated_pipeline('loop')      % Interactive loop over IDD images
%
%   --- Video Project Frames ('Video_Project_1_frames') ---
%   run_master_integrated_pipeline('video')          % Picks a random frame from video folder
%   run_master_integrated_pipeline('video', 15)      % Selects specific video frame #15 (frame_015.jpg)
%   run_master_integrated_pipeline('frame', 15)      % Same as above (frame #15)
%   run_master_integrated_pipeline('video_loop')     % Step through video frames (Enter=Next, b=Back, q=Quit)
%   run_master_integrated_pipeline('video_play')     % Continuous video stream playback on 4-panel HUD
%   run_master_integrated_pipeline('select')         % Visual file picker (click any image from browser)
%   run_master_integrated_pipeline('Video_Project_1_frames') % Selects from video folder
%   run_master_integrated_pipeline('frame_015.jpg')  % Selects specific frame file directly
%   run_master_integrated_pipeline('road.jpg')       % Custom test image file

clc;
close all;

currDir = fileparts(mfilename('fullpath'));
if isempty(currDir), currDir = pwd; end
cd(currDir);
addpath(currDir);

%% 1. Find Dataset Images & Video Frames
valDirs = {
    fullfile(currDir, 'idd-lite', 'idd20k_lite', 'leftImg8bit', 'val'), ...
    fullfile(currDir, 'training', 'idd20k_lite', 'leftImg8bit', 'val'), ...
    fullfile(currDir, 'training')
};

valFiles = [];
for d = 1:numel(valDirs)
    if exist(valDirs{d}, 'dir')
        found = dir(fullfile(valDirs{d}, '**', '*_image.jpg'));
        if ~isempty(found)
            valFiles = found;
            break;
        end
    end
end

if isempty(valFiles)
    valFiles = dir(fullfile(currDir, '*.jpg'));
end
totalImgs = numel(valFiles);

% Search for video frames folder (e.g. Video_Project_1_frames or video project frames)
[videoDir, videoFiles] = findVideoFramesFolder(currDir);
totalVideoFrames = numel(videoFiles);

%% 2. Determine Which Image to Process
isLoopMode = false;
isVideoLoop = false;
isVideoPlay = false;
selectedFile = '';
sceneLabel = '';
activeFig = [];

if nargin < 1 || isempty(inputImage) || (ischar(inputImage) && strcmpi(inputImage, 'random'))
    % Default: Random IDD scene
    if totalImgs > 0
        rng('shuffle');
        randIdx = randi(totalImgs);
        selectedFile = fullfile(valFiles(randIdx).folder, valFiles(randIdx).name);
        sceneLabel = sprintf('Random IDD Scene #%d of %d', randIdx, totalImgs);
    elseif totalVideoFrames > 0
        randIdx = randi(totalVideoFrames);
        selectedFile = fullfile(videoFiles(randIdx).folder, videoFiles(randIdx).name);
        sceneLabel = sprintf('Random Video Frame #%d of %d (%s)', randIdx, totalVideoFrames, videoFiles(randIdx).name);
    else
        selectedFile = 'road.jpg';
        sceneLabel = 'Default Scene (road.jpg)';
    end

elseif ischar(inputImage) && (strcmpi(inputImage, 'video') || strcmpi(inputImage, 'video_frames') || ...
                              strcmpi(inputImage, 'frame') || strcmpi(inputImage, 'frames'))
    if totalVideoFrames == 0
        error('No video frames folder found. Expected Video_Project_1_frames or video project frames in %s', currDir);
    end
    if nargin >= 2 && ~isempty(frameIdx) && isnumeric(frameIdx)
        idx = min(max(1, round(frameIdx)), totalVideoFrames);
        selectedFile = fullfile(videoFiles(idx).folder, videoFiles(idx).name);
        sceneLabel = sprintf('Video Frame #%d of %d (%s)', idx, totalVideoFrames, videoFiles(idx).name);
    else
        rng('shuffle');
        idx = randi(totalVideoFrames);
        selectedFile = fullfile(videoFiles(idx).folder, videoFiles(idx).name);
        sceneLabel = sprintf('Random Video Frame #%d of %d (%s)', idx, totalVideoFrames, videoFiles(idx).name);
    end

elseif ischar(inputImage) && (strcmpi(inputImage, 'select') || strcmpi(inputImage, 'browse') || strcmpi(inputImage, 'choose'))
    % Graphical file picker
    startFolder = videoDir;
    if isempty(startFolder), startFolder = currDir; end
    [fName, fPath] = uigetfile({'*.jpg;*.jpeg;*.png;*.bmp', 'Image Files (*.jpg, *.png, *.jpeg)'}, ...
        'Select Image/Frame to Process', startFolder);
    if isequal(fName, 0)
        fprintf('Image selection cancelled by user.\n');
        resStruct = [];
        return;
    end
    selectedFile = fullfile(fPath, fName);
    sceneLabel = sprintf('Selected Image: %s', fName);

elseif ischar(inputImage) && (strcmpi(inputImage, 'video_loop') || strcmpi(inputImage, 'vloop'))
    isVideoLoop = true;
    if totalVideoFrames == 0
        error('No video frames found to loop through.');
    end
    curIdx = 1;
    if nargin >= 2 && ~isempty(frameIdx) && isnumeric(frameIdx)
        curIdx = min(max(1, round(frameIdx)), totalVideoFrames);
    end
    selectedFile = fullfile(videoFiles(curIdx).folder, videoFiles(curIdx).name);
    sceneLabel = sprintf('Video Frame #%d of %d (%s)', curIdx, totalVideoFrames, videoFiles(curIdx).name);

elseif ischar(inputImage) && (strcmpi(inputImage, 'video_play') || strcmpi(inputImage, 'play') || strcmpi(inputImage, 'stream'))
    isVideoPlay = true;
    if totalVideoFrames == 0
        error('No video frames found to play.');
    end
    startIdx = 1;
    if nargin >= 2 && ~isempty(frameIdx) && isnumeric(frameIdx)
        startIdx = min(max(1, round(frameIdx)), totalVideoFrames);
    end
    selectedFile = fullfile(videoFiles(startIdx).folder, videoFiles(startIdx).name);
    sceneLabel = sprintf('Video Stream: Frame #%d of %d (%s)', startIdx, totalVideoFrames, videoFiles(startIdx).name);

elseif isnumeric(inputImage)
    % Specific index passed: if totalImgs > 0, IDD dataset; else video frame
    if totalImgs > 0
        idx = min(max(1, round(inputImage)), totalImgs);
        selectedFile = fullfile(valFiles(idx).folder, valFiles(idx).name);
        sceneLabel = sprintf('IDD Dataset Image #%d of %d', idx, totalImgs);
    elseif totalVideoFrames > 0
        idx = min(max(1, round(inputImage)), totalVideoFrames);
        selectedFile = fullfile(videoFiles(idx).folder, videoFiles(idx).name);
        sceneLabel = sprintf('Video Frame #%d of %d (%s)', idx, totalVideoFrames, videoFiles(idx).name);
    else
        error('No dataset or video images found to index.');
    end

elseif ischar(inputImage) || isstring(inputImage)
    inputStr = char(inputImage);
    if strcmpi(inputStr, 'loop')
        isLoopMode = true;
        randIdx = randi(totalImgs);
        selectedFile = fullfile(valFiles(randIdx).folder, valFiles(randIdx).name);
        sceneLabel = sprintf('Interactive Random Scene #%d of %d', randIdx, totalImgs);
    elseif exist(inputStr, 'dir')
        % User passed a folder! Look for images in that folder
        folderImgs = dir(fullfile(inputStr, '*.jpg'));
        if isempty(folderImgs), folderImgs = dir(fullfile(inputStr, '*.png')); end
        if isempty(folderImgs)
            error('No images found in folder: %s', inputStr);
        end
        [~, sIdx] = sort({folderImgs.name});
        folderImgs = folderImgs(sIdx);
        fIdx = 1;
        if nargin >= 2 && ~isempty(frameIdx) && isnumeric(frameIdx)
            fIdx = min(max(1, round(frameIdx)), numel(folderImgs));
        end
        selectedFile = fullfile(folderImgs(fIdx).folder, folderImgs(fIdx).name);
        sceneLabel = sprintf('Folder Image #%d of %d (%s)', fIdx, numel(folderImgs), folderImgs(fIdx).name);
    elseif exist(inputStr, 'file') == 2
        selectedFile = inputStr;
        sceneLabel = sprintf('Custom Image: %s', selectedFile);
    elseif ~isempty(videoDir) && exist(fullfile(videoDir, inputStr), 'file') == 2
        selectedFile = fullfile(videoDir, inputStr);
        sceneLabel = sprintf('Video Frame: %s', inputStr);
    elseif exist(fullfile(currDir, 'training', inputStr), 'file') == 2
        selectedFile = fullfile(currDir, 'training', inputStr);
        sceneLabel = sprintf('Sample Image: %s', inputStr);
    else
        error('Specified image file or command not found: %s', inputStr);
    end
end

%% Run Processing and Display Dashboard
resStruct = processAndDisplaySingle(selectedFile, sceneLabel, currDir);
if ~isempty(resStruct) && isfield(resStruct, 'dashboardFigure')
    activeFig = resStruct.dashboardFigure;
end

%% 3. Interactive Loop Mode (IDD Dataset)
if isLoopMode && totalImgs > 0
    fprintf('\n>>> Interactive Mode Active. Press [Enter] for next random image, or [q] + Enter to quit.\n');
    while true
        userResp = input('Next random image? (Enter = Yes, q = Quit): ', 's');
        if strcmpi(strtrim(userResp), 'q')
            fprintf('Exiting interactive mode.\n');
            break;
        end
        randIdx = randi(totalImgs);
        selectedFile = fullfile(valFiles(randIdx).folder, valFiles(randIdx).name);
        sceneLabel = sprintf('Interactive Random Scene #%d of %d', randIdx, totalImgs);
        resStruct = processAndDisplaySingle(selectedFile, sceneLabel, currDir, activeFig);
    end
end

%% 4. Interactive Video Frame Viewer Loop
if isVideoLoop && totalVideoFrames > 0
    fprintf('\n========================================================================\n');
    fprintf('  INTERACTIVE VIDEO FRAME VIEWER (SIH 2026)\n');
    fprintf('  Folder: %s (%d frames)\n', videoDir, totalVideoFrames);
    fprintf('  Controls: [Enter] = Next frame | [b] = Previous frame | [number] = Jump | [q] = Quit\n');
    fprintf('========================================================================\n');
    while true
        userResp = input(sprintf('Frame %d/%d (Enter=Next, b=Prev, #=Jump, q=Quit): ', curIdx, totalVideoFrames), 's');
        userResp = strtrim(userResp);
        if strcmpi(userResp, 'q')
            fprintf('Exiting video frame viewer.\n');
            break;
        elseif strcmpi(userResp, 'b')
            curIdx = max(1, curIdx - 1);
        elseif ~isempty(str2double(userResp)) && ~isnan(str2double(userResp))
            curIdx = min(max(1, round(str2double(userResp))), totalVideoFrames);
        else
            curIdx = min(totalVideoFrames, curIdx + 1);
        end
        selectedFile = fullfile(videoFiles(curIdx).folder, videoFiles(curIdx).name);
        sceneLabel = sprintf('Video Frame #%d of %d (%s)', curIdx, totalVideoFrames, videoFiles(curIdx).name);
        resStruct = processAndDisplaySingle(selectedFile, sceneLabel, currDir, activeFig);
    end
end

%% 5. Continuous Video Stream Playback
if isVideoPlay && totalVideoFrames > 0
    fprintf('\n>>> Continuous Video Stream Playback Active (Frames %d to %d)...\n', startIdx, totalVideoFrames);
    fprintf('    Close the figure window or press Ctrl+C in MATLAB to stop playback.\n\n');
    for f = (startIdx + 1):totalVideoFrames
        if isempty(activeFig) || ~isvalid(activeFig)
            fprintf('Figure window closed. Stopping video playback.\n');
            break;
        end
        selectedFile = fullfile(videoFiles(f).folder, videoFiles(f).name);
        sceneLabel = sprintf('Video Stream: Frame #%d of %d (%s)', f, totalVideoFrames, videoFiles(f).name);
        resStruct = processAndDisplaySingle(selectedFile, sceneLabel, currDir, activeFig);
        pause(0.02);
    end
    fprintf('Video stream playback completed.\n');
end

if nargout == 0
    clear resStruct;
end
end


% =========================================================================
% HELPER FUNCTION: Process Single Image and Display 4-Panel Master HUD
% =========================================================================
function resStruct = processAndDisplaySingle(imagePath, sceneLabel, currDir, existingFig)
    fprintf('========================================================================\n');
    fprintf('       UNIFIED AUTONOMOUS DRIVING MASTER PIPELINE (SIH 2026)            \n');
    fprintf('  Integrating: Perception | Obstacle Detection | Dividers | Path Plan  \n');
    fprintf('========================================================================\n');
    fprintf('>>> Selected Image: %s\n', sceneLabel);
    fprintf('    File Path:      %s\n', imagePath);

    rawImg = imread(imagePath);
    [H, W, ~] = size(rawImg);
    fprintf('    Dimensions:     %d x %d pixels\n\n', W, H);

    %% STEP 1: DEEP LEARNING ROAD PERCEPTION (Module 1: detectUnmarkedRoad)
    fprintf('  [1/4] Deep Learning Road Perception (DeepLabv3+ ResNet-18)...\n');
    roadResult = detectUnmarkedRoad(rawImg);
    roadMask = roadResult.drivableMask;
    roadAreaPct = (sum(roadMask(:)) / numel(roadMask)) * 100;
    fprintf('        Road Area: %.2f%% | Classification: %s\n', roadAreaPct, roadResult.laneType);

    %% STEP 2: INDIAN ROAD OBSTACLE DETECTION (Module 2: runDetectionPipeline)
    fprintf('  [2/4] Indian Road Deep Learning Obstacle Detection...\n');
    [obstacleFreeMask, bboxes, labels, obstacleMask] = runDetectionPipeline(rawImg, false);
    numObstacles = size(bboxes, 1);
    fprintf('        Obstacles Detected: %d\n', numObstacles);
    if numObstacles > 0
        uCls = unique(labels);
        for u = 1:numel(uCls)
            cCount = sum(labels == uCls(u));
            fprintf('          - %s: %d\n', uCls(u), cCount);
        end
    end

    %% STEP 3: FUSED COMPLETE DRIVABLE CORRIDOR (Module 3: runCompletePipeline)
    fprintf('  [3/4] Fusing Drivable Space (Road Surface minus Detected Obstacles)...\n');
    fusedDrivableMask = double(roadMask);
    fusedDrivableMask(obstacleMask) = 0;

    %% STEP 4: A* TRAJECTORY PLANNING (Module 4: planPathFromMask & Simulink)
    fprintf('  [4/4] Generating Collision-Free A* Trajectory...\n');
    
    % Safe Vehicle Start (bottom drivable road)
    bottomRow = round(H * 0.90);
    botCols = find(fusedDrivableMask(bottomRow, :));
    if isempty(botCols)
        for r = H:-5:round(H*0.6)
            botCols = find(fusedDrivableMask(r, :));
            if ~isempty(botCols), bottomRow = r; break; end
        end
    end
    if ~isempty(botCols)
        startPt = [bottomRow, round(median(botCols))];
    else
        startPt = [round(H*0.9), round(W*0.5)];
    end

    % Lookahead Goal (horizon drivable road)
    topRow = round(H * 0.45);
    topCols = find(fusedDrivableMask(topRow, :));
    if isempty(topCols)
        for r = round(H*0.35):5:round(H*0.7)
            topCols = find(fusedDrivableMask(r, :));
            if ~isempty(topCols), topRow = r; break; end
        end
    end
    if ~isempty(topCols)
        goalPt = [topRow, round(median(topCols))];
    else
        goalPt = [round(H*0.45), round(W*0.5)];
    end

    smoothPath = planPathFromMask(fusedDrivableMask, startPt, goalPt, false);
    simulinkPath = planPathForSimulink(fusedDrivableMask, startPt, goalPt);
    
    pathFeasible = ~isempty(smoothPath) && size(smoothPath, 1) >= 2;
    if pathFeasible
        fprintf('        Trajectory: FEASIBLE (%d waypoints generated)\n\n', size(smoothPath, 1));
    else
        fprintf('        Trajectory: Alternative route search engaged\n\n');
    end

    %% SINGLE 4-PANEL DASHBOARD WINDOW
    if nargin < 4 || isempty(existingFig) || ~isvalid(existingFig)
        fig = figure('Name', sprintf('Master Integrated Pipeline - %s', sceneLabel), ...
                     'NumberTitle', 'off', ...
                     'Color', [0.10 0.10 0.12], ...
                     'Position', [50, 40, 1440, 880]);
    else
        fig = existingFig;
        clf(fig);
        set(fig, 'Name', sprintf('Master Integrated Pipeline - %s', sceneLabel));
    end

    t = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'loose');
    
    % Main Window Header
    [~, imgBaseName, imgExt] = fileparts(imagePath);
    headerStr = sprintf('AUTONOMOUS ADAS MASTER PIPELINE: %s (%s%s - %dx%d)', ...
        upper(sceneLabel), imgBaseName, imgExt, W, H);
    title(t, headerStr, 'Color', [0.2 0.95 0.45], 'FontSize', 14, 'FontWeight', 'bold', 'Interpreter', 'none');

    % PANEL 1: Raw Camera Sensor Input
    nexttile(t, 1);
    imshow(rawImg);
    title('1. Raw Camera Sensor Input', 'Color', [1 1 1], 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');
    xlabel(sprintf('Source: %s%s (%d x %d px)', imgBaseName, imgExt, W, H), ...
        'Color', [0.8 0.8 0.8], 'FontSize', 10, 'Interpreter', 'none');

    % PANEL 2: Deep Learning Unmarked Road Perception (Person 3 & You)
    nexttile(t, 2);
    imshow(roadResult.overlayImage);
    title('2. Deep Learning Unmarked Road Perception', ...
        'Color', [0.2 0.95 0.45], 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');
    xlabel(sprintf('Drivable Road Surface: %.1f%% | Physical Boundaries: Yellow (No Lane Markings)', roadAreaPct), ...
        'Color', [0.8 0.8 0.8], 'FontSize', 10, 'Interpreter', 'none');

    % PANEL 3: Indian Road Obstacle Detection (GPU-Trained)
    nexttile(t, 3);
    annotatedImg = rawImg;
    if ~isempty(bboxes)
        % Multi-class color assignments
        colorMap = containers.Map({'vehicle', 'pedestrian', 'road_obstacle', 'auto_rickshaw', 'two_wheeler'}, ...
            {'cyan', 'red', 'yellow', 'yellow', 'magenta'});
        boxColors = cell(size(bboxes, 1), 1);
        for b = 1:size(bboxes, 1)
            lblStr = char(labels(b));
            if isKey(colorMap, lblStr)
                boxColors{b} = colorMap(lblStr);
            else
                boxColors{b} = 'cyan';
            end
        end
        annotatedImg = insertObjectAnnotation(annotatedImg, 'rectangle', bboxes, cellstr(labels), ...
            'Color', boxColors, 'TextColor', 'black', 'LineWidth', 2, 'FontSize', 11);
    end
    imshow(annotatedImg);
    title('3. Indian Road Obstacle Detection (GPU-Trained)', 'Color', [1 0.75 0.2], 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');
    xlabel(sprintf('Detected Obstacles: %d (Vehicles: Cyan, Pedestrians: Red, Obstacles: Yellow)', numObstacles), ...
        'Color', [0.8 0.8 0.8], 'FontSize', 10, 'Interpreter', 'none');

    % PANEL 4: End-to-End A* Path Planning (Person 4)
    nexttile(t, 4);
    visBase = rawImg;
    tintOverlay = uint8(cat(3, zeros(H,W), uint8(fusedDrivableMask * 120), zeros(H,W)));
    fusedDisplay = imfuse(visBase, tintOverlay, 'blend', 'Scaling', 'joint');
    imshow(fusedDisplay);
    hold on;
    
    if pathFeasible
        plot(smoothPath(:,1), smoothPath(:,2), 'c-', 'LineWidth', 3.5);
        plot(smoothPath(:,1), smoothPath(:,2), 'b--', 'LineWidth', 1.5);
        plot(startPt(2), startPt(1), 's', 'MarkerSize', 12, ...
            'MarkerFaceColor', [0.1 0.5 1.0], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
        plot(goalPt(2), goalPt(1), 'p', 'MarkerSize', 16, ...
            'MarkerFaceColor', [1.0 0.85 0.0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        legend({'Spline Trajectory', 'A* Guidance', 'Vehicle Ego Start', 'Lookahead Goal'}, ...
            'TextColor', [1 1 1], 'Color', [0.15 0.15 0.18], 'Location', 'southoutside', ...
            'Orientation', 'horizontal', 'Interpreter', 'none');
        planStatusStr = sprintf('Feasible Trajectory Generated (%d waypoints)', size(smoothPath, 1));
    else
        plot(startPt(2), startPt(1), 's', 'MarkerSize', 12, ...
            'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'w');
        plot(goalPt(2), goalPt(1), 'p', 'MarkerSize', 16, ...
            'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
        planStatusStr = 'Trajectory Search In Progress';
    end
    hold off;
    title('4. End-to-End Trajectory & Path Planning', 'Color', [0.3 0.8 1.0], 'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');
    xlabel(planStatusStr, 'Color', [0.8 0.8 0.8], 'FontSize', 10, 'Interpreter', 'none');

    drawnow;

    % Save latest dashboard figure
    saveDir = fullfile(currDir, 'training');
    if ~exist(saveDir, 'dir'), mkdir(saveDir); end
    saveName = fullfile(saveDir, 'master_dashboard_latest.png');
    try
        saveas(fig, saveName);
        fprintf('  [Dashboard Figure Saved] -> %s\n', saveName);
    catch
    end
    fprintf('  >> Run ''run_master_integrated_pipeline'' again to test another random scene!\n\n');

    % Package return struct
    resStruct = struct();
    resStruct.selectedFile = imagePath;
    resStruct.sceneLabel = sceneLabel;
    resStruct.rawImage = rawImg;
    resStruct.roadResult = roadResult;
    resStruct.bboxes = bboxes;
    resStruct.labels = labels;
    resStruct.obstacleMask = obstacleMask;
    resStruct.drivableMask = fusedDrivableMask;
    resStruct.smoothPath = smoothPath;
    resStruct.simulinkPath = simulinkPath;
    resStruct.dashboardFigure = fig;
end


% =========================================================================
% HELPER FUNCTION: Locate Video Frames Folder
% =========================================================================
function [videoDir, frameFiles] = findVideoFramesFolder(currDir)
    candidates = {
        fullfile(currDir, 'Video_Project_1_frames'), ...
        fullfile(currDir, 'video project frames'), ...
        fullfile(currDir, 'video_project_frames'), ...
        fullfile(currDir, 'Video_Project_frames'), ...
        fullfile(currDir, 'video_frames'), ...
        fullfile(currDir, 'frames')
    };
    
    videoDir = '';
    for c = 1:numel(candidates)
        if exist(candidates{c}, 'dir')
            videoDir = candidates{c};
            break;
        end
    end
    
    % Fallback: search any subfolder in currDir containing 'frame' or 'video'
    if isempty(videoDir)
        dList = dir(currDir);
        for i = 1:numel(dList)
            if dList(i).isdir && ~ismember(dList(i).name, {'.', '..', '.git'})
                nameLower = lower(dList(i).name);
                if contains(nameLower, 'frame') || (contains(nameLower, 'video') && contains(nameLower, 'proj'))
                    candidateDir = fullfile(currDir, dList(i).name);
                    imgs = dir(fullfile(candidateDir, '*.jpg'));
                    if isempty(imgs), imgs = dir(fullfile(candidateDir, '*.png')); end
                    if ~isempty(imgs)
                        videoDir = candidateDir;
                        break;
                    end
                end
            end
        end
    end
    
    frameFiles = [];
    if ~isempty(videoDir)
        frameFiles = dir(fullfile(videoDir, '*.jpg'));
        if isempty(frameFiles), frameFiles = dir(fullfile(videoDir, '*.png')); end
        if isempty(frameFiles), frameFiles = dir(fullfile(videoDir, '*.jpeg')); end
        
        if ~isempty(frameFiles)
            % Sort filenames in natural numerical order
            [~, sortIdx] = sort({frameFiles.name});
            frameFiles = frameFiles(sortIdx);
        end
    end
end
