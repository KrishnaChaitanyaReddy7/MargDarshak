%% prepare_obstacle_labels.m
% Prepares multi-class obstacle ground truth labels from IDD-Lite dataset.
%
% Label Mapping:
%   0: background (road, sidewalk, vegetation, sky, void)
%   1: pedestrian (living things, people, animals)
%   2: two_wheeler (motorcycles, scooters, bicycles)
%   3: auto_rickshaw (3-wheelers)
%   4: vehicle (cars, buses, trucks)
%   5: road_obstacle (poles, barricades, barriers)

clc; clear;

srcRoot = 'E:\SIH\idd-lite\idd20k_lite\gtFine';
dstRoot = 'E:\SIH\training\idd20k_lite\obstacle_labels';

splits = {'train', 'val'};
% Correct IDD-Lite Level-1 Mapping:
%   IDD Class 0: Drivable (Road)            -> 0 (Background)
%   IDD Class 1: Non-Drivable (Sidewalk)    -> 0 (Background)
%   IDD Class 2: Living Things              -> 1 (Pedestrian / Living Things)
%   IDD Class 3: Vehicles (All Indian)     -> 2 (Vehicles: Cars, Autos, Bikes)
%   IDD Class 4: Roadside Objects           -> 3 (Road Obstacles / Barricades)
%   IDD Class 5: Far Objects (Trees/Bldgs)  -> 0 (Background - NOT vehicles!)
%   IDD Class 6: Sky                        -> 0 (Background - NOT obstacles!)
%   IDD Class 255: Unlabeled/Void           -> 0 (Background)

classes = {'background', 'pedestrian', 'vehicle', 'road_obstacle'};

fprintf('=======================================================\n');
fprintf('  PREPARING CORRECT INDIAN ROAD OBSTACLE LABELS (IDD)   \n');
fprintf('=======================================================\n');

for s = 1:numel(splits)
    split = splits{s};
    srcDir = fullfile(srcRoot, split);
    dstDir = fullfile(dstRoot, split);
    
    if ~exist(dstDir, 'dir')
        mkdir(dstDir);
    end
    
    % Find all *_label.png files (excluding *_inst_label.png)
    files = dir(fullfile(srcDir, '**', '*_label.png'));
    % Filter out inst_label
    isInst = contains({files.name}, 'inst_label');
    files = files(~isInst);
    
    fprintf('Processing %s split (%d images)...\n', split, numel(files));
    tic;
    
    for i = 1:numel(files)
        srcPath = fullfile(files(i).folder, files(i).name);
        
        % Preserve subdirectory structure
        relPath = extractAfter(files(i).folder, fullfile(srcRoot, split));
        if startsWith(relPath, filesep), relPath = relPath(2:end); end
        
        subDstDir = fullfile(dstDir, relPath);
        if ~exist(subDstDir, 'dir')
            mkdir(subDstDir);
        end
        
        dstPath = fullfile(subDstDir, files(i).name);
        
        lbl = imread(srcPath);
        outLbl = zeros(size(lbl), 'uint8'); % 0: background (road, sidewalk, buildings, trees, sky)
        
        % Mapping to obstacle classes:
        outLbl(lbl == 2) = 1; % pedestrian & living things
        outLbl(lbl == 3) = 2; % vehicle (cars, autos, two-wheelers, trucks, buses)
        outLbl(lbl == 4) = 3; % road_obstacle (barricades, poles, traffic cones)
        
        imwrite(outLbl, dstPath);
        
        if mod(i, 300) == 0
            fprintf('  [%s] Processed %d / %d images...\n', split, i, numel(files));
        end
    end
    
    t = toc;
    fprintf('Completed %s split in %.2f seconds.\n\n', split, t);
end

fprintf('All obstacle ground truth labels successfully prepared in:\n  %s\n', dstRoot);
