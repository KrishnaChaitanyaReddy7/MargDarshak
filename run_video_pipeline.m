function resStruct = run_video_pipeline(frameOption, speedOrStep)
% RUN_VIDEO_PIPELINE Select and run autonomous perception on Video Project Frames
%
% This command is specifically designed for selecting and testing frames from
% your video frames folder ('Video_Project_1_frames' or 'video project frames').
%
% =========================================================================
% AVAILABLE COMMANDS & USAGE EXAMPLES:
% =========================================================================
%   run_video_pipeline            % Interactive menu / runs Frame 1
%   run_video_pipeline(15)        % Select and run specific Frame #15 (frame_015.jpg)
%   run_video_pipeline('random')  % Select and run a random video frame
%   run_video_pipeline('select')  % Open graphical file picker dialog to click any frame
%   run_video_pipeline('loop')    % Step-by-step frame inspection (Enter=Next, b=Back, q=Quit)
%   run_video_pipeline('play')    % Continuous animated video playback on 4-panel HUD
%   run_video_pipeline('play', 2) % Play with step 2 (skips every 2nd frame for faster playback)
% =========================================================================

clc;
currDir = fileparts(mfilename('fullpath'));
if isempty(currDir), currDir = pwd; end
cd(currDir);
addpath(currDir);

%% 1. Discover Video Frames Directory
[videoDir, frameFiles] = findVideoFramesDir(currDir);
totalFrames = numel(frameFiles);

if totalFrames == 0
    error('Could not find video frames in %s. Please ensure "Video_Project_1_frames" or "video project frames" exists.', currDir);
end

[~, folderBaseName] = fileparts(videoDir);

%% 2. Handle Inputs & Modes
if nargin < 1 || isempty(frameOption)
    fprintf('========================================================================\n');
    fprintf('             VIDEO PROJECT FRAMES ADAS PIPELINE (SIH 2026)             \n');
    fprintf('========================================================================\n');
    fprintf('  Folder Detected:  %s (%s)\n', folderBaseName, videoDir);
    fprintf('  Total Frames:     %d frames (%s -> %s)\n', totalFrames, frameFiles(1).name, frameFiles(end).name);
    fprintf('------------------------------------------------------------------------\n');
    fprintf('  Select Option:\n');
    fprintf('    [1-%d]  : Enter a number to test that specific frame\n', totalFrames);
    fprintf('    ''r''    : Random frame\n');
    fprintf('    ''s''    : Visual file picker dialog\n');
    fprintf('    ''l''    : Step-by-step interactive loop\n');
    fprintf('    ''p''    : Continuous video stream playback\n');
    fprintf('    [Enter]: Run Frame #1 by default\n');
    fprintf('------------------------------------------------------------------------\n');
    
    userChoice = input(sprintf('Your selection [1-%d, r, s, l, p, or Enter]: ', totalFrames), 's');
    userChoice = strtrim(userChoice);
    
    if isempty(userChoice)
        resStruct = run_master_integrated_pipeline('video', 1);
    elseif strcmpi(userChoice, 'r')
        resStruct = run_master_integrated_pipeline('video');
    elseif strcmpi(userChoice, 's')
        resStruct = run_master_integrated_pipeline('select');
    elseif strcmpi(userChoice, 'l')
        resStruct = run_master_integrated_pipeline('video_loop', 1);
    elseif strcmpi(userChoice, 'p')
        resStruct = run_master_integrated_pipeline('video_play', 1);
    else
        numVal = str2double(userChoice);
        if ~isnan(numVal)
            resStruct = run_master_integrated_pipeline('video', round(numVal));
        else
            resStruct = run_master_integrated_pipeline(userChoice);
        end
    end
    return;
end

%% Direct Command Dispatch
if isnumeric(frameOption)
    % e.g. run_video_pipeline(15)
    fIdx = min(max(1, round(frameOption)), totalFrames);
    resStruct = run_master_integrated_pipeline('video', fIdx);

elseif ischar(frameOption) || isstring(frameOption)
    opt = lower(strtrim(char(frameOption)));
    
    switch opt
        case {'random', 'rand', 'r'}
            % e.g. run_video_pipeline('random')
            resStruct = run_master_integrated_pipeline('video');
            
        case {'select', 'browse', 'choose', 'pick', 's'}
            % e.g. run_video_pipeline('select')
            resStruct = run_master_integrated_pipeline('select');
            
        case {'loop', 'step', 'l'}
            % e.g. run_video_pipeline('loop')
            startIdx = 1;
            if nargin >= 2 && isnumeric(speedOrStep)
                startIdx = speedOrStep;
            end
            resStruct = run_master_integrated_pipeline('video_loop', startIdx);
            
        case {'play', 'stream', 'video', 'p'}
            % e.g. run_video_pipeline('play')
            startIdx = 1;
            if nargin >= 2 && isnumeric(speedOrStep)
                startIdx = speedOrStep;
            end
            resStruct = run_master_integrated_pipeline('video_play', startIdx);
            
        otherwise
            % If a filename or relative path is passed: e.g. 'frame_015.jpg'
            resStruct = run_master_integrated_pipeline(frameOption);
    end
end

if nargout == 0
    clear resStruct;
end
end


% =========================================================================
% HELPER FUNCTION: Locate Video Frames Folder
% =========================================================================
function [videoDir, frameFiles] = findVideoFramesDir(currDir)
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
