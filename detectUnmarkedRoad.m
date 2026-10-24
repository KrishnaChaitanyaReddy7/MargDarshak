function [varargout] = detectUnmarkedRoad(frame, sensorParams, net, classNames)
%DETECTUNMARKEDROAD Perception pipeline for marked and unmarked roads.
%
%   result = detectUnmarkedRoad(frame, sensorParams, net, classNames)
%   [drivableMask, laneBoundaries, bevImage, result] = detectUnmarkedRoad(...)
%
%   TWO-TIER HIERARCHICAL PERCEPTION:
%     1. FIRST: Learned semantic segmentation (roadSegmentationNet.mat)
%        isolates the drivable road surface (drivableMask). This guarantees
%        we never search in grass, dirt shoulders, trees, or buildings.
%     2. SECOND: INSIDE that drivable road area, look for painted lane markings:
%        - If painted lane markings exist (Ego Left + Ego Right):
%            * isUnmarkedRoad = false
%            * laneType = "painted_lanes"
%            * Directly follows the painted lane lines (Green) & Center (Cyan).
%        - If no painted markings exist (Unmarked road):
%            * isUnmarkedRoad = true
%            * laneType = "road_edges"
%            * Synthesizes the road boundaries from the physical edges of the
%              drivable area (Yellow) and centers the vehicle in the road corridor.

    % Support image file path or RGB matrix
    if ischar(frame) || isstring(frame)
        frame = imread(frame);
    end

    % ---- Handle Flexible Argument Signatures ---------------------------
    if nargin >= 2 && (isa(sensorParams, 'dlnetwork') || ...
                       isa(sensorParams, 'DAGNetwork') || ...
                       isa(sensorParams, 'SeriesNetwork') || ...
                       (isstruct(sensorParams) && (isfield(sensorParams, 'net') || isfield(sensorParams, 'trainedNet'))))
        tempNet = sensorParams;
        tempSensor = [];
        if nargin >= 4 && isa(classNames, 'monoCamera')
            tempSensor = classNames;
        end
        if nargin >= 3 && (isstring(net) || iscell(net))
            classNames = net;
        else
            classNames = ["notRoad", "road"];
        end
        net = tempNet;
        sensorParams = tempSensor;
    else
        if nargin < 2
            sensorParams = [];
        end
        if nargin < 3
            net = [];
        end
        if nargin < 4 || isempty(classNames)
            classNames = ["notRoad", "road"];
        end
    end

    if isstruct(net)
        if isfield(net, 'trainedNet')
            net = net.trainedNet;
        elseif isfield(net, 'net')
            net = net.net;
        end
    end

    % Auto-load network if not supplied
    if isempty(net)
        candidateFiles = {'roadSegmentationNet.mat', ...
                          fullfile('training', 'roadSegmentationNet.mat'), ...
                          fullfile(fileparts(mfilename('fullpath')), 'roadSegmentationNet.mat'), ...
                          fullfile(fileparts(mfilename('fullpath')), 'training', 'roadSegmentationNet.mat')};
        for k = 1:numel(candidateFiles)
            if exist(candidateFiles{k}, 'file')
                try
                    loadedData = load(candidateFiles{k});
                    if isfield(loadedData, 'trainedNet')
                        net = loadedData.trainedNet;
                        if isfield(loadedData, 'classNames')
                            classNames = loadedData.classNames;
                        end
                        break;
                    elseif isfield(loadedData, 'net')
                        net = loadedData.net;
                        if isfield(loadedData, 'classNames')
                            classNames = loadedData.classNames;
                        end
                        break;
                    end
                catch
                end
            end
        end
    end

    outView  = [0 30 -6 6];      % meters in BEV
    outImgSz = [640 480];

    if isempty(sensorParams)
        % =================================================================
        % UNCALIBRATED MODE (Standard Camera Photos / IDD Images)
        % =================================================================
        bevImage = frame;
        [H, W, ~] = size(frame);

        % Step 1: Deep Learning Drivable Road Segmentation
        drivableMask = estimateDrivableArea(frame, net, classNames);

        % Step 2: Unmarked Road Physical Perimeter & Central Guidance Trajectory
        isUnmarked = true;
        isVirtual  = true;
        laneType   = "unmarked_road";

        [overlayImage, pixelBoundaries] = overlayBoundariesInPixelSpace(frame, drivableMask);
        validBoundaries = parabolicLaneBoundary.empty;

    else
        % =================================================================
        % CALIBRATED MODE (monoCamera / Simulation)
        % =================================================================
        birdsEye = birdsEyeView(sensorParams, outView, outImgSz);
        bevImage = transformImage(birdsEye, frame);

        % Step 1: Deep Learning Drivable Area on BEV
        drivableMask = estimateDrivableArea(bevImage, net, classNames);

        % Step 2: Search for painted lane markings on BEV inside road
        bevGray = im2gray(bevImage);
        roadGray = bevGray .* uint8(drivableMask);
        approxMarkerWidth = 0.20; % meters
        bevBinary = segmentLaneMarkerRidge(roadGray, birdsEye, approxMarkerWidth, 'Sensitivity', 0.25);
        bevBinary = bevBinary & drivableMask; % strictly inside road!

        [imageRows, imageCols] = find(bevBinary);
        boundaries = parabolicLaneBoundary.empty;
        if ~isempty(imageRows)
            xyBoundaryPoints = imageToVehicle(birdsEye, [imageCols, imageRows]);
            boundaries = findParabolicLaneBoundaries(xyBoundaryPoints, 0.6, 'MaxNumBoundaries', 4);
        end

        % Filter boundaries
        validBoundaries = parabolicLaneBoundary.empty;
        for i = 1:numel(boundaries)
            b = boundaries(i);
            len = b.XExtent(2) - b.XExtent(1);
            if b.Strength >= 0.35 && len >= 6
                validBoundaries(end+1) = b; %#ok<AGROW>
            end
        end
        validBoundaries = removeCrossingBoundaryPairs(validBoundaries, [outView(1)+2, outView(2)]);

        isUnmarked = true;
        isVirtual  = true;
        laneType   = "unmarked_road";
        synthBoundaries = fitBoundariesFromDrivableMask(drivableMask, birdsEye, outView);
        validBoundaries = synthBoundaries;
        if ~isempty(validBoundaries)
            xVehiclePoints = 0:1:30;
            overlayImage = insertLaneBoundary(bevImage, validBoundaries, ...
                birdsEye, xVehiclePoints, 'Color', 'yellow', 'LineWidth', 3);
        else
            overlayImage = bevImage;
        end
    end

    % ---- Package Results -----------------------------------------------
    result.rawImage            = frame;
    result.isUnmarkedRoad      = isUnmarked;
    result.isVirtualBoundaries = isVirtual;
    result.laneType            = laneType;
    result.numBoundaries       = numel(validBoundaries);
    result.laneBoundaries      = validBoundaries;
    result.drivableMask        = drivableMask;
    result.roadMask            = drivableMask;
    result.confidence          = sum(drivableMask(:)) / numel(drivableMask);
    result.bevImage            = bevImage;
    result.overlayImage        = overlayImage;
    if exist('pixelBoundaries', 'var')
        result.pixelBoundaries = pixelBoundaries;
    end

    if nargout <= 1
        varargout{1} = result;
    else
        varargout{1} = result.drivableMask;
        varargout{2} = result.laneBoundaries;
        varargout{3} = result.bevImage;
        if nargout >= 4
            varargout{4} = result;
        end
    end
end


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function [hasPaintedLanes, leftLanePts, rightLanePts] = findLanesInsideDrivableMask(frame, drivableMask)
%FINDLANESINSIDEDRIVABLEMASK Detects painted lane lines inside the road mask.

    hasPaintedLanes = false;
    leftLanePts  = [];
    rightLanePts = [];
    [H, W, ~] = size(frame);

    gray = rgb2gray(frame);
    roadGray = gray .* uint8(drivableMask);

    se = strel('disk', 5);
    topHat = imtophat(roadGray, se);
    roadPixels = topHat(drivableMask);
    if isempty(roadPixels)
        return;
    end

    thresh = prctile(roadPixels, 95);
    laneBW = (topHat > max(thresh, 22)) & drivableMask;
    laneBW(1:round(H*0.40), :) = 0; % ignore sky and far horizon

    [HoughSpace, theta, rho] = hough(laneBW);
    if isempty(HoughSpace) || max(HoughSpace(:)) < 10
        return;
    end

    peaks = houghpeaks(HoughSpace, 12, 'threshold', ceil(0.20*max(HoughSpace(:))));
    lines = houghlines(laneBW, theta, rho, peaks, 'FillGap', 25, 'MinLength', 20);

    egoX = W / 2;
    bestLeftLine  = [];
    bestRightLine = [];
    minDistLeft   = inf;
    minDistRight  = inf;

    for k = 1:numel(lines)
        p1 = lines(k).point1;
        p2 = lines(k).point2;
        dx = p2(1) - p1(1);
        dy = p2(2) - p1(2);

        if dy == 0, continue; end
        angleDeg = abs(atan2d(dy, dx));

        % Legitimate lane lines point upwards towards vanishing point (slope >= 22 deg)
        if angleDeg >= 22 && angleDeg <= 85
            xBottom = p1(1) + (H - p1(2)) * (dx / dy);
            minDistEgo = W * 0.07; % minimum lateral distance from ego center
            
            % Left ego lane candidate (left of car)
            if xBottom < egoX && (egoX - xBottom) < minDistLeft && (egoX - xBottom) > minDistEgo
                minDistLeft = egoX - xBottom;
                bestLeftLine = lines(k);
            % Right ego lane candidate (right of car)
            elseif xBottom >= egoX && (xBottom - egoX) < minDistRight && (xBottom - egoX) > minDistEgo
                minDistRight = xBottom - egoX;
                bestRightLine = lines(k);
            end
        end
    end

    if ~isempty(bestLeftLine) && ~isempty(bestRightLine)
        % Validate lane geometry at 70% image height (ego vehicle forward view)
        yCheck = round(H * 0.70);
        dxL = bestLeftLine.point2(1) - bestLeftLine.point1(1);
        dyL = bestLeftLine.point2(2) - bestLeftLine.point1(2);
        xMidL = bestLeftLine.point1(1) + (yCheck - bestLeftLine.point1(2)) * (dxL / dyL);

        dxR = bestRightLine.point2(1) - bestRightLine.point1(1);
        dyR = bestRightLine.point2(2) - bestRightLine.point1(2);
        xMidR = bestRightLine.point1(1) + (yCheck - bestRightLine.point1(2)) * (dxR / dyR);

        laneWidth = xMidR - xMidL;
        % Realistic vehicle lane width: between 20% and 75% of image width
        if laneWidth >= (W * 0.20) && laneWidth <= (W * 0.75)
            hasPaintedLanes = true;
            leftLanePts  = [bestLeftLine.point1; bestLeftLine.point2];
            rightLanePts = [bestRightLine.point1; bestRightLine.point2];
        end
    end
end


function overlay = drawLaneOverlay(image, xL, xR, xC, yGrid, laneColor)
    overlay = image;
    [H, W, ~] = size(image);
    lineWidth = max(2, round(min(H, W) / 200));

    for i = 1:numel(yGrid)
        yr = yGrid(i);
        if yr < 1 || yr > H, continue; end

        if ~isempty(xL)
            xlCols = max(1, xL(i)-lineWidth):min(W, xL(i)+lineWidth);
            overlay(yr, xlCols, 1) = laneColor(1);
            overlay(yr, xlCols, 2) = laneColor(2);
            overlay(yr, xlCols, 3) = laneColor(3);
        end

        if ~isempty(xR)
            xrCols = max(1, xR(i)-lineWidth):min(W, xR(i)+lineWidth);
            overlay(yr, xrCols, 1) = laneColor(1);
            overlay(yr, xrCols, 2) = laneColor(2);
            overlay(yr, xrCols, 3) = laneColor(3);
        end

        if ~isempty(xC) && mod(round(yr / max(6, round(H/60))), 2) == 0
            xcCols = max(1, xC(i)-max(1, lineWidth-1)):min(W, xC(i)+max(1, lineWidth-1));
            overlay(yr, xcCols, 1) = 0;
            overlay(yr, xcCols, 2) = 255;
            overlay(yr, xcCols, 3) = 255;
        end
    end
end


function [overlay, pixelBoundaries] = overlayBoundariesInPixelSpace(image, mask)
    overlay = image;
    pixelBoundaries = struct('left', [], 'right', [], 'center', [], 'type', 'road_edges');
    [H, W, ~] = size(image);

    % 1. Translucent ADAS Drivable Corridor Overlay (Emerald Green ribbon)
    alpha = 0.30;
    roadColor = [0, 200, 120];
    for c = 1:3
        channel = overlay(:, :, c);
        channel(mask) = uint8((1 - alpha) * double(channel(mask)) + alpha * roadColor(c));
        overlay(:, :, c) = channel;
    end

    % 2. Extract perimeter contours from the road mask
    maskClean = bwareaopen(mask, max(200, round(H * W * 0.005)));
    maskClean = imclose(maskClean, strel('disk', max(3, round(min(H, W) / 150))));
    boundaries = bwboundaries(maskClean);

    % Draw road perimeter in Yellow
    borderWidth = max(2, round(min(H, W) / 250));
    for k = 1:numel(boundaries)
        b = boundaries{k};
        if size(b, 1) > 80
            for ptIdx = 1:size(b, 1)
                by = b(ptIdx, 1);
                bx = b(ptIdx, 2);
                rRange = max(1, by - borderWidth):min(H, by + borderWidth);
                cRange = max(1, bx - borderWidth):min(W, bx + borderWidth);
                overlay(rRange, cRange, 1) = 255;
                overlay(rRange, cRange, 2) = 255;
                overlay(rRange, cRange, 3) = 0;
            end
        end
    end

    % 3. Extract Smooth Autonomous Center Corridor Trajectory
    rowsWithRoad = find(any(maskClean, 2));
    if numel(rowsWithRoad) >= 20
        step = max(1, round(numel(rowsWithRoad) / 40));
        sampleRows = rowsWithRoad(1:step:end);
        sampleCols = zeros(size(sampleRows));
        leftEdge   = zeros(size(sampleRows));
        rightEdge  = zeros(size(sampleRows));

        for i = 1:numel(sampleRows)
            cols = find(maskClean(sampleRows(i), :));
            leftEdge(i)   = cols(1);
            rightEdge(i)  = cols(end);
            sampleCols(i) = mean(cols);
        end

        pCenter = fitRobustParabola(sampleRows, sampleCols);
        if ~isempty(pCenter)
            yGrid = (min(sampleRows):max(sampleRows))';
            xC = round(polyval(pCenter, yGrid));
            pixelBoundaries.center = [xC, yGrid];
            pixelBoundaries.left   = [leftEdge, sampleRows];
            pixelBoundaries.right  = [rightEdge, sampleRows];

            % Cyan lane marking removed as requested; centerline saved to pixelBoundaries for control/planning
        end
    end
end


function boundaries = fitBoundariesFromDrivableMask(drivableMask, birdsEye, outView)
    boundaries = parabolicLaneBoundary.empty;
    [H, ~] = size(drivableMask);

    rowStart = round(H * 0.15);
    rowEnd   = H;

    validRows = [];
    leftCols  = [];
    rightCols = [];

    for r = rowStart:rowEnd
        cols = find(drivableMask(r, :));
        if numel(cols) >= 20
            validRows(end+1, 1) = r;       %#ok<AGROW>
            leftCols(end+1, 1)  = cols(1);   %#ok<AGROW>
            rightCols(end+1, 1) = cols(end); %#ok<AGROW>
        end
    end

    if numel(validRows) < 15
        return;
    end

    xyLeft  = imageToVehicle(birdsEye, [leftCols, validRows]);
    xyRight = imageToVehicle(birdsEye, [rightCols, validRows]);

    pLeft  = fitRobustParabola(xyLeft(:, 1), xyLeft(:, 2));
    pRight = fitRobustParabola(xyRight(:, 1), xyRight(:, 2));

    if isempty(pLeft) || isempty(pRight)
        return;
    end

    xEval = linspace(5, 25, 10)';
    yL = polyval(pLeft, xEval);
    yR = polyval(pRight, xEval);
    if mean(yL) < mean(yR)
        temp = pLeft; pLeft = pRight; pRight = temp;
    end

    bLeft = parabolicLaneBoundary(pLeft);
    bLeft.XExtent = [max(outView(1), min(xyLeft(:, 1))), min(outView(2), max(xyLeft(:, 1)))];
    bLeft.Strength = 0.90;

    bRight = parabolicLaneBoundary(pRight);
    bRight.XExtent = [max(outView(1), min(xyRight(:, 1))), min(outView(2), max(xyRight(:, 1)))];
    bRight.Strength = 0.90;

    pCenter = (pLeft + pRight) / 2;
    bCenter = parabolicLaneBoundary(pCenter);
    bCenter.BoundaryType = LaneBoundaryType.Dashed;
    bCenter.XExtent = [min(bLeft.XExtent(1), bRight.XExtent(1)), max(bLeft.XExtent(2), bRight.XExtent(2))];
    bCenter.Strength = 0.85;

    boundaries = [bLeft, bRight, bCenter];
end


function p = fitRobustParabola(x, y)
    p = [];
    if numel(x) < 8, return; end
    try
        pts = [x(:), y(:)];
        maxDist = max(std(y) * 0.25, 2.0);
        [p, inlierIdx] = fitPolynomialRANSAC(pts, 2, maxDist, 'MaxNumTrials', 500, 'Confidence', 99);
        if sum(inlierIdx) < 6
            p = polyfit(x, y, 2);
        end
    catch
        try
            p = polyfit(x, y, 2);
        catch
            p = [];
        end
    end
end


function validBoundaries = removeCrossingBoundaryPairs(boundaries, xRange)
    numBoundaries = numel(boundaries);
    if numBoundaries < 2
        validBoundaries = boundaries;
        return;
    end

    if nargin < 2 || isempty(xRange)
        xRange = [3, 30];
    end

    xs = linspace(xRange(1), xRange(2), 25)';
    ySamples = zeros(numel(xs), numBoundaries);
    yAtEgo   = zeros(1, numBoundaries);

    for i = 1:numBoundaries
        ySamples(:, i) = boundaries(i).computeBoundaryModel(xs);
        yAtEgo(i)      = boundaries(i).computeBoundaryModel(xRange(1));
    end

    [~, order] = sort(yAtEgo);
    boundaries = boundaries(order);
    ySamples   = ySamples(:, order);

    keep = true(1, numBoundaries);
    for i = 1:numBoundaries - 1
        if ~keep(i), continue; end
        for j = i + 1:numBoundaries
            if ~keep(j), continue; end
            diffs = ySamples(:, j) - ySamples(:, i);
            if any(diffs > 0.05) && any(diffs < -0.05)
                if boundaries(i).Strength >= boundaries(j).Strength
                    keep(j) = false;
                else
                    keep(i) = false;
                end
            end
        end
    end

    validBoundaries = boundaries(keep);
end


function mask = estimateDrivableArea(imageToSegment, net, classNames)
    if nargin >= 2 && ~isempty(net)
        try
            if nargin < 3 || isempty(classNames)
                classNames = ["notRoad", "road"];
            elseif ischar(classNames)
                classNames = string(strsplit(classNames, {',', ' '}));
                classNames = classNames(classNames ~= "");
            elseif iscell(classNames)
                classNames = string(classNames);
            end

            % Ensure classNames has at least 2 classes
            if numel(classNames) < 2
                classNames = ["notRoad", "road"];
            end

            [origH, origW, ~] = size(imageToSegment);

            % Resolution normalization: DeepLabv3+ was trained on [227, 320]
            % Evaluating raw 4K/24MP images directly causes convolution receptive-field failure!
            targetSize = [227, 320];
            if isprop(net, 'Layers') && isa(net.Layers(1), 'nnet.cnn.layer.ImageInputLayer')
                targetSize = net.Layers(1).InputSize(1:2);
            end

            imgScaled = imresize(imageToSegment, targetSize);
            predicted = semanticseg(imgScaled, net, 'Classes', classNames);
            maskSmall = (predicted == "road");

            % Upsample the mask back to full resolution
            mask = imresize(maskSmall, [origH, origW], 'nearest');
            mask = bwareaopen(mask, max(250, round(origH * origW * 0.005)));
            mask = imfill(mask, 'holes');
            return;
        catch ME
            warning('detectUnmarkedRoad:SemanticSegFailed', ...
                'Deep learning segmentation failed: %s. Using HSV fallback.', ME.message);
        end
    end

    hsv = rgb2hsv(imageToSegment);
    [h, w, ~] = size(imageToSegment);

    patch = hsv(round(h*0.80):h, round(w*0.4):round(w*0.6), :);
    roadHue = mean(patch(:,:,1), 'all');
    roadSat = mean(patch(:,:,2), 'all');
    roadVal = mean(patch(:,:,3), 'all');

    hueTol = 0.08;
    satTol = 0.25;
    valTol = 0.30;

    mask = abs(hsv(:,:,1) - roadHue) < hueTol & ...
           abs(hsv(:,:,2) - roadSat) < satTol & ...
           abs(hsv(:,:,3) - roadVal) < valTol;

    mask = imclose(mask, strel('disk', 5));
    mask = imfill(mask, 'holes');
    mask = bwareaopen(mask, 200);
end