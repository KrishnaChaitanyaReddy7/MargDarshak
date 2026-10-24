function smoothPath = planPathFromMask(drivableMask, start, goal, showPlot)
% PLANPATHFROMMASK A* Path Planner on Drivable Surface Mask
% Smooths path using spline interpolation and checks for collision safety.
%
% Syntax:
%   smoothPath = planPathFromMask(drivableMask)
%   smoothPath = planPathFromMask(drivableMask, start, goal)
%   smoothPath = planPathFromMask(drivableMask, start, goal, showPlot)
%
% Inputs:
%   drivableMask - Binary 2D mask (1 = drivable, 0 = obstacle/non-road)
%   start        - [row, col] start position (bottom of road). If omitted, auto-detected.
%   goal         - [row, col] goal position (top/horizon of road). If omitted, auto-detected.
%   showPlot     - (Optional) Boolean flag to show standalone Figure 99 (default: true)

if nargin < 4
    showPlot = true;
end

[H, W] = size(drivableMask);

% Ensure drivableMask is double/logical
drivableMask = double(drivableMask > 0.5);

%% Automatic start and goal computation if not provided
if nargin < 2 || isempty(start)
    % Find bottom drivable center (search around 85% to 95% of height)
    bottomRow = round(H * 0.90);
    cols = find(drivableMask(bottomRow, :));
    if isempty(cols)
        % Search upwards for first drivable row from bottom
        for r = H:-5:round(H*0.6)
            cols = find(drivableMask(r, :));
            if ~isempty(cols)
                bottomRow = r;
                break;
            end
        end
    end
    if ~isempty(cols)
        start = [bottomRow, round(median(cols))];
    else
        start = [round(H*0.9), round(W*0.5)];
    end
end

if nargin < 3 || isempty(goal)
    % Find top/horizon drivable center (search around 40% to 55% of height)
    topRow = round(H * 0.45);
    cols = find(drivableMask(topRow, :));
    if isempty(cols)
        % Search downwards from 35% height
        for r = round(H*0.35):5:round(H*0.7)
            cols = find(drivableMask(r, :));
            if ~isempty(cols)
                topRow = r;
                break;
            end
        end
    end
    if ~isempty(cols)
        goal = [topRow, round(median(cols))];
    else
        goal = [round(H*0.45), round(W*0.5)];
    end
end

% Ensure start and goal are inside bounds and drivable
start(1) = max(1, min(H, round(start(1))));
start(2) = max(1, min(W, round(start(2))));
goal(1)  = max(1, min(H, round(goal(1))));
goal(2)  = max(1, min(W, round(goal(2))));

% If start or goal falls on obstacle, nudge to nearest drivable pixel
if drivableMask(start(1), start(2)) == 0
    [rGrid, cGrid] = find(drivableMask);
    if ~isempty(rGrid)
        dists = (rGrid - start(1)).^2 + (cGrid - start(2)).^2;
        [~, minIdx] = min(dists);
        start = [rGrid(minIdx), cGrid(minIdx)];
    end
end

if drivableMask(goal(1), goal(2)) == 0
    [rGrid, cGrid] = find(drivableMask);
    if ~isempty(rGrid)
        dists = (rGrid - goal(1)).^2 + (cGrid - goal(2)).^2;
        [~, minIdx] = min(dists);
        goal = [rGrid(minIdx), cGrid(minIdx)];
    end
end

%% A* Grid Planning with Adaptive Resolution
maxPlanDim = 300;
if max(H, W) > maxPlanDim
    scaleFactor = maxPlanDim / max(H, W);
    gridMask = imresize(drivableMask, scaleFactor, 'nearest');
    pStart = max(1, round(start * scaleFactor));
    pGoal  = max(1, round(goal * scaleFactor));
    pStart(1) = min(size(gridMask, 1), pStart(1));
    pStart(2) = min(size(gridMask, 2), pStart(2));
    pGoal(1)  = min(size(gridMask, 1), pGoal(1));
    pGoal(2)  = min(size(gridMask, 2), pGoal(2));
    
    % Ensure start and goal are on drivable cells in scaled grid
    if gridMask(pStart(1), pStart(2)) == 0
        [pr, pc] = find(gridMask);
        if ~isempty(pr)
            [~, idx] = min((pr - pStart(1)).^2 + (pc - pStart(2)).^2);
            pStart = [pr(idx), pc(idx)];
        end
    end
    if gridMask(pGoal(1), pGoal(2)) == 0
        [pr, pc] = find(gridMask);
        if ~isempty(pr)
            [~, idx] = min((pr - pGoal(1)).^2 + (pc - pGoal(2)).^2);
            pGoal = [pr(idx), pc(idx)];
        end
    end
    
    map = occupancyMap(1 - gridMask, 1);
    planner = plannerAStarGrid(map);
    rawScaledPath = plan(planner, pStart, pGoal);
    
    if ~isempty(rawScaledPath) && size(rawScaledPath, 1) >= 2
        path = rawScaledPath / scaleFactor;
        path(:, 1) = min(H, max(1, path(:, 1)));
        path(:, 2) = min(W, max(1, path(:, 2)));
    else
        path = [];
    end
else
    map = occupancyMap(1 - drivableMask, 1);
    planner = plannerAStarGrid(map);
    path = plan(planner, start, goal);
end

if showPlot
    figure(99);
    clf;
    imagesc(drivableMask);
    colormap([0.8 0.1 0.1; 0.2 0.7 0.2]);
    set(gca, 'YDir', 'reverse');
    hold on;
    axis equal;
    axis([0.5 size(drivableMask,2)+0.5 0.5 size(drivableMask,1)+0.5]);
    xlabel('Columns (X)');
    ylabel('Rows (Y)');
end

if isempty(path) || size(path,1) < 2 || size(path,2) < 2
    warning('No usable path found - returning empty result.');
    smoothPath = [];
    if showPlot
        title('No feasible path found');
        plot(start(2), start(1), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        plot(goal(2), goal(1), 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    end
    return;
end

%% Spline Smoothing
t = 1:size(path,1);
tt = linspace(1, size(path,1), 100);
smoothX = spline(t, path(:,1), tt); % smoothed row coordinates
smoothY = spline(t, path(:,2), tt); % smoothed col coordinates
candidatePath = [smoothY(:), smoothX(:)]; % [col(X), row(Y)]

%% Collision Safety Check
isSafe = true;
for i = 1:size(candidatePath,1)
    pt = round(candidatePath(i,:));
    pt(1) = max(1, min(size(drivableMask,2), pt(1)));
    pt(2) = max(1, min(size(drivableMask,1), pt(2)));
    if drivableMask(pt(2), pt(1)) == 0
        isSafe = false;
        break;
    end
end

if isSafe
    smoothPath = candidatePath;
else
    smoothPath = [path(:,2), path(:,1)];
    disp('Smoothed path was unsafe - using raw A star path instead');
end

if showPlot
    title('A* Path Trajectory');
    plot(smoothPath(:,1), smoothPath(:,2), 'b-', 'LineWidth', 2.5);
    plot(start(2), start(1), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    plot(goal(2), goal(1), 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    legend('Trajectory', 'Start', 'Goal', 'Location', 'northoutside');
end
end
