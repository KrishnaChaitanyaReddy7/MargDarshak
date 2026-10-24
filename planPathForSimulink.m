function smoothPath = planPathForSimulink(drivableMask, start, goal)
map = occupancyMap(1 - drivableMask, 1);
planner = plannerAStarGrid(map);
path = plan(planner, start, goal);

if isempty(path) || size(path,1) < 2 || size(path,2) < 2
    smoothPath = zeros(1,2);  % Simulink needs fixed-type/non-empty output
    return;
end

t = 1:size(path,1);
tt = linspace(1, size(path,1), 100);
smoothX = spline(t, path(:,1), tt);
smoothY = spline(t, path(:,2), tt);
candidatePath = [smoothY(:), smoothX(:)];

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
end
if isempty(path) || size(path,1) < 2 || size(path,2) < 2
    smoothPath = zeros(100,2);  % match the shape of the normal-case output
    return;
end
end