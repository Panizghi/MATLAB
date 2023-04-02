function [goal_loc,roverPath,pointCloud] = rover_path_select(pathNum)

% Copyright 2021-2022 The MathWorks, Inc.

%% Rover Path
load(['sm_mars_rover_data',filesep,'roverPath1.mat']);
load(['sm_mars_rover_data',filesep,'roverPath2.mat']);
if pathNum == 1
    roverPath = rover_path1;
else
    roverPath = rover_path2;
end
goal_loc = [roverPath.x(end) roverPath.y(end)];
% Points for visualizing the waypoints
pointCloud = [roverPath.x  roverPath.y roverPath.z];
end