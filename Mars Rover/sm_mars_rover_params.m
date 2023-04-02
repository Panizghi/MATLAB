%% Script to setup parameters for the sm_mars_rover model 

% Copyright 2021-2022 The MathWorks, Inc

%% Mars Rover Params

run(['sm_mars_rover_data',filesep,'rover_params']) ; 

%% Rigid Terrain Params

run(['sm_mars_rover_data',filesep,'rover_rigid_terrain_params']);

%% Sample Position

sample_position = [72.3000   30.4729  3.5231]; 

%% Rover Path Planning and Control Params
load(['sm_mars_rover_data',filesep,'roverPath1.mat'])
load(['sm_mars_rover_data',filesep,'roverPath2.mat'])
roverPath = rover_path1;
goal_loc = [roverPath.x(end) roverPath.y(end)];
% Points for visualizing the waypoints
pointCloud = [roverPath.x  roverPath.y roverPath.z];

%% Rover Arm Params
run(['sm_mars_rover_data',filesep,'rover_arm_params']);
arm_q0 =  [-2.3554 -125.9144   49.8193 -175.7336  177.6446  180.0000];
%% Rover Arm Planning And Control
load(['sm_mars_rover_data',filesep,'roverArmTaskSpaceConfig.mat']);

%% Rover Camera Assmebly Params
run(['sm_mars_rover_data',filesep,'rover_camera_params']) ;
