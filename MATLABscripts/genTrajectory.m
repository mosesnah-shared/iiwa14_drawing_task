%% Cleaning up + Environment Setup
clear; close all; clc;
addpath( '../DMPmodules')

%% Call DMP kinematic modules


%% Test Trajectory
% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot )  

% Update kinematics
robot.updateKinematics( robot.q_init );
anim.update( 0 );
