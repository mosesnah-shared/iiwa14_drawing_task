%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

% Get the data
raw_data = parse_txt( 'data_plot/Kp1200_letterA_1p0.txt',0 );

% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

t_arr  = raw_data( :,  1   )'; t_arr = t_arr - t_arr( 1 );
q_arr  = raw_data( :,  2:8 )';
p0_arr = raw_data( :, 10:12 )';

Nt = length( t_arr );
p_arr = zeros( 3, Nt );

for i = 1 : Nt
    tmp = robot.getForwardKinematics( q_arr( :, i ) );
    p_arr( :, i ) = tmp( 1:3, 4 );
end

idx_start = 60;
 
f = figure( ); a = axes( 'parent', f );
plot( a, p_arr( 2, idx_start:end ), p_arr( 3, idx_start:end ), 'linewidth', 6 )
hold on
plot( a, p0_arr( 2, idx_start:end ), p0_arr( 3, idx_start:end ), 'linestyle','--', 'color', 'k', 'linewidth', 4 )
axis equal
set( a, 'visible', 'off' , 'xlim', [-0.1, 0.3], 'ylim', [0.15, 0.55])

rmse( p_arr, p0_arr, "all" )

saveas( gcf, 'images/Kp1200_A_1p0.jpeg')