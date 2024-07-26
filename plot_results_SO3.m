%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

% Get the data
raw_data = parse_txt( 'data_plot/Kp1200_Kr70.txt',0 );

% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

t_arr  = raw_data( :,  1   )'; t_arr = t_arr - t_arr( 1 );
q_arr  = raw_data( :,  2:8 )';

Nt = length( t_arr );

R_des = zeros( 3, 3, Nt );

for i = 1 : Nt
    R_des( 1, :, i ) = raw_data( i,  9:11 );
    R_des( 2, :, i ) = raw_data( i, 12:14 );
    R_des( 3, :, i ) = raw_data( i, 15:17 );
end

Nt = length( t_arr );

p_arr = zeros( 3, Nt );
R_arr = zeros( 3, 3, Nt );

for i = 1 : Nt
    tmp = robot.getForwardKinematics( q_arr( :, i ) );
    R_arr( :, :, i ) = tmp( 1:3, 1:3 );
    p_arr( :, i ) = tmp( 1:3, 4 );
end

idx_start = 1100;
f = figure( ); a = axes( 'parent', f );
plot( a, p_arr( 1, idx_start:end ), p_arr( 2, idx_start:end )  )
axis equal
set( a, 'xlim', [0.4, 0.7], 'ylim', [0.00, 0.3])
set( a, 'xticklabel', {}, 'yticklabel', {})

rmse( p_arr( :, idx_start:end ) - mean( p_arr( :, idx_start:end ), 2 ), zeros( 3, Nt-idx_start+1 ), "all" )