%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

% Get the data
raw_data = parse_txt( 'data_plot/Kp600_letterA_0p5.txt',0 );

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
 
f = figure( ); a = axes( 'parent', f );
plot( a, p_arr( 2, : ), p_arr( 3, : ))
hold on
plot( a, p0_arr( 2, : ), p0_arr( 3, : ))
axis equal
set( a, 'visible', 'off' )

saveas( gcf, 'images/Kp600_A_0p5.jpeg')