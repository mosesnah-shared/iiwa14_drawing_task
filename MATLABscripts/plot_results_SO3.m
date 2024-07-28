%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

% Get the data
raw_data = parse_txt( 'data_plot/Kp600_Kr30.txt',0 );

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

% Get the logarithm map 
w0_arr = zeros( 3, Nt );
w_arr  = zeros( 3, Nt );
w_del  = zeros( 3, Nt );
for i = 1 : Nt
    w0_arr( :, i ) = so3_to_R3( LogSO3( R_des( :, :, i ) ) );
     w_arr( :, i ) = so3_to_R3( LogSO3( R_arr( :, :, i ) ) );
     w_del( :, i ) = so3_to_R3( LogSO3( R_des( :, :, i )' * R_arr( :, :, i )  ) );
end


% Define the radius
radius = pi;

% Define the number of points in theta and phi directions
n_points = 30;

% Create a grid of theta and phi values
theta = linspace(0, pi, n_points);
phi = linspace(0, 2*pi, n_points);
[theta, phi] = meshgrid(theta, phi);

% Calculate the Cartesian coordinates of the sphere
x = radius * sin(theta) .* cos(phi);
y = radius * sin(theta) .* sin(phi);
z = radius * cos(theta);

% Plot the wireframe sphere
f = figure( ); a = axes( 'parent', f );
mesh(a, x, y, z, 'EdgeColor', 'k', 'FaceAlpha', 0.0 ); % Wireframe with black edges
% surf(a, x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none'); % Transparent surface
axis equal;
set( a, 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )
set( a, 'visible', 'off' )
set( a, 'xlim', [ -0.5, 0.5 ], 'ylim', [ -0.5, 0.5 ], 'zlim', [ -0.5, 0.5 ])
hold on
plot3( a, w_del( 1, : ), w_del( 2, : ), w_del( 3, : ), 'linewidth', 7 )
% saveas( f, 'Kr100_zoom.jpeg' )
rmse = sqrt( mean( w_del.^2, "all" ) );
rmse
