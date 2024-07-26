%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

%% Call DMP kinematic modules

% Load the kinematic modules
tmp = load( 'A.mat' ); mov = tmp.data;

% The total time of the trajectory and its time array
T     = 6.0;
dt    = 0.0033;  
t_arr = 0:dt:T;  
Nt    = length( t_arr );

N = size( mov.weight, 2 );

time_scl = 1.0;
toffset = 0.0;

% The Canonical System and Nonlinear Forcing Term 
cs = CanonicalSystem( 'discrete', mov.tau * time_scl, mov.alpha_s );
fd = NonlinearForcingTerm( cs, N );    

% The input to the Transformation System
input = fd.calc_forcing_term( t_arr( 1:end-1 ), mov.weight, toffset, eye( 2 ), 'trimmed' );    
ts    = TransformationSystem( mov.alpha_z, mov.beta_z, cs );

[ y_arr, ~, ~] = ts.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), input + mov.alpha_z * mov.beta_z * mov.goal, toffset, t_arr  );            

% Check the trajectory output
f = figure( ); a = axes( 'parent', f );

plot( a, y_arr( 1, : ), y_arr( 2, : ), 'linewidth', 5 )
axis equal

input_traj = 0.03*y_arr;

clear tmp* mov_*

%% Test Trajectory
% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

% Rotation Function
my_rot = @( ang ) [ cos( ang ), -sin( ang ); sin( ang ), cos( ang ) ];

% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.3,1.1], 'yLim', [-0.7,0.7], 'zLim', [0,1.4] );
anim.init( );
anim.attachRobot( robot )  

view( anim.hAxes, [ -90, 90 ] )

% Get the initial position
q_init = robot.q_init;
q_init( 2 ) =  0.3;
q_init( 4 ) = -2.0;
q_init( 6 ) =  1.1;
H_init = robot.getForwardKinematics( q_init );
p_init = H_init( 1:3,   4 );
R_init = H_init( 1:3, 1:3 );

% Update kinematics
robot.updateKinematics( q_init );
anim.update( 0 );

% We need to rotate and flip the trajectory 
input_traj = my_rot( pi/2 )*input_traj;
input_traj( 1, : ) = -input_traj( 1, : );

% Run the controller
plot3( anim.hAxes, input_traj( 1, : )+p_init( 1 ), input_traj( 2, : )+p_init( 2 ), p_init( 3 )*ones( 1, length( input_traj ) ), 'linewidth', 3 )

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 200 * eye( 3 );
Bp = 0.1 * Kp;

% Task-space impedances for Position
Kr = 2 * eye( 3 );

% Joint-space impedance, damping
Bq = .8 * eye( robot.nq );

q = q_init;
dq = zeros( 7, 1 );

for i = 1 : Nt

    t = t_arr( i );
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );
    
    % Get the end-effector position and velocity 
    dp = JH( 1:3, : ) * dq;
    
    % The initial end-effector position 
    H = robot.getForwardKinematics( q );
    R = H( 1:3, 1:3 );
    p = H( 1:3,   4 );

    tau1 = JH( 1:3, : )' * ( Kp * ( [ input_traj( :, i ); 0 ] + p_init - p ) + Bp * ( -dp ) );
    tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( LogSO3( R' * R_init ) );
    tau3 = - Bq * dq;
    tau  = tau1 + tau2 + tau3;

    rhs = M\( -C * dq + tau ); 

    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;
    end
                                      
end

anim.close( )

%% Save Data

csvwrite( 'data_output/A_letter_1p0.csv', input_traj );
