%% Cleaning up + Environment Setup
clear; close all; clc;

% Figure Configuration and Colors
fig_config( 'fontSize', 20, 'markerSize', 10 )

%% Call DMP kinematic modules

% Load the kinematic modules
tmp1 = load( 'A.mat'     ); mov_dis1 = tmp1.data;
tmp2 = load( 'B.mat'     ); mov_dis2 = tmp2.data;
tmp3 = load( 'eight.mat' ); mov_rhy1 = tmp3.data;

% The number of movements and the movement types
movements = {   mov_dis1,   mov_dis2,  mov_rhy1  };
types     = { 'discrete', 'discrete', 'rhythmic' };
Nmov      = length( movements );

% Generate the trajectories
trajectories = cell( 1, Nmov );

% The total time of the trajectory and its time array
T     = 40.0;
dt    = 0.0033;  
t_arr = 0:dt:T;  
Nt    = length( t_arr );

% Time amplitude of each movement 
time_scl = [ 2.5, 3.0, 0.7 ];

% Scaling amplitude 
size_scl = [ 1.0, 1.0, 0.7 ];

% Time offset of each movement
toffset  = [ 2.0, 17.0, 0.0 ];

% Generate the trajectories via Iteration
for i = 1 : Nmov
    
    mov = movements{ i };     

    % The number of Basis Functions for the DMP
    N   = size( mov.weight, 2 );
    
    % Save the alpha_s value for DMP
    if strcmp( types{ i }, 'discrete' )
        as = mov.alpha_s;
    else
        as = 1.0;
    end

    % The Canonical System and Nonlinear Forcing Term 
    cs      = CanonicalSystem( types{ i }, mov.tau * time_scl( i ), as );
    fd      = NonlinearForcingTerm( cs, N );    

    % The input to the Transformation System
    if strcmp( types{ i }, 'discrete' )
        input   = fd.calc_forcing_term( t_arr( 1:end-1 ), mov.weight, toffset( i ), eye( 2 ), 'trimmed' );    
    else
        input   = fd.calc_forcing_term( t_arr( 1:end-1 ), mov.weight, toffset( i ), eye( 2 ) );    
    end
    ts      = TransformationSystem( mov.alpha_z, mov.beta_z, cs );

    [ y_arr, ~, ~] = ts.rollout( zeros( 2, 1 ), zeros( 2, 1 ), zeros( 2, 1 ), input + mov.alpha_z * mov.beta_z * mov.goal, toffset( i ), t_arr  );            
    trajectories{ i }  = y_arr;
end

% Check the trajectory output
f = figure( ); a = axes( 'parent', f );

for i = 1 : Nmov
    tmp_a = subplot( 1, Nmov, i );

    plot( tmp_a, trajectories{ i }( 1, : ), trajectories{ i }( 2, : ), 'linewidth', 3 )
    title( movements{ i }.name, 'fontsize', 30 )
    axis equal
end

clear tmp* mov_*

%% Combine the movements

% Define the offsets of the movement
offsets = { [ 0.0; 0.0 ], [ 0.16; 0.18 ], [ 0.0; 0.0 ] };

% Again, Scale the output
scl_trajs = 0.07*[ 0.3, 0.23, 0.2 ];

% Rotation of the movement
rot_amp  = [ 0.0, 0.0, 0.0]*pi/180.0;

% Rotation Function
my_rot = @( ang ) [ cos( ang ), -sin( ang ); sin( ang ), cos( ang ) ];

% Time offset of the discrete movement
time_off = toffset( 2 );
idx1 = round( time_off/dt );

% Set the activation function
trajectories{ 2 } = circshift( trajectories{ 2 }, 1000 );

% Define the activation functions for each movements
idx_off = 200;
act_func_dis1 = smth_activation( Nt, 1.0, 0.0, idx1-idx_off, idx1+idx_off  );
act_func_dis2 = smth_activation( Nt, 0.0, 1.0, idx1-idx_off, idx1+idx_off  );


% The combined_trajectory
combined_traj = ( offsets{ 1 } + my_rot( rot_amp( 1 ) ) * scl_trajs( 1 ) * trajectories{ 1 } ) .* act_func_dis1 + ...
                ( offsets{ 2 } + my_rot( rot_amp( 2 ) ) * scl_trajs( 2 ) * trajectories{ 2 } ) .* act_func_dis2 + ...
                ( offsets{ 3 } + my_rot( rot_amp( 3 ) ) * scl_trajs( 3 ) * trajectories{ 3 } );

f = figure( ); a = axes( 'parent', f );
plot( a, combined_traj( 1, : ), combined_traj( 2, : ), 'linewidth', 3 )
axis equal

%% Test Trajectory
% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

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
input_traj = my_rot( pi/2 )*combined_traj;
input_traj( 1, : ) = -input_traj( 1, : );

% Run the controller
plot3( anim.hAxes, input_traj( 1, : )+p_init( 1 ), input_traj( 2, : )+p_init( 2 ), p_init( 3 )*ones( 1, length( input_traj ) ), 'linewidth', 3 )

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 400 * eye( 3 );
Bp = 0.1 * Kp;

% Task-space impedances for Position
Kr = 30 * eye( 3 );

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

%% If all good, then save it as data
csvwrite( 'data_output/AB_write_backward2.csv', input_traj );