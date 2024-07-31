/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS,"
without warranty of any kind, including without limitation the warranties
of merchantability, fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable
for any particular purpose. In no event shall KUKA be responsible for loss
or damages arising from the installation or use of the Software,
including but not limited to any indirect, punitive, special, incidental
or consequential damages of any character including, without limitation,
damages for loss of goodwill, work stoppage, computer failure or malfunction,
or any and all other commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by KUKA.
Should the Software prove defective, KUKA is not liable for the entire cost
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2015
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned
to KUKA Roboter GmbH immediately upon request.
This material and the information illustrated or contained herein may not be used,
reproduced, stored in a retrieval system, or transmitted in whole
or in part in any way - electronic, mechanical, photocopying, recording,
or otherwise, without the prior written consent of KUKA Roboter GmbH.






\file
\version {1.9}
*/
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <chrono>
#include <iomanip>
#include <vector>

#include "MyLBRClient.h"
#include "exp_robots.h"

using namespace std;

#ifndef M_PI
    #define M_PI 3.14159265358979
#endif

#ifndef NCoef
    #define NCoef 1
#endif

static double filterOutput[ 7 ][ NCoef+1 ]; // output samples. Static variables are initialised to 0 by default.
static double  filterInput[ 7 ][ NCoef+1 ]; //  input samples. Static variables are initialised to 0 by default.


Eigen::MatrixXd readCSV(const string& filename)
{
    ifstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Couldn't open the file: " << filename << endl;
        exit(1);
    }

    vector<vector<double>> values;

    string line;
    int lineNum = 0;
    int numCols = 0;
    while (getline(file, line))
    {
        ++lineNum;
        stringstream ss(line);
        string cell;
        vector<double> row;
        while (getline(ss, cell, ','))
        {
            try
            {
                row.push_back(stod(cell));
            } catch (const std::invalid_argument& e)
            {
                cerr << "Error: Invalid argument at line " << lineNum << ", column: " << row.size() + 1 << endl;
                exit(1);
            }
        }
        values.push_back(row);
        if (numCols == 0)
            numCols = row.size();
        else if (row.size() != numCols)
        {
            cerr << "Error: Inconsistent number of columns in the CSV file." << endl;
            exit(1);
        }
    }

    if (values.empty())
    {
        cerr << "Error: CSV file is empty." << endl;
        exit(1);
    }

    // Create Eigen Matrix
    Eigen::MatrixXd mat(values.size(), numCols);
    for (int i = 0; i < values.size(); ++i)
    {
        for (int j = 0; j < numCols; ++j)
        {
            mat(i, j) = values[i][j];
        }
    }

    return mat;
}

Eigen::Matrix3d R3_to_so3(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d skewSym;
    skewSym <<     0, -v(2),  v(1),
                 v(2),    0, -v(0),
                -v(1), v(0),     0;
    return skewSym;
}

Eigen::Matrix3d R3_to_SO3(const Eigen::Vector3d& axis_angle)
{
    // Normalize the axis of rotation
    Eigen::Vector3d axis = axis_angle.normalized();

    // Compute the angle of rotation
    double angle = axis_angle.norm();

    // Compute the skew-symmetric matrix
    Eigen::Matrix3d skew_sym = R3_to_so3(axis);

    // Compute the rotation matrix using Rodriguez formula
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity()
                                      + std::sin(angle) * skew_sym
                                      + (1 - std::cos(angle)) * (skew_sym * skew_sym);

    // Perform Gram-Schmidt orthogonalization to ensure orthogonality
    //    Eigen::HouseholderQR<Eigen::Matrix3d> qr(rotation_matrix);
    //    rotation_matrix = qr.householderQ();

    return rotation_matrix;
}

Eigen::Vector3d so3_to_R3(const Eigen::Matrix3d& skewSym)
{
    Eigen::Vector3d v;
    v << skewSym(2, 1), skewSym(0, 2), skewSym(1, 0);
    return v;
}

Eigen::Matrix3d SO3_to_so3(const Eigen::Matrix3d& R_del )
{
    Eigen::Matrix3d w_axis_mat;

    if (std::abs(R_del.trace() + 1) <= 1e-7)
    {
        if (std::abs(R_del(2, 2) + 1) >= 1e-7)
        {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1.0 / sqrt(2 * (1 + R_del(2, 2))) * R_del(0, 2);
            tmp(1) = 1.0 / sqrt(2 * (1 + R_del(2, 2))) * R_del(1, 2);
            tmp(2) = 1.0 / sqrt(2 * (1 + R_del(2, 2))) * (1 + R_del(2, 2));
            w_axis_mat = R3_to_so3(tmp);
        }
        else if (std::abs(R_del(1, 1) + 1) >= 1e-7)
        {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1.0 / sqrt(2 * (1 + R_del(1, 1))) * R_del(0, 1);
            tmp(1) = 1.0 / sqrt(2 * (1 + R_del(1, 1))) * (1 + R_del(1, 1));
            tmp(2) = 1.0 / sqrt(2 * (1 + R_del(1, 1))) * R_del(2, 1);
            w_axis_mat = R3_to_so3(tmp);
        }
        else
        {
            Eigen::Vector3d tmp = Eigen::Vector3d::Zero();
            tmp(0) = 1.0 / sqrt(2 * (1 + R_del(0, 0))) * (1 + R_del(0, 0));
            tmp(1) = 1.0 / sqrt(2 * (1 + R_del(0, 0))) * R_del(1, 0);
            tmp(2) = 1.0 / sqrt(2 * (1 + R_del(0, 0))) * R_del(2, 0);
            w_axis_mat = R3_to_so3(tmp);
        }
    }
    else
    {
        // Calculate theta
        Eigen::Matrix3d diff = R_del - Eigen::Matrix3d::Identity();

        if ( std::abs( diff.norm( ) ) < 1e-6 )
        {
            w_axis_mat = Eigen::Matrix3d::Zero(3, 3);
        }
        else
        {
            double theta = std::acos( 0.5* ( R_del.trace( ) - 1 ) );
            w_axis_mat = theta*( R_del - R_del.transpose( ) ) / (2 * sin(theta));
        }
    }

    return w_axis_mat;
}

//******************************************************************************
MyLBRClient::MyLBRClient(double freqHz, double amplitude)
{

    /** Initialization */
    // !! WARNING !!
    // THESE JOINT POSITION VALUES MUST BE THE SAME WITH THE JAVA APPLICATION!!
    q_init[0] =  -3.21 * M_PI/180;
    q_init[1] =  46.19 * M_PI/180;
    q_init[2] =  17.52 * M_PI/180;
    q_init[3] = -87.16 * M_PI/180;
    q_init[4] =  -5.03 * M_PI/180;
    q_init[5] = -37.73 * M_PI/180;
    q_init[6] =  0.000 * M_PI/180;

    // Use Explicit-cpp to create your robot
    myLBR = new iiwa14( 1, "Dwight", Eigen::Vector3d( 0.0, 0.0, 0.20 ) );

    // Initialization must be called!!
    myLBR->init( );

    // Current position and velocity
    // These two variables are used as "Eigen" objects rather than a double array
    q  = Eigen::VectorXd::Zero( myLBR->nq );
    dq = Eigen::VectorXd::Zero( myLBR->nq );
    q0_init = Eigen::VectorXd::Zero( myLBR->nq );

    // Time variables for control loop
    t         = 0.0;     // The current Time
    ts        = 0.0;     // The  sample Time
    n_step    = 0.0;     // The number of time steps, integer
    amp       = 0.0;

    // Initialize joint torques and joint positions (also needed for waitForCommand()!)
    for( int i=0; i < myLBR->nq; i++ )
    {
        q( i ) = q_init[ i ];
        q_curr[ i ]  = q_init[ i ];
         q_old[ i ]  = q_init[ i ];
        q0_init[ i ] = q_init[ i ];

         // The Actual command of the joint-position and torque
          q_command[ i ] = 0.0;
        tau_command[ i ] = 0.0;
    }

    // Forward Kinematics and the current position
    H = myLBR->getForwardKinematics( q );
    p_init  = H.block< 3, 1 >( 0, 3 );
    R_init  = H.block< 3, 3 >( 0, 0 );

    p_curr  = p_init;
    R_curr  = R_init;

    dp_curr = Eigen::VectorXd::Zero( 3 );

    // Set the Rdesired postures
    R_init_des << -1.0, 0.0,  0.0,
                   0.0, 1.0,  0.0,
                   0.0, 0.0, -1.0;

    Eigen::Vector3d wdel = so3_to_R3( SO3_to_so3( R_init.transpose( ) * R_init_des ) );
    mjt_w_init  = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ),  wdel, 2.0, 2.0 );
    mjt_p1_init = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d( -0.38, -0.3,  0.00 ), 2.0, 2.0 );
    mjt_p2_init = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d(  0.00,  0.0, -0.53 ), 3.0, 3.0 );
    mjt_p3_init = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d(  0.00,  0.0,  0.24 ), 3.0, 2.0 );
    mjt_p4_init = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d(  0.00,  0.0, -0.24 ), 3.0, 2.0 );
    mjt_p5_init = new MinimumJerkTrajectory( 3, Eigen::Vector3d( 0.0, 0.0, 0.0 ), Eigen::Vector3d(  0.00,  0.0,  0.53 ), 3.0, 2.0 );

    // The taus (or torques) for the command
    tau_ctrl   = Eigen::VectorXd::Zero( myLBR->nq );    // The torque from the controller design,
    tau_prev   = Eigen::VectorXd::Zero( myLBR->nq );    // The previous torque         , i.e., tau_{n-1} where tau_n is current value
    tau_pprev  = Eigen::VectorXd::Zero( myLBR->nq );    // The previous-previous torque, i.e., tau_{n-2} where tau_n is current value
    tau_total  = Eigen::VectorXd::Zero( myLBR->nq );    // The total tau which will be commanded

    tau_imp1   = Eigen::VectorXd::Zero( myLBR->nq );    // Position    Task-space  Impedance Control
    tau_imp2   = Eigen::VectorXd::Zero( myLBR->nq );    // Orientation Task-space  Impedance Control
    tau_imp3   = Eigen::VectorXd::Zero( myLBR->nq );    //             Joint-space Impedance Control

    // For the Task-space impedance control, the Forward Kinematics (H) and Hybrid Jacobian Matrix (JH) is Required
    H  = Eigen::Matrix4d::Zero( 4, 4 );
    J  = Eigen::MatrixXd::Zero( 6, myLBR->nq );
    Jp = Eigen::MatrixXd::Zero( 3, myLBR->nq );
    Jr = Eigen::MatrixXd::Zero( 3, myLBR->nq );

    // The stiffness/damping matrices
    Kp = 400 * Eigen::MatrixXd::Identity( 3, 3 );
    Bp =  40 * Eigen::MatrixXd::Identity( 3, 3 );

    Kp( 2, 2 ) = 500;

    Kr =  70 * Eigen::MatrixXd::Identity( 3, 3 );
    Br =   7 * Eigen::MatrixXd::Identity( 3, 3 );

    Kq = 6.0 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );
    Bq = 4.5 * Eigen::MatrixXd::Identity( myLBR->nq, myLBR->nq );

    printf( "=============== Drawing Task =============== \n" );

    is_1st_pressed    = false;
    is_2nd_pressed    = false;
    is_forward_done   = false;
    is_backward_done  = false;
    is_backward2_done = false;

    t_1st_pressed   = 0.0;
    t_2nd_pressed   = 0.0;
    t_forward_done  = 0.0;
    t_backward_done = 0.0;

    // Reading the data
    pos_data_forward   = readCSV( "/home/baxterplayground/Documents/iiwa14_drawing_task/data_output/AB_write_forward.csv" );
    pos_data_backward  = readCSV( "/home/baxterplayground/Documents/iiwa14_drawing_task/data_output/AB_write_backward.csv" );
    pos_data_backward2 = readCSV( "/home/baxterplayground/Documents/iiwa14_drawing_task/data_output/AB_write_backward2.csv" );

    std::cout << "[Forward Movement] Matrix size: " << pos_data_forward.rows() << " rows x " << pos_data_forward.cols() << " columns" << std::endl;
    N_data_forward = pos_data_forward.cols( );
    N_curr_forward = 0;

    std::cout << "[Backward Movement] Matrix size: " << pos_data_backward.rows() << " rows x " << pos_data_backward.cols() << " columns" << std::endl;
    N_data_backward = pos_data_backward.cols( );
    N_curr_backward = N_data_backward-1;

    std::cout << "[Backward Movement] Matrix size: " << pos_data_backward2.rows() << " rows x " << pos_data_backward2.cols() << " columns" << std::endl;
    N_data_backward2 = pos_data_backward2.cols( );
    N_curr_backward2 = 0;

    // Saving the Data
    fw.open(  "forward_data.txt"   );
    fb.open(  "backward_data.txt"  );
    fb2.open( "backward_data2.txt" );
    fmt = Eigen::IOFormat(5, 0, ", ", "\n", "[", "]");


}


/**
* \brief Destructor
*
*/
MyLBRClient::~MyLBRClient()
{
}

/**
* \brief Implements an IIR Filter which is used to send the previous joint position to the command function, so that KUKA's internal friction compensation can be activated. The filter function was generated by the application WinFilter (http://www.winfilter.20m.com/).
*
* @param NewSample The current joint position to be provided as input to the filter.
*/
// iir updates the filterInput and filterOutput
void iir(double NewSample[7])
{
    double ACoef[ NCoef+1 ] =
    {
        0.05921059165970496400,
        0.05921059165970496400
    };

    double BCoef[ NCoef+1 ] =
    {
        1.00000000000000000000,
        -0.88161859236318907000
    };

    // Shift the old samples
    for ( int i=0; i<7; i++ )
    {
        for( int n=NCoef; n>0; n-- )
        {
             filterInput[ i ][ n ] =  filterInput[ i ][ n-1 ];
            filterOutput[ i ][ n ] = filterOutput[ i ][ n-1 ];
        }
    }

    // Calculate the new output
    for ( int i=0; i<7; i++ )
    {
         filterInput[ i ][ 0 ] = NewSample[ i ];
        filterOutput[ i ][ 0 ] = ACoef[ 0 ] * filterInput[ i ][ 0 ];
    }

    for (int i=0; i<7; i++)
    {
        for( int n=1; n<=NCoef; n++ )
        {
            filterOutput[ i ][ 0 ] += ACoef[ n ] * filterInput[ i ][ n ] - BCoef[ n ] * filterOutput[ i ][ n ];
        }
    }
}

//******************************************************************************
void MyLBRClient::onStateChange( ESessionState oldState, ESessionState newState )
{
    LBRClient::onStateChange( oldState, newState );

    // react on state change events
    switch (newState)
    {
        case MONITORING_WAIT:
        {
            break;
        }
        case MONITORING_READY:
        {
            ts = robotState( ).getSampleTime( );
            break;
        }
        case COMMANDING_WAIT:
        {
            break;
        }
        case COMMANDING_ACTIVE:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

//******************************************************************************
void MyLBRClient::monitor()
{

    // Copied from FRIClient.cpp
    robotCommand( ).setJointPosition( robotState( ).getCommandedJointPosition( ) );

    // Copy measured joint positions (radians) to q_curr, which is a double
    memcpy( q_curr, robotState( ).getMeasuredJointPosition( ), 7*sizeof(double) );

    // Initialize the q for the previous NCoef timesteps
    for( int i=0; i<NCoef+1; i++ )
    {
        iir( q_curr );
    }

}

//******************************************************************************
void MyLBRClient::waitForCommand()
{
    // If we want to command torques, we have to command them all the time; even in
    // waitForCommand(). This has to be done due to consistency checks. In this state it is
    // only necessary, that some torque vlaues are sent. The LBR does not take the
    // specific value into account.

    if(robotState().getClientCommandMode() == TORQUE)
    {
        robotCommand( ).setTorque( tau_command );
        robotCommand( ).setJointPosition( robotState( ).getIpoJointPosition( ) );            // Just overlaying same position
    }

}

//******************************************************************************
void MyLBRClient::command()
{

    // ************************************************************
    // Get robot measurements
    memcpy( q_old, q_curr, 7*sizeof( double ) );
    memcpy( q_curr, robotState( ).getMeasuredJointPosition( ), 7*sizeof(double) );

    for (int i=0; i < myLBR->nq; i++)
    {
        q[ i ] = q_curr[ i ];
    }

    for (int i=0; i < 7; i++)
    {
        dq[ i ] = ( q_curr[ i ] - q_old[ i ]) / ts;
    }

    // ************************************************************ //
    // ********************* CONTROLLER START ********************* //
    // ************************************************************ //

    H = myLBR->getForwardKinematics( q );
    p_curr = H.block< 3, 1 >( 0, 3 );
    R_curr = H.block< 3, 3 >( 0, 0 );

    // Get the current end-effector velocity
    // Hybrid Jacobian Matrix (6x7) and its linear velocity part (3x7)
    J  = myLBR->getHybridJacobian( q );

    Jp = J.block( 0, 0, 3, myLBR->nq );
    Jr = J.block( 3, 0, 3, myLBR->nq );

    // Calculate the current end-effector's position
    dp_curr = Jp * dq;

    // Calculate the desired orientation
    w0_init = mjt_w_init->getPosition( t );
    R_des   = R_init * R3_to_SO3( w0_init );

    p0 = p_init + mjt_p1_init->getPosition( t )+ mjt_p2_init->getPosition( t );

    // Start the update
    if( is_1st_pressed && !is_2nd_pressed && !is_forward_done )
    {
        // Conduct the Forward Movement
        if( t_1st_pressed >= 2.0 )
        {
            N_curr_forward++;

            if( N_curr_forward >= N_data_forward )
            {
                N_curr_forward = N_data_forward - 1;
                is_forward_done = true;
            }
        }
    }

    // Add the forward movement
    Eigen::Vector2d tmp_forward = pos_data_forward.col( N_curr_forward );
    p0( 0 ) += tmp_forward( 0 );
    p0( 1 ) += tmp_forward( 1 );

    if( is_1st_pressed && is_2nd_pressed && !is_backward_done )
    {
        // Conduct the Forward Movement
        if( t_2nd_pressed >= 6.0 )
        {
            N_curr_backward--;

            if( N_curr_backward <= 0 )
            {
                N_curr_backward = 0;
                is_backward_done = true;
            }
        }
    }

    // Add the forward movement
    Eigen::Vector2d tmp_back = pos_data_backward.col( N_curr_backward );

    p0( 0 ) -= tmp_back( 0 );
    p0( 1 ) -= tmp_back( 1 );

    if( is_backward_done && !is_backward2_done )
    {
        // Conduct the Forward Movement
        N_curr_backward2++;

        if( N_curr_backward2 >= N_data_backward2 )
        {
            N_curr_backward2 = N_data_backward2 - 1;
            is_backward2_done = true;
        }
    }

    Eigen::Vector2d tmp_back2 = pos_data_backward2.col( N_curr_backward2 );

    p0( 0 ) += tmp_back2( 0 );
    p0( 1 ) += tmp_back2( 1 );

    // Lift up the movement when done
    p0 += mjt_p3_init->getPosition( t_forward_done );

    // Lift down to run the movement
    p0 += mjt_p4_init->getPosition( t_2nd_pressed );

    // Lift down to run the movement
    p0 += mjt_p5_init->getPosition( t_backward_done );

    // The difference between the two rotation matrices
    R_del   = R_curr.transpose( ) * R_des;
    w_axis = so3_to_R3( SO3_to_so3( R_del ) );

    tau_imp1 = Jp.transpose( ) * ( Kp * ( p0 - p_curr ) + Bp * ( - dp_curr ) );
    tau_imp2 = Bq * ( -dq );
    tau_imp3 = Jr.transpose( ) * ( Kr * R_curr * w_axis - Br * Jr * dq );

    // Superposition of Mechanical Impedances
    tau_ctrl = tau_imp1 + tau_imp2 + tau_imp3;

    // ************************************************************ //
    // ********************* CONTROLLER ENDS ********************* //
    // ************************************************************ //

    // A simple filter for the torque command
    tau_total = ( tau_ctrl + tau_prev + tau_pprev ) / 3;

    for ( int i=0; i<7; i++ )
    {
          q_command[ i ] = filterOutput[ i ][ 0 ];
        tau_command[ i ] = tau_total[ i ];
    }

    // Command values (must be double arrays!)
    if ( robotState().getClientCommandMode( ) == TORQUE )
    {
        robotCommand( ).setJointPosition( q_command );
        robotCommand( ).setTorque( tau_command );
    }

    // IIR filter input
    iir( q_curr );

    // If the first-step of the controller
    if ( n_step == 0 )
    {
        tau_prev  = tau_ctrl;
        tau_pprev = tau_ctrl;
    }
    else
    {
        tau_prev  = tau_ctrl;
        tau_pprev = tau_prev;
    }


    // Check if button Pressed for the First Time
    if( robotState().getBooleanIOValue( "MediaFlange.UserButton" ) && !is_1st_pressed && !is_2nd_pressed)
    {
        is_1st_pressed = true;

        // Turn on imitation learning
        std::cout << "1st Button Pressed!" << std::endl;
    }

    if( robotState().getBooleanIOValue( "MediaFlange.UserButton" ) && is_1st_pressed && is_forward_done && !is_2nd_pressed )
    {
        is_2nd_pressed = true;

        // Turn on imitation learning
        std::cout << "2nd Button Pressed!" << std::endl;
        Kp( 2, 2 ) = 600;
    }

    // If the counter reaches the threshold, print to console
    if (  ( n_step % 5 ) == 0 && is_1st_pressed && !is_forward_done )
    {
        fw << "Time: " << std::fixed << std::setw( 5 ) << t;
        fw << " Joint Angle " << q.transpose( ).format( fmt );
        fw << " Virtual Position " << p0.transpose( ).format( fmt );
        fw << std::endl;

        //        std::cout << "Elapsed time for The Torque Calculation "
        //                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        //                  << " us" << std::endl;
    }


    // If the counter reaches the threshold, print to console
    if (  ( n_step % 5 ) == 0 && is_2nd_pressed && !is_backward_done )
    {
        fb << "Time: " << std::fixed << std::setw( 5 ) << t;
        fb << " Joint Angle " << q.transpose( ).format( fmt ) ;
        fb<< " Virtual Position " << p0.transpose( ).format( fmt );
        fb << std::endl;

        //        std::cout << "Elapsed time for The Torque Calculation "
        //                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        //                  << " us" << std::endl;
    }

    // If the counter reaches the threshold, print to console
    if (  ( n_step % 5 ) == 0 && is_2nd_pressed && !is_backward2_done )
    {
        fb2 << "Time: " << std::fixed << std::setw( 5 ) << t;
        fb2 << " Joint Angle " << q.transpose( ).format( fmt ) ;
        fb2 << " Virtual Position " << p0.transpose( ).format( fmt );
        fb2 << std::endl;

        //        std::cout << "Elapsed time for The Torque Calculation "
        //                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
        //                  << " us" << std::endl;
    }


    // 1st Button pressed
    if( is_1st_pressed )
    {
        t_1st_pressed += ts;
    }

    // 2nd Button pressed
    if( is_2nd_pressed )
    {
        t_2nd_pressed += ts;
    }

    if( is_forward_done )
    {
        t_forward_done += ts;
    }

    if( is_backward2_done )
    {
        t_backward_done += ts;
    }

    // Add the sample time to the current time
    t += ts;
    n_step++;

}
