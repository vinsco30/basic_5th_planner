#include "planner.hpp"
using namespace std;

//--------------------------5 order--------------------------------------------
// AX = B
// A = inv(X)*B - here we compute matrix A
Eigen::MatrixXd computeQuinticCoeff(double t0, double tf, std::vector<double> vec_q0, std::vector<double> vec_qf)
{

    Eigen::MatrixXd X(6, 6);
    Eigen::MatrixXd B(6, 1);

    X(0, 0) = 1;
    X(0, 1) = t0;
    X(0, 2) = std::pow(t0, 2);
    X(0, 3) = std::pow(t0, 3);
    X(0, 4) = std::pow(t0, 4);
    X(0, 5) = std::pow(t0, 5);

    X(1, 0) = 0;
    X(1, 1) = 1;
    X(1, 2) = 2 * t0;
    X(1, 3) = 3 * std::pow(t0, 2);
    X(1, 4) = 4 * std::pow(t0, 3);
    X(1, 5) = 5 * std::pow(t0, 4);

    X(2, 0) = 0;
    X(2, 1) = 0;
    X(2, 2) = 2;
    X(2, 3) = 6 * t0;
    X(2, 4) = 12 * std::pow(t0, 2);
    X(2, 5) = 20 * std::pow(t0, 3);

    X(3, 0) = 1;
    X(3, 1) = tf;
    X(3, 2) = std::pow(tf, 2);
    X(3, 3) = std::pow(tf, 3);
    X(3, 4) = std::pow(tf, 4);
    X(3, 5) = std::pow(tf, 5);

    X(4, 0) = 0;
    X(4, 1) = 1;
    X(4, 2) = 2 * tf;
    X(4, 3) = 3 * std::pow(tf, 2);
    X(4, 4) = 4 * std::pow(tf, 3);
    X(4, 5) = 5 * std::pow(tf, 4);

    X(5, 0) = 0;
    X(5, 1) = 0;
    X(5, 2) = 2;
    X(5, 3) = 6 * tf;
    X(5, 4) = 12 * std::pow(tf, 2);
    X(5, 5) = 20 * std::pow(tf, 3);

    B(0, 0) = vec_q0[0];
    B(1, 0) = vec_q0[1];
    B(2, 0) = vec_q0[2];
    B(3, 0) = vec_qf[0];
    B(4, 0) = vec_qf[1];
    B(5, 0) = vec_qf[2];

    return (X.inverse() * B);
    
}

//-----------------------5 order-----------------------------------------------
void computeQuinticTraj(Eigen::MatrixXd A, double t0, double tf, int n, std::vector<double> & qd, std::vector<double> & d_qd, std::vector<double> & dd_qd, std::vector<double> & time) {
    std::vector<double> a = {A(0, 0), A(1, 0), A(2, 0), A(3, 0), A(4, 0), A(5, 0)};

    float step = (tf - t0) / n;
    for (float t = t0; t < tf; t += step) {

        float qdi = a[0] + a[1] * t + a[2] * std::pow(t, 2) + a[3] * std::pow(t, 3) + a[4] * std::pow(t, 4) + a[5] * std::pow(t, 5);
        float d_qdi = a[1] + 2 * a[2] * t + 3 * a[3] * std::pow(t, 2) + 4 * a[4] * std::pow(t, 3) + 5 * a[5] * std::pow(t, 4);
        float dd_qdi = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * std::pow(t, 2) + 20 * a[5] * std::pow(t, 3);

        qd.push_back(qdi);
        d_qd.push_back(d_qdi);
        dd_qd.push_back(dd_qdi);
        time.push_back(t);
    }    
}

void QUINTIC_PLANNER::generateTakeOffTraj( const Eigen::Vector3d& start_pos, const Eigen::Vector3d& final_pos, const float cv ) {

    std::vector<double> vec_s0{0.0, 0.0, 0.0};
    
    double tf = ( final_pos-start_pos ).norm() / cv;
    double dt = 0.01;
    double t0 = 0.0;
    double n_points = tf * 1/dt;
    int np = ceil( n_points );
    std::vector<double> vec_sf{( final_pos-start_pos ).norm(), 0.0, 0.0};

    Eigen::MatrixXd traj_A = computeQuinticCoeff( t0, tf,  vec_s0, vec_sf );  
    std::vector<double> s, d_s, dd_s, time_s;

    computeQuinticTraj( traj_A, t0, tf, np, s, d_s, dd_s, time_s );
    std::vector<double> xd, yd, zd, d_xd, d_yd, d_zd, dd_xd, dd_yd, dd_zd;

    for( int i=0; i<s.size(); i++ ) {
        xd.push_back(start_pos(0) + (s[i]/( final_pos-start_pos ).norm())*
            ( final_pos(0)-start_pos(0) ));
        yd.push_back(start_pos(1) + (s[i]/( final_pos-start_pos ).norm())*
            ( final_pos(1)-start_pos(1) ));
        zd.push_back(start_pos(2) + (s[i]/( final_pos-start_pos ).norm())*
            ( final_pos(2)-start_pos(2) ));
        d_xd.push_back(d_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(0)-start_pos(0) ));
        d_yd.push_back(d_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(1)-start_pos(1) ));
        d_zd.push_back(d_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(2)-start_pos(2) ));
        dd_xd.push_back(dd_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(0)-start_pos(0) ));
        dd_yd.push_back(dd_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(1)-start_pos(1) ));
        dd_zd.push_back(dd_s[i]/( final_pos-start_pos ).norm()*
            ( final_pos(2)-start_pos(2) ));
    }
    int j = 0;
    _trajectory_execution = false;
    while( j<xd.size() && !_stop_trajectory ) {
        _pos_cmd(0) = xd[j];
        _pos_cmd(1) = yd[j];
        _pos_cmd(2) = zd[j];
        _vel_cmd(0) = d_xd[j];
        _vel_cmd(1) = d_yd[j];
        _vel_cmd(2) = d_zd[j];
        _acc_cmd(0) = dd_xd[j];
        _acc_cmd(1) = dd_yd[j];
        _acc_cmd(2) = dd_zd[j];

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        j++;
        _trajectory_execution = true;
    }
    _trajectory_execution = false;
    _stop_trajectory = false;  

}

void QUINTIC_PLANNER::generateGoToTraj( const Eigen::Vector3d& start_pos, const float& start_yaw, const Eigen::Vector3d& final_pos, const float & final_yaw, const float cv ) {

    //Yaw heading to the target point trajectory
    Eigen::Vector2d heading = final_pos.head(2) - start_pos.head(2);
    heading /= heading.norm();
    double yaw_first = atan2(heading(1), heading(0));
    std::cout<<"\n Yaw towards the target point: "<<yaw_first<<" rad\n";

    //Control to avoid high yaw changes
    if ( yaw_first >= -M_PI && yaw_first <= -M_PI_2 ) {
        if (start_yaw >= M_PI_2 && start_yaw <= M_PI ) {
            yaw_first += 2*M_PI; 
            cout<<"\n First case Heading angle now: "<<yaw_first<<endl;
        }
    }
    if ( yaw_first >= M_PI_2 && yaw_first <= M_PI ) {
        if ( start_yaw >= -M_PI && start_yaw <= -M_PI_2 ) {
            yaw_first -= 2*M_PI;
            cout<<"\n Second case, Heading angle: "<<yaw_first<<endl;
        }
    
    }

    std::vector<double> vec_yaw_s0{0.0, 0.0, 0.0};
    std::vector<double> vec_yaw_sf{yaw_first, 0.0, 0.0};

    double dt = 0.01;
    double t0 = 0.0;
    double tf = fabs( yaw_first - start_yaw ) / _cv_rot;
    double n_points = tf * 1/dt;
    int np = ceil( n_points );
    
    Eigen::MatrixXd traj_A_yaw1 = computeQuinticCoeff( t0, tf,  vec_yaw_s0, vec_yaw_sf );
    std::vector<double> s_yaw, d_s_yaw, dd_s_yaw, time_s_yaw;

    computeQuinticTraj( traj_A_yaw1, t0, tf, np, s_yaw, d_s_yaw, dd_s_yaw, time_s_yaw );

    int j = 0;
    _trajectory_execution = false;
    while( j<s_yaw.size() && !_stop_trajectory ) {
        _yaw_cmd = s_yaw[j];
        _yaw_rate_cmd = d_s_yaw[j];
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        j++;
        _trajectory_execution = true;
    }
    _trajectory_execution = false;
    // std::vector<double> vec_s0{0.0, 0.0, 0.0};

    
    // double tf = ( final_pos-start_pos ).norm() / cv;
    // double dt = 0.01;
    // double t0 = 0.0;
    // double n_points = tf * 1/dt;
    // int np = ceil( n_points );
    // std::vector<double> vec_sf{( final_pos-start_pos ).norm(), 0.0, 0.0};

    // Eigen::MatrixXd traj_A = computeQuinticCoeff( t0, tf,  vec_s0, vec_sf );  
    // std::vector<double> s, d_s, dd_s, time_s;

    // computeQuinticTraj( traj_A, t0, tf, np, s, d_s, dd_s, time_s );
    // std::vector<double> xd, yd, zd, d_xd, d_yd, d_zd, dd_xd, dd_yd, dd_zd;
}
