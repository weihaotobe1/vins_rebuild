#include "visualization2.h"

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const double &header, pgsystem* _pgsystem)
{
	Eigen::Quaterniond quadrotor_Q = Q ;
	MyOdometryMsg odometry;
	odometry.header = header;
	odometry.mP = P;
	odometry.mQ = quadrotor_Q;
	odometry.mV = V;
	_pgsystem->imu_forward_callback(odometry);
}

void pubOdometryTF(const Estimator &estimator, const double &header, pgsystem* _pgsystem)
{
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        MyOdometryMsg odometry;
        odometry.header = header;
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[ESTWINDOW_SIZE]);
        odometry.mP = estimator.Ps[ESTWINDOW_SIZE];
        odometry.mQ = tmp_Q;
        odometry.mV = estimator.Vs[ESTWINDOW_SIZE];
        odometry.tic = estimator.tic[0];
        Quaterniond tmp_q{estimator.ric[0]};
        odometry.qic = tmp_q;
        _pgsystem->vio_callback(odometry);
    }
}

/*void pubOdometry(const Estimator &estimator, const double &header, pgsystem* _pgsystem)
{
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        MyOdometryMsg odometry;
        odometry.header = header;
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[ESTWINDOW_SIZE]);
        odometry.mP = estimator.Ps[ESTWINDOW_SIZE];
        odometry.mQ = tmp_Q;
        odometry.mV = estimator.Vs[ESTWINDOW_SIZE];   
        _pgsystem->vio_callback(odometry); 
	}
}

void pubTF(const Estimator &estimator, const double &header, pgsystem* _pgsystem)
{
	if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    MyOdometryMsg odometry;
    odometry.header = header;
    odometry.mP = estimator.tic[0];
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.mQ = tmp_q;
    _pgsystem->extrinsic_callback(odometry);
}*/

void pubRelocalization(const Estimator &estimator, pgsystem* _pgsystem)
{
	MyOdometryMsg odometry;
    odometry.header = estimator.relo_frame_stamp;
    odometry.mP = estimator.relo_relative_t;
    odometry.mQ = estimator.relo_relative_q;
    odometry.mV[0] = estimator.relo_relative_yaw;
    odometry.mV[1] = estimator.relo_frame_index;
    _pgsystem->relo_relative_pose_callback(odometry);
}

void pubKeyframe(const Estimator &estimator, pgsystem* _pgsystem)
{
	if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = ESTWINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        MyOdometryMsg odometry;
        odometry.header = estimator.Headers[ESTWINDOW_SIZE - 2];
        
        odometry.mP = P;
        odometry.mQ = R;

		_pgsystem->pose_callback(odometry);

        MyPointCloudMsg point_cloud;
        point_cloud.header = estimator.Headers[ESTWINDOW_SIZE - 2];
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < ESTWINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= ESTWINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                Eigen::Vector3d tmpP;
                tmpP[0] = w_pts_i(0);
                tmpP[1] = w_pts_i(1);
                tmpP[2] = w_pts_i(2);
                point_cloud.vp3.push_back(tmpP);

                int imu_j = ESTWINDOW_SIZE - 2 - it_per_id.start_frame;
                Eigen::Vector2d p_2d;
                p_2d[0] = it_per_id.feature_per_frame[imu_j].point.x();
                p_2d[1] = it_per_id.feature_per_frame[imu_j].point.y();
                point_cloud.vp2.push_back(p_2d);
                Eigen::Vector2d p_uv;
                p_uv[0] = it_per_id.feature_per_frame[imu_j].uv.x();
                p_uv[1] = it_per_id.feature_per_frame[imu_j].uv.y();
                point_cloud.vuv.push_back(p_uv);

                point_cloud.vid.push_back(it_per_id.feature_id);

            }

        }
        _pgsystem->point_callback(point_cloud);
    }
}
