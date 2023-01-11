#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (100)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  inline bool is_initialized() {return !imu_need_init_;}
  //inline void get_kf_time(double &time) {time = last_kf_update_time;}

  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  void PropagateState(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, double target_time);
  void RemoveImuMsgsFromPast(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);

  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;

 private:
  void IMU_init(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  //deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;

  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

void ImuProcess::IMU_init(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = imu_buffer.front()->linear_acceleration;
    const auto &gyr_acc = imu_buffer.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : imu_buffer)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);

  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001;
  kf_state.change_P(init_P);
  last_imu_ = imu_buffer.back();
  sensor_msgs::ImuConstPtr first_imu = imu_buffer.front();
  kf_state.set_time(first_imu->header.stamp.toSec());
}

void ImuProcess::PropagateState(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, double target_time) // find replacement for measure group!
{
  if(imu_buffer.empty()) {return;};

  //state_ikfom imu_state = kf_state.get_x();
    
  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(imu_buffer, kf_state, init_iter_num);

    if (init_iter_num > MAX_INI_COUNT)
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);
    }

    return;
  }

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr;

  double dt = 0;

  input_ikfom in;
  auto it_imu = imu_buffer.begin();
  int skip_counter = 0;
  
  while(it_imu < (imu_buffer.end() - 1))
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    // Not used atm because we predict until exact target time
    //if(tail->header.stamp.toSec() > target_time) {
    //  return;
    //}

    // this only makes sense for the next else condition (**)
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z); 
                
    // TODO: this should be dt dependand (0.5 only holds for the prediction within 2 imu frames)

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;
    double last_kf_update_time = kf_state.get_time();
    if(target_time < last_kf_update_time){
      ROS_ERROR("target_time in the past! dt: %f", target_time - last_kf_update_time);
      return;
    }
    else if(head->header.stamp.toSec() < last_kf_update_time && 
       tail->header.stamp.toSec() < last_kf_update_time) // old data 
    {
      skip_counter++;
      ROS_WARN("KF prediction stamp ahead of imu stamp, skipping msg (%i in a row)", skip_counter);
      it_imu += 1; // this line is VERY important
      continue;
    }
    else if(head->header.stamp.toSec() < last_kf_update_time && 
            tail->header.stamp.toSec() > last_kf_update_time)
    {
      dt = tail->header.stamp.toSec() - last_kf_update_time; // closing the gap to the next IMU update
    }
    else if(head->header.stamp.toSec() < target_time && 
      tail->header.stamp.toSec() > target_time) { // precisely matching the target time
      dt = target_time - head->header.stamp.toSec();
    }
    else // head < target time && tail < target_time (**)
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }
    
    //reset skip_counter
    skip_counter = 0;

    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    
    if(dt > 0.1 || dt < 0.0) {
      ROS_WARN("PropagateState(): Suspicious dt = %f", dt);
    }
    
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    state_ikfom imu_state = kf_state.get_x(); // overwriting imu_state after every propagation step
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    
    double &&t = head->header.stamp.toSec() + dt; // && creates temporaries without making a copy, WTF why here?
    IMUpose.push_back(set_pose6d(t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
    kf_state.set_time(t);
    
    // remove head element and set the pointer one further
    //it_imu = imu_buffer.erase(it_imu); // how shall we keep this while not erasing it all?
    it_imu += 1;
    
    if(t >= target_time){ // TODO integrate this more nicely
      break;
      ROS_INFO("Prediction sucess");
    }
  }
}

void ImuProcess::RemoveImuMsgsFromPast(deque<sensor_msgs::Imu::ConstPtr> &imu_buffer, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state){
  auto it_imu = imu_buffer.begin();
  double kf_time = kf_state.get_time();
  
  while(it_imu < (imu_buffer.end() - 1)) // we always want to keep the last msg!
  {
    auto &&msg = *(it_imu);
    
    double msg_time = msg->header.stamp.toSec(); // make sure the we keep the last one before the
    if (msg_time > kf_time){
      break;
    }

    // remove head element and set the pointer one further
    it_imu = imu_buffer.erase(it_imu); // how shall we keep this while not erasing it all?
  }
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/

  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  // move IMUpose to the start of the Lidar measurement window
  if(IMUpose.empty()){
    ROS_ERROR("IMUpose buffer empty");
    return;
  }
  
  if(IMUpose.front().time > pcl_beg_time){
    ROS_WARN("IMUpose.front().time > pcl_beg_time");
    ROS_WARN("IMUpose.front: %f", IMUpose.front().time);
    ROS_WARN("PCL beg time:  %f", pcl_beg_time);
    ROS_WARN("PCL end time:  %f", pcl_end_time);
  }
  
  /*** calculated the pos and attitude prediction at the frame-end ***/
  double last_kf_update_time = kf_state.get_time();
  double note = pcl_end_time >= last_kf_update_time ? 1.0 : -1.0;
  if (note == -1.0) {
    ROS_WARN("last_kf_update_time > pcl_end_time:");
    ROS_WARN("PCL end: %f", pcl_end_time);
    ROS_WARN("KF end:  %f", last_kf_update_time);
  }
  
  double dt = note * (pcl_end_time - last_kf_update_time);
  //ROS_INFO_THROTTLE(1,"dt KF to pcl_end time: %f", dt);

  //kf_state.predict(dt, Q, in);

  state_ikfom imu_state = kf_state.get_x(); // TODO might propagate to the last state?
  //last_imu_ = IMUpose.back(); // most current imu measurement (within the lidar measurement window)
  last_lidar_end_time = pcl_end_time;

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  M3D R_imu;
  V3D acc_imu, vel_imu, pos_imu, angvel_avr;


  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) + pcl_beg_time> head->time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000) + pcl_beg_time - head->time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
  
  // clear but keep the last element for the next iteration
  Pose6D tmp_pose = IMUpose.back();
  IMUpose.clear(); // empty after update
  IMUpose.push_back(tmp_pose);
}
