// This node uses simulated data of the actuators to compute the AUV dynamic
// behavior. This node can be used to simulate real AUV behavior and its
// interaction
// with the environtment. User can add currents and a preliminary version of
// collision
// detection has been implemented.

#include <ros/ros.h>
// import tf

// Messages
#include <auv_msgs/BodyForceReq.h>
#include <cola2_msgs/Setpoints.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>

// Services
#include <cola2_msgs/SimulatedCurrents.h>

// Other
#include <cola2_lib/cola2_util.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

// # Custom libs
// from cola2_lib import cola2_lib, cola2_ros_lib

namespace Eigen
{
// using Vector6d = Matrix<double, 6, 1>;
// using Matrix6d = Matrix<double, 6, 6>;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
}

/*!
   \brief Make the skew-symmetric matrix representation of the vector for
   cross-product.
   \param x The first vector of a cross product.
   \return The skew-symmetric matrix for cross product.
*/
// Eigen::Matrix3d __s__(const Eigen::Vector3d &x) const
Eigen::Matrix3d crossMatrix(const Eigen::Vector3d &x)
{
  Eigen::Matrix3d x_hat;
  x_hat << 0, -x(2), x(1), x(2), 0, -x(0), -x(1), x(0), 0;
  return x_hat;
}

double randomNormal()
{
  // Seed with a real random value, if available
  static std::random_device r;
  static std::default_random_engine e(r());
  static std::normal_distribution<> normal_dist(0.0, 1.0);
  return normal_dist(e);
}

Eigen::Quaterniond euler2quaternion(const Eigen::Vector3d &rpy)
{
  return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d euler2rotation(const Eigen::Vector3d &rpy)
{
  return Eigen::Matrix3d(euler2quaternion(rpy));
}

std::vector<double> getParamVector(const ros::NodeHandle &nh, const std::string &tname)
{
  std::vector<double> vec;
  nh.getParam(tname, vec);
  return vec;
}

void getParamVector3d(const ros::NodeHandle &nh, const std::string &tname, Eigen::Vector3d &v)
{
  // Get raw
  std::vector<double> vec;
  nh.getParam(tname, vec);
  // Construct
  assert(vec.size() == 3);
  for (int i = 0; i < 3; i++)
  {
    v(i) = vec[i];
  }
}
void getParamVector6d(const ros::NodeHandle &nh, const std::string &tname, Eigen::Vector6d &v)
{
  // Get raw
  std::vector<double> vec;
  nh.getParam(tname, vec);
  // Construct
  assert(vec.size() == 6);
  for (int i = 0; i < 6; i++)
  {
    v(i) = vec[i];
  }
}
void getParamMatrix3d(const ros::NodeHandle &nh, const std::string &tname, Eigen::Matrix3d &v)
{
  // Get raw
  std::vector<double> vec;
  nh.getParam(tname, vec);
  // Construct
  assert(vec.size() == 9);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      v(i, j) = vec[i * 3 + j];
    }
  }
}
void getParamMatrixXd(const ros::NodeHandle &nh, const std::string &tname, Eigen::MatrixXd &v, int rows)
{
  // Get raw
  std::cout << "getMatrix " << std::endl;
  std::cout << tname << std::endl;
  std::vector<double> vec;
  std::cout << "getMatrix " << std::endl;
  nh.getParam(tname, vec);
  std::cout << "getMatrix " << std::endl;
  std::cout << vec.size() << std::endl;
  // Construct
  assert(vec.size() % rows == 0);
  int cols = vec.size() / rows;
  v = Eigen::MatrixXd::Zero(rows, cols);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      v(i, j) = vec[i * cols + j];
    }
  }
}

/*!
   \brief Simulates the dynamics of an AUV from thrusters rpm and fins angles.
*/
class Dynamics
{
private:
  struct Config
  {
    // Force
    std::string force_topic_;
    bool use_force_topic_ = false;
    // Thrusters
    std::string thrusters_topic_;
    int thrusters_num_ = 0;
    Eigen::MatrixXd thrusters_matrix_;
    double max_thrusters_rpm_;
    // Forward and backward thrusters coeff
    double ctf_;
    double ctb_;
    double dzv_;
    double dv_;
    double dh_;
    // Fins
    std::string fins_topic_;
    int fins_num_ = 0;
    double a_fins_;
    double k_cd_fins_;
    double k_cl_fins_;
    double max_fins_angle_;
    // Other
    double period_;
    double rate_;
    std::string frame_id_;
    std::string world_frame_id_;
    // Contact sensor
    std::string collisions_topic_;
    bool contact_sensor_available_ = false;
    // Body
    std::string odom_topic_;
    double mass_;
    double buoyancy_;
    double g_;
    double radius_;
    double water_density_;  // water density
    Eigen::Matrix3d tensor_;
    Eigen::Vector3d gravity_center_;
    Eigen::Vector6d damping_;
    Eigen::Vector6d quadratic_damping_;
    Eigen::Vector6d p0_;
    Eigen::Vector6d v0_;
    double sea_bottom_depth_ = 10.0;
    // Currents
    Eigen::Vector3d current_mean_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_sigma_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_min_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_max_ = Eigen::Vector3d::Zero();
    bool current_enabled_ = false;
  };

  // Config
  Config config_;
  // Eigen
  Eigen::Vector6d p_, p_dot_;
  Eigen::Vector6d v_, v_dot_;
  Eigen::Vector6d collision_force_;
  Eigen::Matrix6d M_, IM_;
  Eigen::VectorXd u_, old_u_;
  Eigen::VectorXd f_, old_f_;
  // Messages
  auv_msgs::BodyForceReq force_;
  // Node handler
  ros::NodeHandle nh_;
  // Publishers
  ros::Publisher pub_odom_;
  ros::Publisher pub_odom_gazebo_;
  // Subscribers
  ros::Subscriber sub_collision_;
  ros::Subscriber sub_fins_;
  ros::Subscriber sub_force_;
  ros::Subscriber sub_thrusters_;
  // Transforms
  tf2_ros::TransformBroadcaster tfbr_;
  // Services
  ros::ServiceServer srv_current_;

public:
  Dynamics()
  {
    std::cout << "Dynamics()" << std::endl;
    // Load dynamic parameters
    getConfig();
    // Initialize vars and matrices. They are not init. in the constructor, but
    // readability is improved
    initialize();
    // TODO: delete after debug
    config_.odom_topic_ = "new_dynamics_odom";
    config_.frame_id_ = "new_dynamics_frame";
    // Create publishers
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(config_.odom_topic_, 2);
    pub_odom_gazebo_ = nh_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 2);
    // Create subscribers
    sub_force_ = nh_.subscribe(config_.force_topic_, 1, &Dynamics::cbkForce, this);
    sub_thrusters_ = nh_.subscribe(config_.thrusters_topic_, 1, &Dynamics::cbkThrusters, this);
    // Create services
    srv_current_ = nh_.advertiseService("cola2_sim/current_simulation", &Dynamics::srvCurrentSimulation, this);
    // Optional subscribers
    if (config_.fins_num_ > 0)
    {
      sub_fins_ = nh_.subscribe(config_.fins_topic_, 1, &Dynamics::cbkFins, this);
    }
    if (config_.contact_sensor_available_)
    {
      collision_force_ = Eigen::Vector6d::Zero();
      sub_fins_ = nh_.subscribe(config_.collisions_topic_, 1, &Dynamics::cbkCollision, this);
    }
    // Show message
    ROS_INFO("initialized");
  }

  /*!
     \brief Thruster callback, input in rpm.
     \param msg The thrusters message received.
  */
  void cbkThrusters(const cola2_msgs::Setpoints &msg)
  {
    std::cout << "cbkThrusters()" << std::endl;
    old_u_ = u_;
    for (int i = 0; i < config_.thrusters_num_; i++)
    {
      u_(i) = msg.setpoints[i];
      if (u_(i) > +std::abs(config_.max_thrusters_rpm_))
      {
        u_(i) = +std::abs(config_.max_thrusters_rpm_);
      }
      else if (u_(i) < -std::abs(config_.max_thrusters_rpm_))
      {
        u_(i) = -std::abs(config_.max_thrusters_rpm_);
      }
    }
  }

  /*!
     \brief Thruster callback, input in rpm.
     \param msg The thrusters message received.
  */
  void cbkForce(const auv_msgs::BodyForceReq &msg)
  {
    std::cout << "cbkForce()" << std::endl;
    force_ = msg;
  }

  void cbkFins(const cola2_msgs::Setpoints &msg)
  {
    std::cout << "cbkFins()" << std::endl;
    old_f_ = f_;
    for (int i = 0; i < config_.fins_num_; i++)
    {
      f_(i) = msg.setpoints[i];
      if (f_(i) > +std::abs(config_.max_fins_angle_))
      {
        f_(i) = +std::abs(config_.max_fins_angle_);
      }
      else if (f_(i) < -std::abs(config_.max_fins_angle_))
      {
        f_(i) = -std::abs(config_.max_fins_angle_);
      }
    }
  }

  void cbkCollision(const geometry_msgs::WrenchStamped &msg)
  {
    std::cout << "cbkCollision()" << std::endl;
    collision_force_(0) = -msg.wrench.force.x / 10.0;
    collision_force_(1) = -msg.wrench.force.z / 10.0;
    collision_force_(2) = +msg.wrench.force.y / 10.0;
    collision_force_(3) = -msg.wrench.torque.x / 10.;
    collision_force_(4) = -msg.wrench.torque.z / 10.;
    collision_force_(5) = +msg.wrench.torque.y / 10.;
  }

  bool srvCurrentSimulation(cola2_msgs::SimulatedCurrentsRequest &request,
                            cola2_msgs::SimulatedCurrentsResponse &response)
  {
    std::cout << "srvCurrentSimulation()" << std::endl;
    config_.current_enabled_ = request.enabled;
    for (int i = 0; i < 3; i++)
    {
      config_.current_mean_(i) = request.current_mean[i];
      config_.current_sigma_(i) = request.current_sigma[i];
      // check if any sigma is 0.0
      if (config_.current_sigma_(i) <= 0.0)
      {
        config_.current_sigma_(i) = 0.1;
      }
    }
    // Compute Max and min with Full with at half maximum FWHM
    // sigma is not sigma^2
    // FWHM = 2*np.sqrt(2*np.log(2))*sigma
    Eigen::Vector3d fwhm_value = 2 * sqrt(2 * log(2)) * config_.current_sigma_;
    config_.current_max_ = config_.current_mean_ + fwhm_value;
    config_.current_min_ = config_.current_mean_ - fwhm_value;
    return true;
  }

  double getRate() const
  {
    std::cout << "getRate()" << std::endl;
    return config_.rate_;
  }

  /*!
     \brief Initialize vars and matrices.
  */
  void initialize()
  {
    std::cout << "initialize()" << std::endl;
    // Init pose, velocity and rate
    p_ = config_.p0_;
    p_dot_ = Eigen::Vector6d::Zero();
    v_ = config_.v0_;
    v_dot_ = Eigen::Vector6d::Zero();

    // Inertia Tensor. Principal moments of inertia, and products of inertia
    // [kg*m*m]
    // Ixx = self.tensor[0]
    // Ixy = self.tensor[1]
    // Ixz = self.tensor[2]
    // Iyx = self.tensor[3]
    // Iyy = self.tensor[4]
    // Iyz = self.tensor[5]
    // Izx = self.tensor[6]
    // Izy = self.tensor[7]
    // Izz = self.tensor[8]
    // m = self.mass
    // xg = self.gravity_center[0]
    // yg = self.gravity_center[1]
    // zg = self.gravity_center[2]
    // Mrb=[m,     0,      0,      0,      m*zg,       -m*yg,
    //      0,     m,      0,      -m*zg,  0,          m*xg,
    //      0,     0,      m,      m*yg,   -m*xg,      0,
    //      0,     -m*zg,  m*yg,   Ixx,    Ixy,        Ixz,
    //      m*zg,  0,      -m*xg,  Iyx,    Iyy,        Iyz,
    //      -m*yg, m*xg,   0,      Izx,    Izy,        Izz]
    Eigen::Matrix6d Mrb = Eigen::Matrix6d::Zero();
    Mrb.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config_.mass_;
    Mrb.block<3, 3>(3, 0) = crossMatrix(config_.gravity_center_) * config_.mass_;
    Mrb.block<3, 3>(0, 3) = crossMatrix(config_.gravity_center_).transpose() * config_.mass_;
    Mrb.block<3, 3>(3, 3) = config_.tensor_;

    // Inertia matrix of the rigid body
    // Added Mass derivative TODO: This is not a valid added mass matrix!
    // Ma=[m/2,    0,      0,      0,      0,      0,
    //     0,      m/2,    0,      0,      0,      0,
    //     0,      0,      m/2,    0,      0,      0,
    //     0,      0,      0,      0,      0,      0,
    //     0,      0,      0,      0,      0,      0,
    //     0,      0,      0,      0,      0,      0]
    Eigen::Matrix6d Ma = Eigen::Matrix6d::Zero();
    Ma.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * config_.mass_ * 0.5;

    // Mass matrix: Mrb + Ma
    M_ = Mrb + Ma;
    IM_ = M_.inverse();

    // Init currents
    // np.random.seed()
    // #self.e_vc = np.random.normal(self.current_mean, self.current_sigma)

    // Force message
    force_ = auv_msgs::BodyForceReq();

    // Initial thrusters setpoint
    u_ = Eigen::VectorXd::Zero(config_.thrusters_num_);
    old_u_ = u_;  // Previous setpoints

    // Initial fins setpoint
    // f_ = Eigen::VectorXd::Zero(config_.fins_num_);
    f_ = Eigen::Vector2d::Zero();
    old_f_ = f_;  // Previous setpoints
  }

  /*!
     \brief Water currents, returns a velocity.
  */
  Eigen::Vector6d computeCurrents()
  {
    std::cout << "computeCurrents()" << std::endl;
    // Compute random currents
    Eigen::Vector6d ans = Eigen::Vector6d::Zero();
    if (config_.current_enabled_)
    {
      // Current random values
      Eigen::Vector3d t;
      for (int i = 0; i < 3; i++)
      {
        double vc = randomNormal() * config_.current_sigma_(i) + config_.current_mean_(i);
        if (vc > config_.current_max_(i))
        {
          vc = config_.current_max_(i);
        }
        else if (vc < config_.current_min_(i))
        {
          vc = config_.current_min_(i);
        }
        t(i) = vc;
      }
      // Transform to vehicle
      Eigen::Matrix3d rot = euler2rotation(p_.tail(3));
      ans.head(3) = rot.transpose() * t;
    }
    return ans;
  }

  Eigen::Matrix6d dampingMatrix(const Eigen::Vector6d &vel)
  {
    std::cout << "dampingMatrix()" << std::endl;
    Eigen::Matrix6d damp = Eigen::Matrix6d::Zero();
    for (int i = 0; i < 6; i++)
    {
      damp(i, i) = config_.damping_(i) + config_.quadratic_damping_(i) * std::abs(vel(i));
    }
    return damp;
  }

  /*!
     \brief Compute the force of each thruster from rpm.
  */
  Eigen::Vector6d generalizedForce(const Eigen::Vector6d &du) const
  {
    std::cout << "generalizedForce()" << std::endl;
    // Build the signed (lineal/quadratic) thruster coeficient array
    // Signed square of each thruster setpoint
    Eigen::Vector6d duu = du.array() * du.array().abs();
    Eigen::Matrix6d ct = Eigen::Matrix6d::Zero();
    for (int i = 0; i < duu.rows(); i++)
    {
      if (duu(i) >= 0.0)
      {
        // Forward
        ct(i, i) = config_.ctf_;
      }
      else
      {
        // Backward
        ct(i, i) = config_.ctb_;
      }
    }
    Eigen::Matrix6d b = config_.thrusters_matrix_ * ct;

    // # Example of g500
    // #   b2 = [-ct[0],        -ct[1],         .0,             .0, .0,
    // #        .0,             .0,             .0,             .0, ct[4],
    // #        .0,             .0,             -ct[2],         -ct[3], .0,
    // #        .0,             .0,             .0,             .0, .0,
    // #        .0,             .0,             -ct[2]*self.dv, ct[3]*self.dv,
    // .0,
    // #        -ct[0]*self.dh, ct[1]*self.dh,  .0,             .0, .0]
    // #   b2 = np.array(b2).reshape(6,5)

    // The value of t is the generalized force
    return b * duu;
  }

  Eigen::Vector6d computeFins(const Eigen::Vector6d &vel, const Eigen::Vector2d &fins) const
  {
    std::cout << "computeFins()" << std::endl;
    // # New fins model
    // # fins[0] -> left fin
    // # fins[1] -> right fin
    // # February of 2015

    // Water velocity on the fins
    double water_vel = vel(0);
    if (water_vel > 0)
    {
      water_vel = sqrt(vel(0) * vel(0) + (25.0 * vel(0) / (config_.water_density_ * 3.141592 * 0.049 * 0.049)));
    }
    // Compute force
    Eigen::Vector6d f = Eigen::Vector6d::Zero();
    if (config_.fins_num_ > 0)
    {
      f(0) = -(0.5 * config_.water_density_ * config_.a_fins_ * water_vel * std::abs(water_vel) * config_.k_cd_fins_) *
             (std::abs(cos(1 * fins(0))) + std::abs(cos(1 * fins(1))));
      f(1) = 0.0;
      f(2) = +(0.5 * config_.water_density_ * config_.a_fins_ * water_vel * std::abs(water_vel) * config_.k_cl_fins_) *
             (sin(4.5 * fins(0)) + sin(4.5 * fins(1)));
      f(3) = +(0.5 * config_.water_density_ * config_.a_fins_ * water_vel * std::abs(water_vel) * config_.k_cl_fins_) *
             (sin(4.5 * fins(0)) - sin(4.5 * fins(1))) * 0.14;
      f(4) = +(0.5 * config_.water_density_ * config_.a_fins_ * water_vel * std::abs(water_vel) * config_.k_cl_fins_) *
             (sin(4.5 * fins(0)) + sin(4.5 * fins(1))) * 0.65;
      f(5) = 0.0;
    }
    return f;
  }

  Eigen::Matrix6d coriolisMatrix(const Eigen::Vector6d &vel) const
  {
    std::cout << "coriolisMatrix()" << std::endl;
    Eigen::Matrix3d s1 = crossMatrix(M_.block<3, 3>(0, 0) * vel.head(3) + M_.block<3, 3>(0, 3) * vel.tail(3));
    Eigen::Matrix3d s2 = crossMatrix(M_.block<3, 3>(3, 0) * vel.head(3) + M_.block<3, 3>(3, 3) * vel.tail(3));
    Eigen::Matrix6d c = Eigen::Matrix6d::Zero();
    c.block<3, 3>(0, 3) = -s1;
    c.block<3, 3>(3, 0) = -s1;
    c.block<3, 3>(3, 3) = -s2;
    return c;
  }

  /*!
     \brief Gravity and weight matrix.
  */
  Eigen::Vector6d gravity(const Eigen::Vector6d &pos) const
  {
    std::cout << "gravity()" << std::endl;
    // Weight and buoyancy from [Kg] to [N]
    double W = config_.mass_ * config_.g_;
    double B = config_.buoyancy_ * config_.g_;

    // If the vehicle moves out of the water the flotability decreases
    double corr_pos = pos(2) + config_.radius_;  // Corrected z position
    double F = 0.0;
    if (corr_pos >= config_.radius_)
    {
      F = B;
    }
    else if (corr_pos <= -config_.radius_)
    {
      F = 0.0;
    }
    else
    {
      double r2 = pow(config_.radius_, 2.0);
      double total_area = M_PI * r2;
      double c = sqrt(r2 - pow(corr_pos, 2.0));
      double area_segment = atan2(c, corr_pos) * r2;
      double area_triangle = corr_pos * c;
      double area_outside = area_segment - area_triangle;
      F = B * (1.0 - area_outside / total_area);
    }

    // Gravity center position in the robot fixed frame (x',y',z') [m]
    double zg = config_.gravity_center_(2);
    Eigen::Vector6d g;
    g << (W - F) * sin(pos(4)), -(W - F) * cos(pos(4)) * sin(pos(3)), -(W - F) * cos(pos(4)) * cos(pos(3)),
        zg * W * cos(pos(4)) * sin(pos(3)), zg * W * sin(pos(4)), 0.0;
    return g;
  }

  /*!
     \brief Given the setpoint for each thruster, the previous velocity and the
     previous position computes the v_dot.
  */
  Eigen::Vector6d inverseDynamic(const Eigen::Vector6d &pos, const Eigen::Vector6d &vel, const Eigen::VectorXd &u,
                                 const Eigen::Vector2d &fins, const Eigen::Vector6d &current)
  {
    std::cout << "inverseDynamic()" << std::endl;
    Eigen::Vector6d a;
    if (config_.use_force_topic_)
    {
      a << force_.wrench.force.x, force_.wrench.force.y, force_.wrench.force.z, force_.wrench.torque.x,
          force_.wrench.torque.y, force_.wrench.torque.z;
    }
    else
    {
      Eigen::Vector6d t = generalizedForce(u);
      Eigen::Vector6d f = computeFins(vel, fins);
      a = t + f;
    }
    Eigen::Matrix6d c = coriolisMatrix(vel);
    Eigen::Matrix6d d = dampingMatrix(vel + current);
    Eigen::Vector6d g = gravity(pos);
    Eigen::Vector6d c_v = (c - d) * (vel + current);
    Eigen::Vector6d v_dot;
    if (config_.contact_sensor_available_)
    {
      v_dot = IM_ * (a - c_v - g - collision_force_);
    }
    else
    {
      v_dot = IM_ * (a - c_v - g);
    }

    if (config_.contact_sensor_available_)
    {
      for (int i = 0; i < 3; i++)
      {
        if (((collision_force_(i) > 0) && (v_dot(i) > 0)) || ((collision_force_(i) < 0) && (v_dot(i) < 0)))
        {
          // Same sign
          v_dot(i) = 0;
        }
        if (((collision_force_(i) > 0) && (v_(i) > 0)) || ((collision_force_(i) < 0) && (v_(i) < 0)))
        {
          // Same sign
          v_(i) = 0;
        }
      }
    }
    return v_dot;
  }

  /*!
     \brief Given the current velocity and the previous position computes the
     p_dot.
  */
  Eigen::Vector6d kinematics(const Eigen::Vector6d &pos, const Eigen::Vector6d &vel)
  {
    std::cout << "kinematics()" << std::endl;
    std::cout << "  pos" << std::endl << pos << std::endl;
    std::cout << "  vel" << std::endl << vel << std::endl;
    double roll = pos(3);
    double pitch = pos(4);
    double yaw = pos(5);
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Eigen::Matrix3d rec;
    rec << cy * cp, -sy * cr + cy * sp * sr, sy * sr + cy * cr * sp, sy * cp, cy * cr + sr * sp * sy,
        -cy * sr + sp * sy * cr, -sp, cp * sr, cp * cr;
    std::cout << "  rec" << std::endl << rec << std::endl;

    Eigen::Matrix3d to;
    to << 1.0, sr * tan(pitch), cr * tan(pitch), 0.0, cr, -sr, 0.0, sr / cp, cr / cp;
    std::cout << "  to" << std::endl << to << std::endl;

    Eigen::Vector6d p_dot;
    p_dot.head(3) = rec * vel.head(3);
    p_dot.tail(3) = to * vel.tail(3);
    std::cout << "  p_dot" << std::endl << p_dot << std::endl;
    return p_dot;
  }

  /*!
     \brief Main loop operations
  */
  void iterate()
  {
    std::cout << "iterate()" << std::endl;
    std::cout << "  p_" << std::endl << p_ << std::endl;
    std::cout << "  v_" << std::endl << v_ << std::endl;
    std::cout << "  old_u_" << std::endl << old_u_ << std::endl;
    std::cout << "  old_f_" << std::endl << old_f_ << std::endl;
    // Compute current
    Eigen::Vector6d current = computeCurrents();
    std::cout << "  current" << std::endl << current << std::endl;

    // Runge-Kutta, 4th order
    Eigen::Vector6d k1_pos = kinematics(p_, v_);
    Eigen::Vector6d k1_vel = inverseDynamic(p_, v_, old_u_, old_f_, current);
    Eigen::Vector6d k2_pos = kinematics(p_ + config_.period_ * 0.5 * k1_pos, v_ + config_.period_ * 0.5 * k1_vel);
    Eigen::Vector6d k2_vel = inverseDynamic(p_ + config_.period_ * 0.5 * k1_pos, v_ + config_.period_ * 0.5 * k1_vel,
                                            0.5 * (old_u_ + u_), 0.5 * (old_f_ + f_), current);
    Eigen::Vector6d k3_pos = kinematics(p_ + config_.period_ * 0.5 * k2_pos, v_ + config_.period_ * 0.5 * k2_vel);
    Eigen::Vector6d k3_vel = inverseDynamic(p_ + config_.period_ * 0.5 * k2_pos, v_ + config_.period_ * 0.5 * k2_vel,
                                            0.5 * (old_u_ + u_), 0.5 * (old_f_ + f_), current);
    Eigen::Vector6d k4_pos = kinematics(p_ + config_.period_ * k3_pos, v_ + config_.period_ * k3_vel);
    Eigen::Vector6d k4_vel =
        inverseDynamic(p_ + config_.period_ * k3_pos, v_ + config_.period_ * k3_vel, u_, f_, current);

    p_ += config_.period_ / 6.0 * (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos);
    v_ += config_.period_ / 6.0 * (k1_vel + 2.0 * k2_vel + 2.0 * k3_vel + k4_vel);

    p_(3) = cola2::util::normalizeAngle(p_(3));
    p_(4) = cola2::util::normalizeAngle(p_(4));
    p_(5) = cola2::util::normalizeAngle(p_(5));

    // Publish odometry
    pubOdometry();
  }

  void pubOdometry()
  {
    std::cout << "pubOdometry()" << std::endl;
    // Header
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = config_.world_frame_id_;
    odom.child_frame_id = config_.frame_id_;
    // Position
    odom.pose.pose.position.x = p_(0);
    odom.pose.pose.position.y = p_(1);
    odom.pose.pose.position.z = p_(2);
    // Orientation
    Eigen::Quaterniond quat = euler2quaternion(p_.tail(3));
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    // Velocities
    odom.twist.twist.linear.x = v_(0);
    odom.twist.twist.linear.y = v_(1);
    odom.twist.twist.linear.z = v_(2);
    odom.twist.twist.angular.x = v_(3);
    odom.twist.twist.angular.y = v_(4);
    odom.twist.twist.angular.z = v_(5);
    // Publish
    pub_odom_.publish(odom);

    // Broadcast transform
    geometry_msgs::TransformStamped tfmsg;
    tfmsg.header = odom.header;
    tfmsg.child_frame_id = odom.child_frame_id;
    tfmsg.transform.translation.x = odom.pose.pose.position.x;
    tfmsg.transform.translation.y = odom.pose.pose.position.y;
    tfmsg.transform.translation.z = odom.pose.pose.position.z;
    tfmsg.transform.rotation = odom.pose.pose.orientation;
    tfbr_.sendTransform(tfmsg);

    // ################################################################
    // ###########     Publish position for gazebo     ################
    // ################################################################
    // gazebo_odom = ModelState()
    //
    // ###### TODO: WARNING! Gazebo uses Z up configuraton!!!  ###
    // # I've created a custom rotation that rotates the position 180 degress
    // # in roll, but the orientation transformation is only yaw = -yaw.
    // # CHECK WHAT HAPPENS WHITH ROLL AND PITCH!
    // """rot = tf.transformations.euler_matrix(math.pi, 0.0, 0.0)
    // position = np.matrix([odom.pose.pose.position.x,
    //                   odom.pose.pose.position.y,
    //                   odom.pose.pose.position.z,
    //                   1.0]).reshape(4, 1)
    //
    // new_position = rot * position
    // eulr =
    // tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
    //                                                  odom.pose.pose.orientation.y,
    //                                                  odom.pose.pose.orientation.z,
    //                                                  odom.pose.pose.orientation.w])
    // new_quat = tf.transformations.quaternion_from_euler(eulr[0], eulr[1],
    // -eulr[2])
    //
    // gazebo_odom.model_name = 'girona500'
    // gazebo_odom.pose.position.x = float(new_position[0])
    // gazebo_odom.pose.position.y = float(new_position[1])
    // gazebo_odom.pose.position.z = float(new_position[2]) +
    // self.sea_bottom_depth
    // gazebo_odom.pose.orientation = odom.pose.pose.orientation
    // gazebo_odom.pose.orientation.x = new_quat[0]
    // gazebo_odom.pose.orientation.y = new_quat[1]
    // gazebo_odom.pose.orientation.z = new_quat[2]
    // gazebo_odom.pose.orientation.w = new_quat[3]
    // gazebo_odom.reference_frame = 'world'
    // self.pub_odom_gazebo.publish(gazebo_odom)
    //
    // # Brodcast world to girona50000_gazebo_link
    // br = tf.TransformBroadcaster()
    // br.sendTransform((gazebo_odom.pose.position.x,
    //                   gazebo_odom.pose.position.y,
    //                   gazebo_odom.pose.position.z),
    //                  new_quat,
    //                  odom.header.stamp,
    //                  "girona500_gazebo_link",
    //                  self.world_frame_id) """
    //
    // ##################################################################
    // #       ALTERNATIVE WITH NO ROTATIONS                       #####
    // gazebo_odom.model_name = 'girona500'
    // gazebo_odom.pose = odom.pose.pose
    // gazebo_odom.reference_frame = 'world'
    // self.pub_odom_gazebo.publish(gazebo_odom)
    //
    // ##################################################################
  }

  /*!
     \brief Get config from param server.
  */
  void getConfig()
  {
    std::cout << "getConfig()" << std::endl;
    // Get name
    std::string vehicle_name;
    nh_.getParam("vehicle_name", vehicle_name);
    // Get other params
    char temp[200];
    // Force
    std::snprintf(temp, sizeof(temp), "dynamics/%s/force_topic", vehicle_name.c_str());
    nh_.getParam(temp, config_.force_topic_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/use_force_topic", vehicle_name.c_str());
    nh_.getParam(temp, config_.use_force_topic_);
    // Thrusters
    std::snprintf(temp, sizeof(temp), "dynamics/%s/thrusters_topic", vehicle_name.c_str());
    nh_.getParam(temp, config_.thrusters_topic_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/number_of_thrusters", vehicle_name.c_str());
    nh_.getParam(temp, config_.thrusters_num_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/thrusters_matrix", vehicle_name.c_str());
    getParamMatrixXd(nh_, temp, config_.thrusters_matrix_, 6);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/max_thrusters_rpm", vehicle_name.c_str());
    nh_.getParam(temp, config_.max_thrusters_rpm_);
    // Thrusters coeff
    std::snprintf(temp, sizeof(temp), "dynamics/%s/ctf", vehicle_name.c_str());
    nh_.getParam(temp, config_.ctf_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/ctb", vehicle_name.c_str());
    nh_.getParam(temp, config_.ctb_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/dzv", vehicle_name.c_str());
    nh_.getParam(temp, config_.dzv_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/dv", vehicle_name.c_str());
    nh_.getParam(temp, config_.dv_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/dh", vehicle_name.c_str());
    nh_.getParam(temp, config_.dh_);
    // Fins
    std::snprintf(temp, sizeof(temp), "dynamics/%s/fins_topic", vehicle_name.c_str());
    nh_.getParam(temp, config_.fins_topic_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/number_of_fins", vehicle_name.c_str());
    nh_.getParam(temp, config_.fins_num_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/a_fins", vehicle_name.c_str());
    nh_.getParam(temp, config_.a_fins_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/k_cd_fins", vehicle_name.c_str());
    nh_.getParam(temp, config_.k_cd_fins_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/k_cl_fins", vehicle_name.c_str());
    nh_.getParam(temp, config_.k_cl_fins_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/max_fins_angle", vehicle_name.c_str());
    nh_.getParam(temp, config_.max_fins_angle_);
    // Other
    std::snprintf(temp, sizeof(temp), "dynamics/%s/period", vehicle_name.c_str());
    nh_.getParam(temp, config_.period_);
    config_.rate_ = 1.0 / config_.period_;
    std::snprintf(temp, sizeof(temp), "dynamics/%s/frame_id", vehicle_name.c_str());
    nh_.getParam(temp, config_.frame_id_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/world_frame_id", vehicle_name.c_str());
    nh_.getParam(temp, config_.world_frame_id_);
    // Collsions
    std::snprintf(temp, sizeof(temp), "dynamics/%s/uwsim_contact_sensor", vehicle_name.c_str());
    nh_.getParam(temp, config_.collisions_topic_);
    if (config_.collisions_topic_.size() > 0)
    {
      config_.contact_sensor_available_ = true;
      // config_.thrusters_matrix = np.array(self.thrusters_matrix).reshape(6, self.thrusters);
    }
    // Body
    std::snprintf(temp, sizeof(temp), "dynamics/%s/odom_topic_name", vehicle_name.c_str());
    nh_.getParam(temp, config_.odom_topic_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/mass", vehicle_name.c_str());
    nh_.getParam(temp, config_.mass_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/buoyancy", vehicle_name.c_str());
    nh_.getParam(temp, config_.buoyancy_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/g", vehicle_name.c_str());
    nh_.getParam(temp, config_.g_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/radius", vehicle_name.c_str());
    nh_.getParam(temp, config_.radius_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/density", vehicle_name.c_str());
    nh_.getParam(temp, config_.water_density_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/tensor", vehicle_name.c_str());
    getParamMatrix3d(nh_, temp, config_.tensor_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/gravity_center", vehicle_name.c_str());
    getParamVector3d(nh_, temp, config_.gravity_center_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/damping", vehicle_name.c_str());
    getParamVector6d(nh_, temp, config_.damping_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/quadratic_damping", vehicle_name.c_str());
    getParamVector6d(nh_, temp, config_.quadratic_damping_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/initial_pose", vehicle_name.c_str());
    getParamVector6d(nh_, temp, config_.p0_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/initial_velocity", vehicle_name.c_str());
    getParamVector6d(nh_, temp, config_.v0_);
    std::snprintf(temp, sizeof(temp), "dynamics/%s/sea_bottom_depth", vehicle_name.c_str());
    nh_.getParam(temp, config_.sea_bottom_depth_);
    // Currents
    std::snprintf(temp, sizeof(temp), "dynamics/%s/current_mean", vehicle_name.c_str());
    if (nh_.hasParam(temp))
    {
      getParamVector3d(nh_, temp, config_.current_mean_);
      std::cout << "getConfig curr sigma" << std::endl;
      std::snprintf(temp, sizeof(temp), "dynamics/%s/current_sigma", vehicle_name.c_str());
      getParamVector3d(nh_, temp, config_.current_sigma_);
      std::cout << "getConfig curr min" << std::endl;
      std::snprintf(temp, sizeof(temp), "dynamics/%s/current_min", vehicle_name.c_str());
      getParamVector3d(nh_, temp, config_.current_min_);
      std::cout << "getConfig curr max" << std::endl;
      std::snprintf(temp, sizeof(temp), "dynamics/%s/current_max", vehicle_name.c_str());
      getParamVector3d(nh_, temp, config_.current_max_);
      std::snprintf(temp, sizeof(temp), "dynamics/%s/current_enabled", vehicle_name.c_str());
      nh_.getParam(temp, config_.current_enabled_);
    }
    // Show params
    showConfig();
  }

  void showConfig()
  {
    // Force
    std::cout << "=====" << std::endl;
    std::cout << "Force" << std::endl;
    std::cout << "=====" << std::endl;
    std::cout << "force_topic = " << config_.force_topic_ << std::endl;
    std::cout << "use_force_topic = " << config_.use_force_topic_ << std::endl;
    // Thrusters
    std::cout << "=========" << std::endl;
    std::cout << "Thrusters" << std::endl;
    std::cout << "=========" << std::endl;
    std::cout << "thrusters_topic = " << config_.thrusters_topic_ << std::endl;
    std::cout << "thrusters_num_ = " << config_.thrusters_num_ << std::endl;
    std::cout << "thrusters_matrix = " << std::endl << config_.thrusters_matrix_ << std::endl;
    std::cout << "max_thrusters_rpm = " << config_.max_thrusters_rpm_ << std::endl;
    // Forward and backward thrusters coeff
    std::cout << "===============" << std::endl;
    std::cout << "Thrusters coeff" << std::endl;
    std::cout << "===============" << std::endl;
    std::cout << "ctf = " << config_.ctf_ << std::endl;
    std::cout << "ctb = " << config_.ctb_ << std::endl;
    std::cout << "dzv = " << config_.dzv_ << std::endl;
    std::cout << "dv = " << config_.dv_ << std::endl;
    std::cout << "dh = " << config_.dh_ << std::endl;
    // Fins
    std::cout << "====" << std::endl;
    std::cout << "Fins" << std::endl;
    std::cout << "====" << std::endl;
    std::cout << "fins_topic = " << config_.fins_topic_ << std::endl;
    std::cout << "fins_num = " << config_.fins_num_ << std::endl;
    std::cout << "a_fins = " << config_.a_fins_ << std::endl;
    std::cout << "k_cd_fins = " << config_.k_cd_fins_ << std::endl;
    std::cout << "k_cl_fins = " << config_.k_cl_fins_ << std::endl;
    std::cout << "max_fins_angle = " << config_.max_fins_angle_ << std::endl;
    // Other
    std::cout << "=====" << std::endl;
    std::cout << "Other" << std::endl;
    std::cout << "=====" << std::endl;
    std::cout << "period = " << config_.period_ << std::endl;
    std::cout << "rate = " << config_.rate_ << std::endl;
    std::cout << "frame_id = " << config_.frame_id_ << std::endl;
    std::cout << "world_frame_id = " << config_.world_frame_id_ << std::endl;
    // Contact sensor
    std::cout << "==========" << std::endl;
    std::cout << "Collisions" << std::endl;
    std::cout << "==========" << std::endl;
    std::cout << "collisions_topic = " << config_.collisions_topic_ << std::endl;
    std::cout << "contact_sensor_available = " << config_.contact_sensor_available_ << std::endl;
    // Body
    std::cout << "====" << std::endl;
    std::cout << "Body" << std::endl;
    std::cout << "====" << std::endl;
    std::cout << "odom_topic = " << config_.odom_topic_ << std::endl;
    std::cout << "mass = " << config_.mass_ << std::endl;
    std::cout << "buoyancy = " << config_.buoyancy_ << std::endl;
    std::cout << "g = " << config_.g_ << std::endl;
    std::cout << "radius = " << config_.radius_ << std::endl;
    std::cout << "water_density = " << config_.water_density_ << std::endl;
    std::cout << "tensor = " << std::endl << config_.tensor_ << std::endl;
    std::cout << "gravity_center = " << std::endl << config_.gravity_center_ << std::endl;
    std::cout << "damping = " << std::endl << config_.damping_ << std::endl;
    std::cout << "quadratic_damping = " << std::endl << config_.quadratic_damping_ << std::endl;
    std::cout << "p0 = " << std::endl << config_.p0_ << std::endl;
    std::cout << "v0 = " << std::endl << config_.v0_ << std::endl;
    std::cout << "sea_bottom_depth = " << config_.sea_bottom_depth_ << std::endl;
    // Currents
    std::cout << "========" << std::endl;
    std::cout << "Currents" << std::endl;
    std::cout << "========" << std::endl;
    std::cout << "current_mean = " << std::endl << config_.current_mean_ << std::endl;
    std::cout << "current_sigma = " << std::endl << config_.current_sigma_ << std::endl;
    std::cout << "current_max = " << std::endl << config_.current_max_ << std::endl;
    std::cout << "current_min = " << std::endl << config_.current_min_ << std::endl;
    std::cout << "current_enabled = " << config_.current_enabled_ << std::endl;
    std::cout << "========" << std::endl;
  }
};  // class Dynamics

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dynamics_new");
  Dynamics node;
  ros::Rate rate(node.getRate());
  while (ros::ok())
  {
    node.iterate();
    rate.sleep();
  }
}
