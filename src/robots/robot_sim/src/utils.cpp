#include <robot_sim/utils.h>
#include <cmath>
#include <vector>

namespace as64_
{

namespace robot_sim_
{

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm)
{
    Eigen::Quaternion<double> temp_quat(rotm);
    Eigen::Vector4d quat;
    quat << temp_quat.w(), temp_quat.x(), temp_quat.y(), temp_quat.z();

    quat = quat * (2*(quat(0)>=0)-1); // to avoid discontinuities

    return quat;
}

arma::vec rotm2quat(const arma::mat &rotm)
{
  arma::vec quat(4);

  Eigen::Map<const Eigen::Matrix3d> rotm_wrapper(rotm.memptr());
  Eigen::Map<Eigen::Vector4d> quat_wrapper(quat.memptr());
  quat_wrapper = rotm2quat(rotm_wrapper);

  return quat;
}

arma::vec posError(const arma::vec &p1, const arma::vec &p2)
{
  return p1-p2;
}

arma::vec orientError(const arma::mat &R1, const arma::mat &R2)
{
  return (robot_sim_::rotm2quat(R1).subvec(1,3) - robot_sim_::rotm2quat(R2).subvec(1,3));
}

std::vector<arma::vec> get5thOrder(double t, const arma::vec p0, const arma::vec pT, double totalTime)
{
  std::vector<arma::vec> retTemp(3);

  int N_dim = p0.n_rows;

  if (t < 0)
  {
    // before start
    retTemp[0] = p0;
    retTemp[1] = arma::vec().zeros(N_dim);
    retTemp[2] = arma::vec().zeros(N_dim);
  }
  else if (t > totalTime)
  {
    // after the end
    retTemp[0] = pT;
    retTemp[1] = arma::vec().zeros(N_dim);
    retTemp[2] = arma::vec().zeros(N_dim);
  }
  else
  {
    // somewhere betweeen ...
    // position
    retTemp[0] = p0 +
                     (pT - p0) * (10 * std::pow(t / totalTime, 3) -
                     15 * std::pow(t / totalTime, 4) +
                     6 * std::pow(t / totalTime, 5));
    // vecolity
    retTemp[1] = (pT - p0) * (30 * std::pow(t, 2) / std::pow(totalTime, 3) -
                     60 * std::pow(t, 3) / std::pow(totalTime, 4) +
                     30 * std::pow(t, 4) / std::pow(totalTime, 5));
    // acceleration
    retTemp[2] = (pT - p0) * (60 * t / std::pow(totalTime, 3) -
                     180 * std::pow(t, 2) / std::pow(totalTime, 4) +
                     120 * std::pow(t, 3) / std::pow(totalTime, 5));
  }

  // return vector
  return retTemp;
}

void print_err_msg(const std::string &msg)
{
  std::cerr << "\033[1m\033[31m" << "[RobotSim ERROR]: " << msg << "\033[0m";
}

void print_info_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[34m" << "[RobotSim INFO]: " << msg << "\033[0m";
}

void print_warn_msg(const std::string &msg)
{
  std::cout << "\033[1m\033[33m" << "[RobotSim WARNING]: " << msg << "\033[0m";
}

}; // namespace robot_sim_

}; // namespace as64_