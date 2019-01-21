#ifndef ROBOT_SIM_UTILS_H
#define ROBOT_SIM_UTILS_H

#include <Eigen/Dense>
#include <armadillo>
#include <ctime>

namespace as64_
{

namespace robot_sim_
{

Eigen::Vector4d rotm2quat(Eigen::Matrix3d rotm);

arma::vec rotm2quat(const arma::mat &rotm);

std::vector<arma::vec> get5thOrder(double t, arma::vec p0, arma::vec pT, double totalTime);

arma::vec posError(const arma::vec &p1, const arma::vec &p2);
arma::vec orientError(const arma::mat &R1, const arma::mat &R2);

void print_err_msg(const std::string &msg);
void print_info_msg(const std::string &msg);
void print_warn_msg(const std::string &msg);

class Timer
{
public:
    Timer()
    {
      clock_gettime(CLOCK_REALTIME, &beg_);
    }

    long int elapsedNanoSec()
    {
        clock_gettime(CLOCK_REALTIME, &end_);
        return (end_.tv_sec - beg_.tv_sec)*1000000000 + (end_.tv_nsec - beg_.tv_nsec);
    }

    double elapsedMicroSec()
    {
        return elapsedNanoSec()/1000.0;
    }

    double elapsedMilliSec()
    {
        return elapsedNanoSec()/1000000.0;
    }

    double elapsedSec()
    {
        return elapsedNanoSec()/1000000000.0;
    }

    void start() { clock_gettime(CLOCK_REALTIME, &beg_); }

private:
    timespec beg_, end_;
};

}; // namespace robot_sim_

}; // namespace as64_

#endif //ROBOT_SIM_UTILS_H
