#ifndef STEWART_PLATFORM_KINEMATICS_H
#define STEWART_PLATFORM_KINEMATICS_H
#include<Eigen/Dense>
#include<vector>
#include<cmath>
class StewartPlatform
{
    public:
        StewartPlatform(double radius_base, double phi_base, double radius_plat, double phi_plat);
        Eigen::MatrixXd getIK(const std::vector<double>& pose);
        Eigen::MatrixXd getJacobian();
        std::vector<double> getFK(const std::vector<double>& starting_pose,
                              const std::vector<double>& lengths_desired,
                              bool plot_flag);
    private:
        Eigen::MatrixXd inverseKinematics(const std::vector<double>& pose);
        Eigen::MatrixXd computeJacobian();
        std::vector<double> forwardKinematics(std::vector<double> pose,
                                            const std::vector<double>& lengths_desired,
                                            bool plot_flag);

        double r_b, phi_b, r_p, phi_p;
        double base_height = 133.8792459 + 22;
        double pivot_distance = 11.0;

        Eigen::MatrixXd b_i;       // Base joint positions
        Eigen::MatrixXd p_i;       // Platform joint positions (rotated)
        Eigen::MatrixXd l_i;       // Leg vectors
        Eigen::MatrixXd l_i_unit;  // Unit leg vectors
        Eigen::MatrixXd jacobian;  // Jacobian matrix
        Eigen::Matrix3d R_mat;     // Current rotation matrix
};

#endif //STEWART_PLATFORM_KINEMATICS_H