#include "stewartPlatformKinematics.h"

StewartPlatform::StewartPlatform(double radius_base, double phi_base, double radius_plat, double phi_plat)
{
    double r_b = radius_base;
    double phi_b = phi_base;
    double r_p = radius_plat; 
    double phi_p = phi_plat;
}

Eigen::MatrixXd StewartPlatform::inverseKinematics(const std::vector<double>& pose);