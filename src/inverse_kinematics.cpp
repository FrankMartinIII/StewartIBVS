#define _USE_MATH_DEFINES
#include<cstdlib>
#include<iostream>
#include<cmath>
#include<iomanip>
#include<string>
#include<vector>
#include<Eigen/Dense>

class IK
{
    private:
        const int bottomPlateDegreeVals[6] = {43, 77, 163, 197, 283, 317}; //How many degrees the hole is on the base plate away from the y axis
        const int topPlateDegreeVals[6] = {20, 100, 140, 220, 260, 340}; //How many degrees the hole is on the top plate away from the y axis
        const int bottomPlateHoleRadius = 59; //Radius of the plate going through the center of the holes
        const int topPlateHoleRadius = 41;
        const double attachmentPointOffset = 11;
        const double homeHeight = 133.8792459 + attachmentPointOffset + attachmentPointOffset; //This assumes an extension length of 15 mm for the starting position. The addional 34 is to account for the distance from the pivot to the center of each plate (17mm)
        

        double bottomPlateXPoints[6];
        double bottomPlateYPoints[6];
        double topPlateXPoints[6];
        double topPlateYPoints[6];

        double legLengths[6] = {0, 0, 0, 0, 0, 0};
    public:
        IK()
        {
            calcPoints();
            calcLegLengths(0, 0, 0, 0, 0, 0);
        }


    void calcPoints()
    {
        //Conversion from degrees to radians and then to x y coordinates
        for(int i = 0 ; i < 6 ; i++)
        {
            bottomPlateXPoints[i] = bottomPlateHoleRadius * cos(bottomPlateDegreeVals[i] * M_PI / 180.0);
            bottomPlateYPoints[i] = bottomPlateHoleRadius * sin(bottomPlateDegreeVals[i] * M_PI / 180.0);
    
            topPlateXPoints[i] = topPlateHoleRadius * cos(topPlateDegreeVals[i] * M_PI / 180.0);
            topPlateYPoints[i] = topPlateHoleRadius * sin(topPlateDegreeVals[i] * M_PI / 180.0);
        }
    }

    //Calculate the needed lengths of each leg to reach the desired position and angle
    void calcLegLengths(double x_trans, double y_trans, double z_trans,  double roll, double pitch, double yaw)
    {
        z_trans = z_trans + homeHeight;
        //Convert roll, pitch, and yaw to radians
        roll = roll * M_PI / 180;
        pitch = pitch * M_PI / 180;
        yaw = yaw * M_PI / 180;

        //phi, theta, and psi are Euler angls of Z-X-Z sequence, which are not roll, pitch, and yaw in RPY convention.
        //please read: https://howthingsfly.si.edu/flight-dynamics/roll-pitch-and-yaw.

        //double phi = roll;
        //double theta = pitch;
        //double psi = yaw;

        //Define rotation matrix with roll, pitch, and yaw angles.
        //please read: https://msl.cs.uiuc.edu/planning/node102.html
        //Re-delare rotation matrix with roll, pitch, and yaw angles.

        double rotationMatrix[3][3] = {
            {cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)},
            {sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)},
            {-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll)},
        };

        //Now need to multiply the rotation matrix by the translation values
        /*
        [R][x_trans
            y_trans
            z_trans] which will be a 3x1 matrix
        */

        //Do this for each leg
        for(int motorNum = 0 ; motorNum < 6 ; motorNum++)
        {
                double pa1 = topPlateXPoints[motorNum];
                double pa2 = topPlateYPoints[motorNum];
                //double pa3 = 0; //Z val is zero so third col of rotation matrix is actually irrelevant and a waste of cals
                //Set pa3 to home position leg length
                double pa3 = - attachmentPointOffset;

                double a1 = x_trans + (rotationMatrix[0][0] * pa1 + rotationMatrix[0][1] * pa2 + rotationMatrix[0][2] * pa3);
                double a2 = y_trans + (rotationMatrix[1][0] * pa1 + rotationMatrix[1][1] * pa2 + rotationMatrix[1][2] * pa3);
                double a3 = z_trans + (rotationMatrix[2][0] * pa1 + rotationMatrix[2][1] * pa2 + rotationMatrix[2][2] * pa3);


                //L must be computed in same coordinate system, i.e., top plate coordinate system in your case, so, the bottom plate are at -height_hexapod rather than 0.
                //How to compute height_hexapod? the length of linear motor is length + strake/2.
                //please correct it.
                //L should be a vector - a vector of the x y z coordinates of where the leg is attached to the base
                double l_vector[3] = {a1 - bottomPlateXPoints[motorNum] , a2 - bottomPlateYPoints[motorNum], a3 - attachmentPointOffset}; //Saying bottom plate connections are at positive 11 on the z-axis

                //Now find length of the leg by doing dot product of L with itself and square rooting the value.
                double lengthOfLeg = sqrt(l_vector[0] * l_vector[0] + l_vector[1] * l_vector[1] + l_vector[2] * l_vector[2]);

                legLengths[motorNum] = lengthOfLeg;
        }

    }

    std::vector<double> getLegLengths()
    {
        std::vector<double> v(legLengths, legLengths + sizeof(legLengths) / sizeof(legLengths[0]));
        return v;
    }

    Eigen::Matrix<double, 6, 6> computeJacobian(double xtrans, double ytrans, double ztrans, double roll, double pitch, double yaw)
    {
        //Compute the jacobian from the estimated pose

        ztrans = ztrans + homeHeight;
        //Convert to radians
        roll = roll * M_PI / 180;
        pitch = pitch * M_PI / 180;
        yaw = yaw * M_PI / 180;

        //Rotation matrix using Eigen (ZYX)
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        Eigen::Matrix<double, 6, 6> J;

        for(int i = 0 ; i < 6 ; i++)
        {
            //Top attachment point in platform frame
            Eigen::Vector3d p_i(topPlateXPoints[i], topPlateYPoints[i], -attachmentPointOffset);

            // Transform to world frame
            Eigen::Vector3d a_i = Eigen::Vector3d(xtrans, ytrans, ztrans) + R * p_i;

            // Bottom attachment in world frame
            Eigen::Vector3d b_i(bottomPlateXPoints[i], bottomPlateYPoints[i], attachmentPointOffset);

            // Leg vector
            Eigen::Vector3d l_i = a_i - b_i;
            Eigen::Vector3d l_i_unit = l_i.normalized();

            //Translation component of J
            J.block<1,3>(i,0) = l_i_unit.transpose();

            //Rotation component of J
            Eigen::Vector3d crossProd = (R * p_i).cross(l_i_unit);
            J.block<1,3>(i,3) = crossProd.transpose();
        }

        return J;
    }

    std::vector<double> forwardKinematics(const std::vector<double>& legLengthsFromFeedback, const std::vector<double>& initialPoseEst, int maxIter = 100, double tolerance = 1e-4, double alpha = .5)
    {
        IK ik;
        //Put initial pose into eigen vector
        Eigen::VectorXd poseEst(initialPoseEst.size());
        for(int i = 0 ; i < initialPoseEst.size() ; i++)
        {
            poseEst(i) = initialPoseEst.at(i);
        }

        for(int iter = 0 ; iter < maxIter ; iter++)
        {
            //Calc current leg lengths at pose estimate
            ik.calcLegLengths(poseEst(0), poseEst(1), poseEst(2), poseEst(3), poseEst(4), poseEst(5));
            std::vector<double> currentLegs = ik.getLegLengths();
            Eigen::VectorXd error(6);

            for(int i = 0 ; i < 6 ; i++)
            {
                //Difference between feedback vs estimate leg lens
                error(i) = legLengthsFromFeedback.at(i) - currentLegs.at(i);
            }

            if(error.norm() < tolerance)
            {
                break;
            }

            //Use J to adjust pose 
            Eigen::MatrixXd J = computeJacobian(poseEst(0), poseEst(1), poseEst(2), poseEst(3), poseEst(4), poseEst(5));
            Eigen::MatrixXd JTJ = J.transpose() * J + 1e-6 * Eigen::MatrixXd::Identity(6, 6);
            Eigen::VectorXd delta = JTJ.ldlt().solve(J.transpose() * error);

            poseEst += alpha * delta;
            if(iter == maxIter-1)
            {
                std::cout << "Forward kinematics did not converge" << std::endl;
                std::cout << "Leg lens" << std::endl;
                for(double j: legLengthsFromFeedback)
                {
                    std::cout << j << "  ";
                }
                std::cout << "Initial pose estimate" << std::endl;
                for(double j: initialPoseEst)
                {
                    std::cout << j << "  ";
                }
            }
        }

        return std::vector<double>(poseEst.data(), poseEst.data() + poseEst.size());
    } 
};
