#include "inverse_kinematics.cpp"
int main(int argc, char** argv)
{
    IK ik;
    std::vector<double> v = ik.getLegLengths();

    for(int i = 0 ; i < v.size() ; i++)
    {
        std::cout << v.at(i) << std::endl;
    }

    ik.calcLegLengths(-2, 5, 0, 3, 5, 6);
    v = ik.getLegLengths();

    for(int i = 0 ; i < v.size() ; i++)
    {
        std::cout << v.at(i) << std::endl;
        //v.at(i) = v[i] + 1;
    }
    std::vector<double> initPoseEst = {-2, 5, 0, 3, 5, 6};
    std::vector<double> guess = ik.forwardKinematics(v, initPoseEst, 500);

    for(int i = 0 ; i < guess.size() ; i++)
    {
        std::cout << guess.at(i) << std::endl;
    }
}