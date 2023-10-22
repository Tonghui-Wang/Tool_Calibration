#include <iostream>
#include <eigen3/Eigen/Dense>
// #include <eigen3/unsupported/Eigen/CXX11/Tensor>

using namespace std;

typedef Eigen::Matrix<float, 6, 1> Vector6d;

int main()
{
    // 1. 输入标定点位姿，每点的坐标按[XYZABC]格式描述，其中ABC为绕ZYX轴旋转的角度值
    Vector6d Cal1, Cal2, Cal3, Cal4, Cal5, Cal6;
    Cal1 << 1028.29, 233.29, 1432.98, -156.2, 44.8, -126.6;
    Cal2 << 1118.69, -231.99, 1478.85, 153.6, 19.0, 136.1;
    Cal3 << 1129.45, 25.0, 1564.0, -175.1, 29.4, -175.0;
    Cal4 << 983.98, 0.39, 1502.31, 179.8, 54.3, 180.0;
    Cal5 << 605.47, 0.39, 1502.31, 179.8, 54.3, 180.0;
    Cal6 << 983.98, 0.39, 1122.24, 179.8, 54.3, 180.0;

    // 2. 计算各标定点的姿态描述Rot和位置描述Pos
    Eigen::Matrix3d Rot1, Rot2, Rot3, Rot4, Rot5, Rot6;// 标定点姿态声明
    Eigen::Vector3d Pos1, Pos2, Pos3, Pos4, Pos5, Pos6;// 标定点位置声明
    Eigen::AngleAxisd Roll, Pitch, Yaw;// RPY向量声明

    Roll = Eigen::AngleAxisd(Cal1(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal1(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal1(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot1 = Yaw * Pitch * Roll;
    Pos1 << Cal1(0), Cal1(1), Cal1(2);

    Roll = Eigen::AngleAxisd(Cal2(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal2(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal2(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot2 = Yaw * Pitch * Roll;
    Pos2 << Cal2(0), Cal2(1), Cal2(2);

    Roll = Eigen::AngleAxisd(Cal3(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal3(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal3(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot3 = Yaw * Pitch * Roll;
    Pos3 << Cal3(0), Cal3(1), Cal3(2);

    Roll = Eigen::AngleAxisd(Cal4(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal4(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal4(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot4 = Yaw * Pitch * Roll;
    Pos4 << Cal4(0), Cal4(1), Cal4(2);

    Roll = Eigen::AngleAxisd(Cal5(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal5(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal5(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot5 = Yaw * Pitch * Roll;
    Pos5 << Cal5(0), Cal5(1), Cal5(2);

    Roll = Eigen::AngleAxisd(Cal6(5) * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Pitch = Eigen::AngleAxisd(Cal6(4) * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Yaw = Eigen::AngleAxisd(Cal6(3) * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Rot6 = Yaw * Pitch * Roll;
    Pos6 << Cal6(0), Cal6(1), Cal6(2);

    // 3. 位置偏移解算
    Eigen::MatrixXd tmp1(9, 3), tmp2(9, 1);
    tmp1.block<3, 3>(0, 0) = Rot2 - Rot1;
    tmp1.block<3, 3>(3, 0) = Rot3 - Rot2;
    tmp1.block<3, 3>(6, 0) = Rot4 - Rot3;
    tmp2.block<3, 1>(0, 0) = Pos1 - Pos2;
    tmp2.block<3, 1>(3, 0) = Pos2 - Pos3;
    tmp2.block<3, 1>(6, 0) = Pos3 - Pos4;

    Eigen::Vector3d Pos = (tmp1.transpose() * tmp1).inverse() * tmp1.transpose() * tmp2;

    // 4. 姿态偏移解算
    Eigen::Vector3d Vn = (Pos5 - Pos4).normalized();
    Eigen::Vector3d Va = (Pos6 - Pos4).normalized();
    Eigen::Vector3d Vo = Va.cross(Vn);
    Va = Vn.cross(Vo);
    
    Eigen::Matrix3d tmp3;
    tmp3.block<3, 1>(0, 0) = Vn;
    tmp3.block<3, 1>(0, 1) = Vo;
    tmp3.block<3, 1>(0, 2) = Va;

    Eigen::Matrix3d Rot = Rot4.inverse() * tmp3;

    // 5. 位姿偏移解算并输出
    Eigen::Matrix4d Res;
    Res.block<3, 3>(0, 0) = Rot;
    Res.block<3, 1>(0, 3) = Pos;
    Res.block<1, 4>(3, 0) << 0, 0, 0, 1;

    cout << Res <<endl;
}
