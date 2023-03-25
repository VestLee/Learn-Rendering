/*
 * @Date: 2023-01-28 14:58:26
 * @LastEditors: mxx
 * @LastEditTime: 2023-03-25 13:27:18
 * @FilePath: \Learn-Rendering\src\Test.cpp
 */
// #include <iostream>
// #include <stdio.h>
//
// #include "Eigen/Core"
// #include "opencv2/opencv.hpp"
//
// int main(int argc, char** argv)
//{
//     Eigen::Vector3d point(2,1,1);
//
//     Eigen::Matrix3d rotate;
//     //逆时针旋转45° => 正角表示逆时针旋转
//     rotate<<cos(EIGEN_PI/4),-sin(EIGEN_PI/4),0,
//             sin(EIGEN_PI/4),cos(EIGEN_PI/4),0,
//             0,0,1;
//
//     Eigen::Matrix3d translate;
//     //平移（1，2）
//     translate<< 1,0,1,
//                 0,1,2,
//                 0,0,1;
//     //Transform
//     std::cout<<translate*rotate*point<<std::endl;
//
//     return 0;
// }