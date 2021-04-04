#include <stdio.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>

int main()
{
    Eigen::Vector4d B(9, 10, -2, 3);
    Eigen::Matrix4d A;
    A << 2, 1, 0, 5,
         1, 2, 1, 2,
         0, 1, 2, 4,
         1, 3, 6, 4.5;
    Eigen::Matrix<double, 4, 1> coef_tmp = A.lu().solve(B);
    std::cout << coef_tmp << std::endl;
    puts("C语言中文网");
    return 0;
}
