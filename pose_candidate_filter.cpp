
#include <array>
#include <cmath>
#include <iostream>

// 正方形旋转对称性对应的四个示例候选
const std::array<double, 4> candidates = {
    0.0, 90.0, 180.0, 270.0
};

// 计算两个朝向之间的最小角度差（单位：度）
double angleDiff(double a, double b)
{
    return std::abs(std::remainder(a - b, 360.0));
}

// 用先验筛选候选
void testFilter(double prior, double tolerance)
{
    int valid_count = 0;
    double selected_yaw = 0.0;

    std::cout << "\nPrior = " << prior
              << ", tolerance = " << tolerance << "\n";

    for (double yaw : candidates)
    {
        double error = angleDiff(yaw, prior);

        if (error <= tolerance)
        {
            ++valid_count;
            selected_yaw = yaw;
            std::cout << "Accepted: " << yaw << " deg\n";
        }
    }

    if (valid_count == 1)
    {
        std::cout << "UNIQUE: "
                  << selected_yaw << " deg\n";
    }
    else if (valid_count > 1)
    {
        std::cout << "AMBIGUOUS\n";
    }
    else
    {
        std::cout << "INVALID\n";
    }
}

int main()
{
    testFilter(85.0, 15.0);  // 实验 A
    testFilter(40.0, 50.0);  // 实验 B
    testFilter(45.0, 10.0);  // 实验 C

    return 0;
}
