
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

struct Pose2D
{
    double x;
    double y;
    double yaw; // 单位：度
};

// 将角度转换到 [0, 360) 范围
double wrap360(double degrees)
{
    double result = std::fmod(degrees, 360.0);
    return result < 0.0 ? result + 360.0 : result;
}

// 计算两个角度之间的最小差值
double angleDiff(double a, double b)
{
    return std::abs(std::remainder(a - b, 360.0));
}

// 根据地图参考物位姿和 LiDAR 观测，生成四组候选
std::array<Pose2D, 4> generateCandidates(
    const Pose2D &map_box,
    const Pose2D &laser_box)
{
    constexpr double pi = 3.14159265358979323846;
    std::array<Pose2D, 4> result{};

    for (int k = 0; k < 4; ++k)
    {
        // 1. 考虑正方形的四种等价朝向
        double box_yaw_in_laser = laser_box.yaw + 90.0 * k;

        // 2. 反推 LiDAR 在地图中的朝向
        double laser_yaw_in_map =
            wrap360(map_box.yaw - box_yaw_in_laser);

        double angle = laser_yaw_in_map * pi / 180.0;

        // 3. 根据朝向反推 LiDAR 地图位置
        result[k].x = map_box.x -
                      (std::cos(angle) * laser_box.x -
                       std::sin(angle) * laser_box.y);

        result[k].y = map_box.y -
                      (std::sin(angle) * laser_box.x +
                       std::cos(angle) * laser_box.y);

        result[k].yaw = laser_yaw_in_map;
    }

    return result;
}

// 根据朝向先验筛选候选
void checkPrior(
    const std::array<Pose2D, 4> &candidates,
    double prior,
    double tolerance)
{
    int count = 0;
    int selected = -1;

    for (int k = 0; k < 4; ++k)
    {
        if (angleDiff(candidates[k].yaw, prior) <= tolerance)
        {
            ++count;
            selected = k;
        }
    }

    std::cout << "prior=" << prior
              << ", tolerance=" << tolerance << ": ";

    if (count == 1)
    {
        const auto &p = candidates[selected];

        std::cout << "UNIQUE k=" << selected
                  << " pose=(" << p.x << ", "
                  << p.y << ", " << p.yaw << " deg)\n";
    }
    else if (count > 1)
    {
        std::cout << "AMBIGUOUS (" << count << " candidates)\n";
    }
    else
    {
        std::cout << "INVALID (no candidate passes the prior)\n";
    }
}

int main()
{
    Pose2D map_box{4.0, 3.0, 90.0};
    Pose2D laser_box{
        0.020759136093969725,
        -0.4967337697294948,
        0.382203282227};

    // 阶段一：生成完整位姿候选
    auto candidates = generateCandidates(map_box, laser_box);

    std::cout << std::fixed << std::setprecision(2);

    for (int k = 0; k < 4; ++k)
    {
        const auto &p = candidates[k];

        std::cout << "k=" << k
                  << " pose=(" << p.x << ", "
                  << p.y << ", " << p.yaw << " deg)\n";
    }

    // 阶段二：进行三个控制变量实验
    checkPrior(candidates, 85.0, 15.0);
    checkPrior(candidates, 40.0, 50.0);
    checkPrior(candidates, 45.0, 10.0);

    return 0;
}
