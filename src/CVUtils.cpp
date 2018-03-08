//
// Created by rain on 18-3-6.
//

#include "CVUtils.h"

namespace PL_VO
{
// https://github.com/izhengfan/ba_demo_ceres
static double uniformrand(double lowerBndr, double upperBndr)
{
    return lowerBndr + ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gaussrand(double mean, double sigma)
{
    double x, y, r2;
    do
    {
        x = -1.0 + 2.0 * uniformrand(0.0, 1.0);
        y = -1.0 + 2.0 * uniformrand(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to)
{
    return static_cast<int>(uniformrand(from, to));
}

double Sample::uniform()
{
    return uniformrand(0., 1.);
}

double Sample::gaussian(double sigma)
{
    return gaussrand(0., sigma);
}

} // namespace PL_VO

