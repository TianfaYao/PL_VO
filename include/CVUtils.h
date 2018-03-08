//
// Created by rain on 18-3-6.
//

#ifndef PL_VO_CVUTILS_H
#define PL_VO_CVUTILS_H

#include <iostream>
#include <cmath>

using namespace std;

namespace PL_VO
{

class Sample
{
public:
    static int uniform(int from, int to);
    static double uniform();
    static double gaussian(double sigma);

}; // class Sample

} // namespace PL_VO



#endif //PL_VO_CVUTILS_H
