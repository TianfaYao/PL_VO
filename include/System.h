//
// Created by rain on 17-12-29.
//

#ifndef PL_VO_SYSTEM_H
#define PL_VO_SYSTEM_H

#include <iostream>
#include <vector>
#include "Camera.h"
#include "Tracking.h"
#include "LocalMapping.h"
#include "MapDrawer.h"
#include "Viewer.h"

using namespace std;

namespace PL_VO
{

class Camera;
class Tracking;
class Map;
class LocalMapping;
class MapDrawer;
class Viewer;

class System
{

public:

    System(const string &strSettingsFile);

    ~System();

    Eigen::Matrix<double, 7, 1> TrackRGBD(const cv::Mat &imagergb, const cv::Mat &imagedepth, const double &timeStamps);

    void SaveTrajectory(const string &filename);

    void Shutdown();

private:
    Camera *mpCamera;
    Tracking *mpTracking;
    LocalMapping *mpLocalMapping;
    Map *mpMap;
    MapDrawer *mpMapDrawer;
    Viewer *mpViewer;

    thread* mptLocalMapping;
    thread* mptViewer;

};// class System

} // namespace PL_VO
#endif //PL_VO_SYSTEM_H
