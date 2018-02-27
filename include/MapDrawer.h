//
// Created by rain on 18-2-27.
//

#ifndef PL_VO_MAPDRAWER_H
#define PL_VO_MAPDRAWER_H


#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include "Map.h"
#include "KeyFrame.h"

namespace PL_VO
{

class Map;
class KeyFrame;

class MapDrawer
{

public:

    MapDrawer(Map* pMap, const string &strSettingPath);

    pangolin::OpenGlMatrix toOpenGLMatrix(const Eigen::Matrix<double, 3, 4> &Twc);

    void DrawMapPoints();

    void DrawKeyFrames(const bool &bDrawKF, const bool &bDrawGraph);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(const Eigen::Matrix<double, 3, 4> &Twc);

//    void SetReferenceKeyFrame(KeyFrame *pKF);

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &Mcw);

    Map *mpMap = nullptr;

private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Eigen::Matrix<double, 3, 4> mCameraPose;

    mutex mMutexCamera;

}; // class MapDrawer

} // namespace PL_VO


#endif //PL_VO_MAPDRAWER_H
