//
// Created by rain on 17-12-29.
//

#ifndef PL_VO_TRACKING_H
#define PL_VO_TRACKING_H


#include <string>
#include <iostream>
#include <mutex>
#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "Camera.h"
#include "Frame.h"
#include "LineFeature.h"
#include "PointFeature.h"
#include "Converter.h"
#include "Optimizer.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "MapDrawer.h"

using namespace std;

namespace PL_VO
{

class Camera;
class Frame;
class LineFeature;
class PointFeature;
class Map;
class KeyFrame;
class LocalMapping;
class Viewer;
class MapDrawer;

class Tracking
{

public:

    Tracking(Camera *pCamera);

    ~Tracking();

    void SetMap(Map *pMap);

    void SetCurLastFrame(Frame *pcurFrame, Frame *plastFrame);

    void SetLastKeyFrame(KeyFrame *pKeyFrame);

    void Track(const cv::Mat &imagegray, const cv::Mat &imD, const double &timeStamps);

    bool TrackRefFrame(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches);

    bool TrackRefFrameByPnP(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches);

    void UpdateMapLPfeature(const vector<cv::DMatch> &vpointMatches, const vector<cv::DMatch> &vlineMatches);

    void SetLocalMapping(LocalMapping *pLocalMapping);

    bool NeedNewKeyFrame();

    void CreateNewKeyFrame();

    cv::Mat GetImageShow();

    void SetViewer(Viewer *pViewer);

    void SetMapDrawer(MapDrawer *pMapDrawer);


private:

    void UpdateMapPointfeature(const vector<cv::DMatch> &vpointMatches);

    void UpdateMapLinefeature(const vector<cv::DMatch> &vlineMatches);

    Camera *mpcamera = nullptr;
    Frame *mpcurrentFrame = nullptr;
    Frame *mplastFrame = nullptr;
    KeyFrame *mpLastKeyFrame = nullptr;
    LineFeature *mpLineFeature = nullptr;
    PointFeature *mpPointFeature = nullptr;
    LocalMapping *mpLocalMapping = nullptr;
    Map *mpMap = nullptr;
    Viewer *mpViewer = nullptr;
    MapDrawer *mpMapDrawer = nullptr;

    Sophus::SE3d mPoseInc;

    cv::Mat mimageGray;
    cv::Mat mimagergb;
    cv::Mat mlastimageGrays;
    cv::Mat mlastimagergb;
    cv::Mat mimageDepth;
    cv::Mat mlastimageDepth;

    size_t countMapPoint;
    size_t countMapLine;

    mutex mMutex;

}; // class Tracking

} // namespce PL_VO


#endif //PL_VO_TRACKING_H
