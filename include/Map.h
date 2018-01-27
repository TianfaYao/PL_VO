//
// Created by rain on 18-1-2.
//

#ifndef PL_VO_MAP_H
#define PL_VO_MAP_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "Frame.h"

namespace PL_VO
{

class Frame;
class MapPoint;
class MapLine;
class KeyFrame;
struct PointFeature2D;
struct LineFeature2D;

struct PointFeature2D
{
    PointFeature2D(const Eigen::Vector2d &pixel, const int &level=0, const double &score=0, const int &idxMatch=-1)
                   : mpixel(pixel), mlevel(level), mscore(score), midxMatch(idxMatch)
    {

    }

    Frame *mpFrame = nullptr;
    MapPoint *mpMapPoint = nullptr;

    int midxMatch = -1;
    Eigen::Vector2d mpixel = Eigen::Vector2d(0, 0);
    Eigen::Vector3d mPoint3dw = Eigen::Vector3d(0, 0, 0);
    double mdepth = -1;
    int mlevel = -1;
    double mangle = 0;
    cv::Mat desc = cv::Mat(1, 32, CV_8UC1);
    bool mbinlier = true;
    double mscore = 0;
    double mSigma2;

}; // struct PointFeature2D

struct LineFeature2D
{
    LineFeature2D(const Eigen::Vector2d &Startpixel, const Eigen::Vector2d &Endpixel, const int &level=0,
                  const double &score=0, const int &idxMatch=-1) : mStartpixel(Startpixel), mEndpixel(Endpixel),
                  mlevel(level), mscore(score), midxMatch(idxMatch)
    {
        Eigen::Vector3d startPointH;
        Eigen::Vector3d endPointH;

        startPointH[0] = Startpixel[0];
        startPointH[1] = Startpixel[1];
        startPointH[2] = 1;
        endPointH[0] = Endpixel[0];
        endPointH[1] = Endpixel[1];
        endPointH[2] = 1;
        mLineCoef = startPointH.cross(endPointH);
        // TODO it need to think over
        mLineCoef = mLineCoef/sqrt(mLineCoef[0]*mLineCoef[0] + mLineCoef[1]*mLineCoef[1]);
    }

    Frame *mpFrame = nullptr;
    MapLine *mpMapLine = nullptr;

    int midxMatch = -1;
    Eigen::Vector2d mStartpixel = Eigen::Vector2d(0, 0);
    Eigen::Vector2d mEndpixel = Eigen::Vector2d(0, 0);
    Eigen::Vector3d mLineCoef = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d mStartPoint3dw = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d mEndPoint3dw = Eigen::Vector3d(0, 0, 0);
    double mStartdepth = -1;
    double mEnddepth = -1;
    int mlevel = -1;
    double mangle = 0;
    cv::Mat desc = cv::Mat(1, 32, CV_8UC1);
    bool mbinlier = true;
    double mscore = 0;
    double mSigma2;

}; // struct LineFeature2D

class MapPoint
{
public:

    MapPoint();

    int GetObservedNum(){ return mmpPointFeature2D.size();}
    int GetObservedInliersNum()
    {
        int cnt = 0;
        for (auto it = mmpPointFeature2D.begin(); it != mmpPointFeature2D.end(); it++)
        {
            if (it->second->mbinlier)
                cnt++;
        }
        return cnt;
    }
    vector<Frame*> GetObservedFrame();

    size_t  mID = -1;
    Eigen::Vector3d mPosew = Eigen::Vector3d(0, 0, 0);
    cv::Mat mdesc = cv::Mat(1, 32, CV_8UC1);
    bool mbbad = false;
    vector<Frame*> mvpFrameinvert;
    map<size_t, PointFeature2D*> mmpPointFeature2D;

private:
    mutex mMutexFeatures;

}; // class MapPoint

class MapLine
{
public:

    MapLine();

    int GetObservedNum(){ return mmpLineFeature2D.size();}
    int GetObservedInliersNum()
    {
        int cnt = 0;
        for (auto it = mmpLineFeature2D.begin(); it != mmpLineFeature2D.end(); it++)
        {
            if (it->second->mbinlier)
                cnt++;
        }
        return cnt;
    }
    vector<Frame*> GetObservedFrame();

    size_t mID = -1;
    Eigen::Vector3d mPoseStartw = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d mPoseEndw  = Eigen::Vector3d(0, 0, 0);
    bool mbbad = false;
    cv::Mat mdesc = cv::Mat(1, 32, CV_8UC1);
    vector<Frame*> mvpFrameinvert;
    map<size_t, LineFeature2D*> mmpLineFeature2D;

private:
    mutex mMutexFeatures;

}; // class MapLine

class Map
{
public:

    Map();

    vector<KeyFrame*> GetAllKeyFrames();
    vector<MapPoint*> GetAllMapPoints();
    vector<MapLine*> GetAllMapLines();

    void AddKeyFrame(KeyFrame *pKeyFrame);
    void AddMapPoint(MapPoint *pMapPoint);
    void AddMapLine(MapLine *pMapLine);

    void EraseMapPoint(MapPoint* pMapPoint);
    void EraseMapLine(MapLine* pMapLine);
    void EraseKeyFrame(KeyFrame* pKeyFrame);

    vector<Frame*> mvpFrames;

protected:

    set<MapLine*> mspMapLine;
    set<MapPoint*> mspMapPoint;
    set<KeyFrame*> mspKeyFrame;

    mutex mMutexMap;

}; // class Map

} // namespace PL_VO

#endif //PL_VO_MAP_H
