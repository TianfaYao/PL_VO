//
// Created by rain on 17-12-29.
//

#ifndef PL_VO_FRAME_H
#define PL_VO_FRAME_H

#include <line_descriptor_custom.hpp>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include "LineFeature.h"
#include "PointFeature.h"
#include "Camera.h"
#include "Map.h"
#include "Converter.h"
#include "TicToc.h"

namespace PL_VO
{

class Camera;
class LineFeature;
class PointFeature;
struct LineFeature2D;
struct PointFeature2D;

class Frame
{

public:
    Frame(const double &timeStamp, Camera *pCamera, LineFeature *pLineFeature, PointFeature *pPointFeature);

    Frame(const Frame &frame);

    size_t GetFrameID();

    const size_t GetFrameID() const;

    Eigen::Vector3d getCameraCenter();

    void detectFeature(const cv::Mat &imagergb, const cv::Mat &imagedepth);

    void UndistortPointFeature();

    void UndistortLineFeature();

    void UndistortKeyFeature();

    void ComputeImageBounds(const cv::Mat &image);

    void matchLPFeature(const cv::Mat &pointdesc1, const cv::Mat &pointdesc2, vector<cv::DMatch> &vpointmatches12,
                        const cv::Mat &linedesc1, const cv::Mat &linedesc2, vector<cv::DMatch> &vlinematches12);

    void refineLPMatches(const vector<cv::KeyPoint> &mvKeyPoint1, const vector<cv::KeyPoint> &mvKeyPoint2,
                         const vector<cv::line_descriptor::KeyLine> &mvKeyLine1,
                         const vector<cv::line_descriptor::KeyLine> &mvKeyLine2,
                         const vector<cv::DMatch> &vpointMatches12, vector<cv::DMatch> &vpointRefineMatches12,
                         const vector<cv::DMatch> &vlineMatches12, vector<cv::DMatch> &vlineRefineMatches12);

    double FindDepth(const cv::KeyPoint &kp, const cv::Mat &imagedepth);

    void AddMapPoint(const cv::Mat &imageDepth, const vector<cv::DMatch> vpointMatches, const vector<cv::DMatch> vlineMatches);

    Camera *mpCamera;
    LineFeature *mpLineFeature;
    PointFeature *mpPointFeature;

    double mtimeStamp;

    Sophus::SE3 Tcw;
    Sophus::SE3 Twc;

    cv::Mat mpointDesc;
    cv::Mat mlineDesc;
    vector<cv::KeyPoint> mvKeyPoint;
    vector<cv::line_descriptor::KeyLine> mvKeyLine;
    vector<cv::KeyPoint> mvKeyPointUn;
    vector<cv::line_descriptor::KeyLine> mvKeyLineUn;
    vector<LineFeature2D*> mvLineFeature2D;
    vector<PointFeature2D*> mvPointFeature2D;

private:

    static size_t gCount;
    size_t mID;
    int mImageHeight;
    int mImageWidth;


}; // class Frame

} // namespace PL_VO
#endif //PL_VO_FRAME_H
