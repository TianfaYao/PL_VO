//
// Created by rain on 17-12-29.
//

#include "System.h"

namespace PL_VO
{

System::System(const string &strSettingsFile)
{
    mpCamera = new Camera(strSettingsFile);
    mpTracking = new Tracking(mpCamera);
    mpMap = new Map;
    mpLocalMapping = new LocalMapping(mpMap);

    mpLocalMapping->SetTracking(mpTracking);
    mpTracking->SetMap(mpMap);
    mpTracking->SetLocalMapping(mpLocalMapping);

    mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapping);
}

System::~System()
{
    delete mpCamera;
}

Eigen::Matrix<double, 7, 1>  System::TrackRGBD(const cv::Mat &imagergb, const cv::Mat &imagedepth, const double &timeStamps)
{
    mpTracking->Track(imagergb, imagedepth, timeStamps);
}

void System::SaveTrajectory(const string &filename)
{
    cout << "save the camera trajectory to " << filename << endl;
    ofstream ofstreamer;

    ofstreamer.open(filename.c_str());

    ofstreamer << fixed;

    for (auto frame: mpMap->mvpFrames)
    {
        ofstreamer << setprecision(6) << frame->mtimeStamp << " " << frame->Tcw.translation().transpose() << endl;
    }

    ofstreamer.close();
}

} // namespace PL_VO