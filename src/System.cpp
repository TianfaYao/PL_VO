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
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    mpViewer = new Viewer(strSettingsFile, mpMapDrawer);

    mpLocalMapping->SetTracking(mpTracking);
    mpTracking->SetMap(mpMap);
    mpTracking->SetLocalMapping(mpLocalMapping);
    mpTracking->SetViewer(mpViewer);
    mpTracking->SetMapDrawer(mpMapDrawer);

    if (true)
        mptViewer = new thread(&Viewer::Run, mpViewer);


//    mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapping);
}

System::~System()
{
    delete mpCamera;
    delete mpTracking;
    delete mpMap;
    delete mpLocalMapping;
}

Eigen::Matrix<double, 7, 1>  System::TrackRGBD(const cv::Mat &imagergb, const cv::Mat &imagedepth, const double &timeStamps)
{
    mpTracking->Track(imagergb, imagedepth, timeStamps);
    mpLocalMapping->Run();
}

void System::SaveTrajectory(const string &filename)
{
    cout << "save the camera trajectory to " << filename << endl;
    ofstream ofstreamer;

    ofstreamer.open(filename.c_str());

    ofstreamer << fixed;

    for (auto pframe: mpMap->mvpFrames)
    {
//        if (pframe->isKeyFrame())
            ofstreamer << setprecision(6) << pframe->mtimeStamp << " " << pframe->Tcw.inverse().translation().transpose() << endl;
    }

    ofstreamer.close();
}

} // namespace PL_VO