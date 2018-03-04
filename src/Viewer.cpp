//
// Created by rain on 18-2-27.
//

#include "Viewer.h"

namespace PL_VO
{

Viewer::Viewer(const string &strSettingPath, MapDrawer* pMapDrawer):mpMapDrawer(pMapDrawer)
{
    cv::FileStorage fsSettings(strSettingPath.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at " << strSettingPath << endl;
        exit(-1);
    }

    mImageHeight = fsSettings["Camera.height"];
    mImageWidth = fsSettings["Camera.width"];

    mViewpointX = fsSettings["Viewer.ViewpointX"];
    mViewpointY = fsSettings["Viewer.ViewpointY"];
    mViewpointZ = fsSettings["Viewer.ViewpointZ"];
    mViewpointF = fsSettings["Viewer.ViewpointF"];
}

void Viewer::UpdateShowImage(Tracking *pTracking)
{
    unique_lock<mutex> lock(mMutex);
//    pTracking->mRawImage.copyTo(mframe);
//    mvFraPointsID = pTracking->mpCurrentFrame->mvFraPointsID;
//    mvFraPointsCnt = pTracking->mpCurrentFrame->mvFraPointsCnt;

    mImageShow = pTracking->GetImageShow();
    if (mImageShow.empty())
    {
        LOG(FATAL) << "the image is empty" << endl;
    }

    if (mImageShow.channels() == 1)
        cv::cvtColor(mImageShow, mImageShow, CV_GRAY2RGB);
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("RAIN_VIO: Map Viewer", 1440, 900); // 1024 768

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.8,1.0,0.0,0.10);

    // the name of the buttion; default setting; whether the selection box;
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowLines("menu.Show Lines",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1440,900,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
    );

    pangolin::Handler3D handler(s_cam);

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.2, 1.0, -1440.0f/900.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& rgbimage = pangolin::Display("image")
            .SetBounds(0.65,1.0,0.65,1.0,(-1.0)*mImageWidth/mImageHeight)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(1)
    {


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        s_cam.Follow(Twc);

        d_cam.Activate(s_cam);

        mpMapDrawer->DrawCurrentCamera(Twc);

        if (menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);

        if (menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        if (menuShowLines)
            mpMapDrawer->DrawMapLines();

        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // the mutex lock is very important, if not, the image sometimes is blurred and program easily shutdown
        if (!mImageShow.empty())
        {
            unique_lock<mutex> lock(mMutex);
            imageTexture.Upload(mImageShow.data,GL_RGB,GL_UNSIGNED_BYTE);
        }

        //display the image
        rgbimage.Activate();
        glColor3f(1.0,1.0,1.0);

        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();

        if (Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if (CheckFinish())
            break;

    } // while(1)

    SetFinish();

} // Viewer::Run()

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if (mbFinishRequested)
    {
        return false;
    }
    else if (mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

} // namespace PL_VO