//
// Created by rain on 17-12-28.
//

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "System.h"

using namespace std;


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void SaveGroundTruthfile(const string &strGroundTruthname,vector<pair<double, Sophus::SE3d>> &vSE3GroundTruth);

void LoadGroundTruth(const string &strGroundTruthname, vector<pair<double, Sophus::SE3d>> &vSE3GT);

int main(int argc, char **argv)
{

    google::InitGoogleLogging(argv[0]);

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;

    string strAssociationFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg2_desk/associate.txt");
    string strGroundTruthFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg2_desk/groundtruth.txt");
    string strSequenceFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg2_desk");
    string strSettingsFile("../Example/TUM2.yaml");

//    string strAssociationFilename("/home/rain/workspace/DataSets/freiburg3_long_office_household/associate.txt");
//    string strGroundTruthFilename("/home/rain/workspace/DataSets/freiburg3_long_office_household/groundtruth.txt");
//    string strSequenceFilename("/home/rain/workspace/DataSets/freiburg3_long_office_household");
//    string strSettingsFile("../Example/TUM3.yaml");

//    string strAssociationFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg3_structure_notexture_far/associations.txt");
//    string strGroundTruthFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg3_structure_notexture_far/groundtruth.txt");
//    string strSequenceFilename("/home/rain/workspace/DataSets/rgbd_dataset_freiburg3_structure_notexture_far");
//    string strSettingsFile("../Example/TUM3.yaml");

    cout << "datasets asscociateion file: " << strAssociationFilename << endl;
    cout << "datasets sequence file: " << strSequenceFilename << endl;
    cout << "datasets ground truth: " << strGroundTruthFilename << endl;
    cout << "setting file: "<< strSettingsFile << endl;

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

//    vector<pair<double, Sophus::SE3d>> vSE3GT;
//    LoadGroundTruth(strGroundTruthFilename, vSE3GT);
//    return 0;

    size_t imagesize = vstrImageFilenamesRGB.size();

    if (imagesize <= 0)
    {
        cerr << endl << "no images found in that path " << endl;
        return 1;
    }

    if (vstrImageFilenamesRGB.size() != vstrImageFilenamesD.size())
    {
        cerr << endl << "the number of the rgb image and depth image are different " << endl;
        return 1;
    }

    PL_VO::System vo(strSettingsFile);

    cv::Mat imRGB, imDepth;
    for (size_t i = 0; i < imagesize; i++)
    {
        imRGB = cv::imread(strSequenceFilename +"/"+ vstrImageFilenamesRGB[i], CV_LOAD_IMAGE_UNCHANGED);
        imDepth = cv::imread(strSequenceFilename +"/"+ vstrImageFilenamesD[i], CV_LOAD_IMAGE_UNCHANGED);
        double imagetimestamps = vTimestamps[i];

        if (imRGB.empty())
        {
            cerr << "failed to load image file " << strSequenceFilename +"/"+ vstrImageFilenamesRGB[i] << endl;
        }
        if (imDepth.empty())
        {
            cerr << "failed to load image file " << strSequenceFilename +"/"+ vstrImageFilenamesD[i] << endl;
        }
        vo.TrackRGBD(imRGB, imDepth, imagetimestamps);
    }

    vo.SaveTrajectory("tum_trajectory1.txt");

    while (1)
        usleep(1000);

    return 0;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());

    if (!fAssociation.is_open())
        cerr << "the file path is wrong! " << endl;

    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

void LoadGroundTruth(const string &strGroundTruthname, vector<pair<double, Sophus::SE3d>> &vSE3GT)
{
    ifstream fGroundTruth;
    fGroundTruth.open(strGroundTruthname.c_str());

    if (!fGroundTruth.is_open())
        cerr << "the groundtruth file path is wrong! " << endl;


    while(!fGroundTruth.eof())
    {
        string s;
        getline(fGroundTruth, s);

        if (s[0] == '#')
            continue;

        if (!s.empty())
        {
            stringstream ss;
            ss << s;

            double t, tx, ty, tz, qx, qy, qz, qw;
            ss >> t;
            ss >> tx;
            ss >> ty;
            ss >> tz;
            ss >> qx;
            ss >> qy;
            ss >> qz;
            ss >> qw;

            Sophus::SE3d SE3_(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
            vSE3GT.emplace_back(make_pair(t, SE3_));

        }
    }

    Sophus::SE3d SE30;

    SE30 = vSE3GT[0].second;
    SE30 = SE30.inverse();

    for (auto &SE3_ : vSE3GT)
    {
        SE3_.second = SE30*SE3_.second;
    }

    SaveGroundTruthfile("groundtruth1.txt", vSE3GT);
}

void SaveGroundTruthfile(const string &strGroundTruthname,vector<pair<double, Sophus::SE3d>> &vSE3GroundTruth)
{
    ofstream fGrountTruth(strGroundTruthname);

    CHECK(fGrountTruth.is_open());

    for (auto SE3_ : vSE3GroundTruth)
    {
        fGrountTruth << setprecision(6)
                     << to_string(SE3_.first) << " "
                     << setprecision(7)
                     << SE3_.second.translation()[0] << " "
                     << SE3_.second.translation()[1] << " "
                     << SE3_.second.translation()[2] << " "
                     << SE3_.second.so3().unit_quaternion().coeffs()[0] << " "
                     << SE3_.second.so3().unit_quaternion().coeffs()[1] << " "
                     << SE3_.second.so3().unit_quaternion().coeffs()[2] << " "
                     << SE3_.second.so3().unit_quaternion().coeffs()[3] << " "
                     << endl;
    }

    fGrountTruth.close();
}