//
// Created by rain on 17-12-21.
//
#include <iostream>
#include <glog/logging.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <line_descriptor_custom.hpp>

#include "LineFeature.h"
#include "TicToc.h"

using namespace std;

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    const string strDataSets("/home/rain/workspace/DataSets");
    const string strrgbimg("/rgbd_dataset_freiburg2_desk/rgb");
    const string strdepthimg("/rgbd_dataset_freiburg2_desk/depth");

    const string strimg1FilePath = "../test/test_line_match/1.png";
    const string strimg2FilePath = "../test/test_line_match/2.png";
    const string strimg1depthFilePath = "../test/test_line_match/1_depth.png";

    cv::Mat img1 = cv::imread(strimg1FilePath, CV_LOAD_IMAGE_COLOR);
    if (img1.empty())
    {
        cout << "can not load the image " << strimg1FilePath << endl;
        return 1;
    }
    cv::Mat img2 = cv::imread(strimg2FilePath, CV_LOAD_IMAGE_COLOR);
    cv::Mat img1depth = cv::imread(strimg1depthFilePath, CV_LOAD_IMAGE_UNCHANGED);

    PL_VO::LineFeature *pLineFeature;

    double minLinelength = PL_VO::Config::minLineLength() * 480 ;

    PL_VO::TicToc tictoc;
    PL_VO::TicToc tictoc2;

    vector<cv::line_descriptor::KeyLine> vkeylines1;
    cv::Mat linedesc1;
    vector<cv::line_descriptor::KeyLine> vkeylines2;
    cv::Mat linedesc2;

    pLineFeature->detectLinefeature(img1, vkeylines1, linedesc1,  minLinelength);
    pLineFeature->detectLinefeature(img2, vkeylines2, linedesc2,  minLinelength);

    cout << "line feature detection times(ms): " << tictoc2.toc() << endl;

    cv::BFMatcher bfmatcher;
    vector<vector<cv::DMatch>> vlinematches12;

    pLineFeature->matchLineFeatures(&bfmatcher, linedesc1, linedesc2, vlinematches12);

    cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> bdm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

    /* require match */
    vector<cv::DMatch> matches;
    vector<cv::DMatch> distancematches;
    bdm->match(linedesc1, linedesc2, matches );

    cv::Mat homography;
    vector<cv::DMatch> fundamentalmatches;

    PL_VO::TicToc refinetictoc;

    fundamentalmatches = pLineFeature->refineMatchesWithFundamental(vkeylines1, vkeylines2, 0.0, matches, homography);

    cout << "fundamental matrix refine the matches costs time(ms): " << refinetictoc.toc() << endl;

    distancematches = pLineFeature->refineMatchesWithDistance(matches);

    cout << "matches: " << matches.size() << endl;
    cout << "distance matches: " << distancematches.size() << endl;
    cout << "fundamental matches: "<< fundamentalmatches.size() << endl;
    cout << "total times(ms): " << tictoc.toc() << endl;

    cv::Mat showimg, showimg2, showimg3;
    std::vector<char> mask(matches.size(), 1);
    cv::line_descriptor::drawLineMatches(img1, vkeylines1, img2, vkeylines2, matches, showimg, cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
                                         cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);

    cv::line_descriptor::drawLineMatches(img1, vkeylines1, img2, vkeylines2, distancematches, showimg2, cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
                                         cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);

    cv::line_descriptor::drawLineMatches(img1, vkeylines1, img2, vkeylines2, fundamentalmatches, showimg3, cv::Scalar::all(-1), cv::Scalar::all(-1), mask,
                                         cv::line_descriptor::DrawLinesMatchesFlags::DEFAULT);

    cv::imshow(" ", showimg);
    cv::imshow("good match", showimg2);
    cv::imshow("homography match", showimg3);
    cv::waitKey(0);

    return 0;
}

