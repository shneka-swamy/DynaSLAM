/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
*
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include <vector>
#include <tuple>
#include<opencv2/core/core.hpp>

#include "Geometry.h"
#include "MaskNet.h"
#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

cv::Mat segment(cv::Mat maskRCNN, cv::Mat kernel) {
    cv::Mat mask = cv::Mat::ones(480, 640, CV_8U);
    cv::Mat maskRCNNdil = maskRCNN.clone();
    cv::dilate(maskRCNN,maskRCNNdil, kernel);
    mask = mask - maskRCNNdil;
    return mask;
}

std::vector<std::tuple<int, int, int, int>> boundingSegmentation(cv::Mat segmented_image) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::tuple<int, int, int, int>> boxes;

    cv::findContours(segmented_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        std::tuple<int, int, int, int> box(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
        boxes.push_back(box);
    }

    return boxes;
}

cv::Mat createMaskFromBoxes(const std::vector<std::tuple<int, int, int, int>>& boxes) {
    cv::Mat mask = cv::Mat::zeros(480, 640, CV_8U);
    // cout << "boxes size: " << boxes.size() << endl;

    for (const auto& box : boxes) {
        int x, y, w, h;
        std::tie(x, y, w, h) = box;
        cv::Rect roi(x, y, w - x, h - y);
        cv::Mat roiMask(mask, roi);
        roiMask.setTo(cv::Scalar(1));
    }

    return mask;
}

std::vector<std::tuple<int, int, int, int>> templateMatching(cv::Mat frame, cv::Mat source, const std::vector<std::tuple<int, int, int, int>> &boxes) {
    std::vector<std::tuple<int, int, int, int>> new_boxes;
    cv::Mat frame_gray, source_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(source, source_gray, cv::COLOR_RGB2GRAY);

    for (const auto& box : boxes) {
        int x, y, w, h;
        std::tie(x, y, w, h) = box;
        cv::Mat templateImage = source_gray(cv::Range(y, h), cv::Range(x, w));
        cv::Mat res;
        cv::matchTemplate(frame_gray, templateImage, res, cv::TM_CCOEFF_NORMED);
        double min_val, max_val;
        cv::Point min_loc, max_loc;
        cv::minMaxLoc(res, &min_val, &max_val, &min_loc, &max_loc);
        x = max_loc.x;
        y = max_loc.y;
        w = templateImage.cols;
        h = templateImage.rows;
        new_boxes.push_back(std::make_tuple(x, y, x+w, y+h));
    }

    return new_boxes;
}

class SegmentDecisionGen {
    private:
        int framesThreshold;
        double boxThreshold;
        int frameCount;
        int previousBoxes;
    public:
    SegmentDecisionGen(int framesThreshold, double boxThreshold) : framesThreshold(framesThreshold), boxThreshold(boxThreshold),
    frameCount(-1), previousBoxes(0) {
        assert(framesThreshold > 0);
        assert(boxThreshold > 0);
        assert(boxThreshold <= 1);
    }

    bool segmentDecision(const std::vector<std::tuple<int, int, int, int>>& lastBoxes) {
        assert(framesThreshold > 0);
        assert(boxThreshold > 0);
        assert(boxThreshold <= 1);
        frameCount = (frameCount + 1) % framesThreshold;

        if (frameCount == 0 || lastBoxes.size() < boxThreshold * previousBoxes) {
            frameCount = 0;
            previousBoxes = lastBoxes.size();
            return true;
        }

        previousBoxes = lastBoxes.size();
        return false;
    };

};

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence (path_to_masks)" << endl;
        return 1;
    }

    int framesThreshold = 10;
    double boxThreshold = 0.9;

    //auto segDecision = SegmentDecisionGen(framesThreshold, boxThreshold);

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    // make sure the file exists
    std::ifstream f(strFile.c_str());
    assert(f.good());
    if (f.is_open()) {
        f.close();
    }
    else {
        std::cout << "File not found: " << strFile << std::endl;
        return 1;
    }
    std::cout << "Loading Images " << strFile << std::endl;
    LoadImages(strFile, vstrImageFilenames, vTimestamps);
    std::cout << "Images Loaded" << std::endl;

    int nImages = vstrImageFilenames.size();

    // Initialize Mask R-CNN
    DynaSLAM::SegmentDynObject* MaskNet;
    
    cout << "Loading Mask R-CNN. This could take a while..." << endl;
    MaskNet = new DynaSLAM::SegmentDynObject();
    cout << "Mask R-CNN loaded!" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Vector for segmentation time statistics
    vector<double> vTimesSeg(nImages);
    std::vector<double> vTimesMono(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;

    // Dilation settings
    int dilation_size = 15;
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                        cv::Point( dilation_size, dilation_size ) );
    bool image_set = false;
    cv::Mat prevImage;
    std::vector<std::tuple<int, int, int, int>> prevBoxes;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Segment out the images
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
        std::vector<std::tuple<int, int, int, int>> boxes;
        
        auto ifstart = std::chrono::steady_clock::now();

        if (true) { //segDecision.segmentDecision(prevBoxes)) {
            cv::Mat maskRCNN = MaskNet->GetSegmentation(im,string(argv[4]),vstrImageFilenames[ni].replace(0,4,"")); //0 background y 1 foreground
            mask = segment(maskRCNN, kernel);
            boxes = boundingSegmentation(mask);
        } else {
            boxes = templateMatching(im, prevImage, prevBoxes);
            mask = createMaskFromBoxes(boxes);
        }
        auto ifend = std::chrono::steady_clock::now();
        auto ifduration = std::chrono::duration_cast<std::chrono::duration<double> >(ifend - ifstart);
        vTimesSeg[ni] = ifduration.count();
        
        auto trackStart = std::chrono::steady_clock::now();
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, mask, tframe);
        auto trackEnd = std::chrono::steady_clock::now();
        auto trackDuration = std::chrono::duration_cast<std::chrono::duration<double> >(trackEnd - trackStart);
        vTimesMono[ni] = trackDuration.count();

        // print time with 5.3f precision
        std::cout << "IfBlock Time " << std::fixed << std::setprecision(5) << ifduration.count() << " , MonoBlock " << std::fixed << std::setprecision(5) << trackDuration.count() << std::endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        prevImage = im.clone();
        prevBoxes = boxes;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    auto ifmean = std::accumulate(vTimesSeg.begin(), vTimesSeg.end(), 0.0) / vTimesSeg.size();
    auto trackmean = std::accumulate(vTimesMono.begin(), vTimesMono.end(), 0.0) / vTimesTrack.size();

    std::cout << "IfBlock Time " << ifmean << " s" << std::endl;
    std::cout << "Track Time " << trackmean << " s" << std::endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
