#include "/home/eng/Desktop/aruco_nano/aruco_nano.h"
#include <opencv2/highgui.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>

using namespace std;

std::string path = "/home/eng/Desktop/aruco_nano/tag_36h11.png";

int main(){
    auto image=cv::imread(path);
    if(image.empty()){
        cout << "Could not open image" << endl;
        return -1;
    }   

    auto markers=aruconano::MarkerDetector::detect(image);
    cout << "Detected " << markers.size() << " markers" << endl;

    for(const auto &m:markers)
       m.draw(image);
     
     cv::imwrite("out.png",image);

    cv::Mat camMatrix,distCoeff;
    camMatrix=cv::Mat::eye(3,3,CV_32F);
    camMatrix.at<float>(0,0)=1000;
    camMatrix.at<float>(1,1)=1000;
    camMatrix.at<float>(0,2)=320;
    camMatrix.at<float>(1,2)=240;
    distCoeff=cv::Mat::zeros(4,1,CV_32F);

    float markerSize=0.05;//5cm
    std::pair<cv::Vec3d, cv::Vec3d> r_t; // rotation and translation vectors
    for(const auto &m:markers)
       r_t=m.estimatePose(camMatrix,distCoeff,markerSize);

    auto Rvec = r_t.first;
    auto Tvec = r_t.second;
    cout << "Rvec: " << Rvec << endl;
    cout << "Tvec: " << Tvec << endl;
    
    cv::aruco::drawAxis(image, camMatrix, distCoeff, Rvec, Tvec, 0.1);
        cv::imshow("out", image);
        char key = (char) cv::waitKey(0);
        if (key == 27)
          

    return 0;
}
