#include "/home/eng/Desktop/aruco_nano/aruco_nano.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>


using namespace std;

int main(int argc, const char ** argv)
{
       cv::VideoCapture cap(0,200);
        int fpsCamera = 30;
int fpsCapture = 10;
        double fps2 = cap.get(cv::CAP_PROP_FPS);
        std::cout << "fps2 : " << fps2 << std::endl;

        if (!cap.isOpened())
        {
            cap.release();
            return -1;
        }

        std::chrono::time_point<std::chrono::high_resolution_clock>
                   prev_frame_time(std::chrono::high_resolution_clock::now());
        std::chrono::time_point<std::chrono::high_resolution_clock>
                           new_frame_time;

        cv::Mat frame;
        while (true)
        {
           cap >> frame;
           if (frame.empty()) {
                cout << "Could not capture frame" << endl;
                break;
        }

            new_frame_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration1(new_frame_time - prev_frame_time);
            double fps = 1/duration1.count();
            std::cout << "fps : " << fps << std::endl;

            auto markers = aruconano::MarkerDetector::detect(frame);
            cout << "Detected " << markers.size() << " markers" << endl;

            for (const auto &m : markers)
                m.draw(frame);
            
            cv::Mat camMatrix, distCoeff;
            camMatrix = cv::Mat::eye(3, 3, CV_32F);
            camMatrix.at<float>(0, 0) = 1000;
            camMatrix.at<float>(1, 1) = 1000;
            camMatrix.at<float>(0, 2) = 320;
            camMatrix.at<float>(1, 2) = 240;
            distCoeff = cv::Mat::zeros(4, 1, CV_32F);

            float markerSize = 0.05; // 5cm
            std::pair<cv::Vec3d, cv::Vec3d> r_t; // rotation and translation vectors

            for (const auto &m : markers)
                r_t = m.estimatePose(camMatrix, distCoeff, markerSize);

            auto Rvec = r_t.first;
            auto Tvec = r_t.second;
            cout << "Rvec: " << Rvec << endl;
            cout << "Tvec: " << Tvec << endl;
            cv::aruco::drawAxis(frame, camMatrix, distCoeff, Rvec, Tvec, 0.1);

            cv::resize(frame, frame, cv::Size(640, 480));

            if(duration1.count() > 1/fpsCapture)
              {
                        	prev_frame_time = new_frame_time;
                  
            	imshow("Frame", frame);
            }

            int key = cv::waitKey(1000/fpsCamera );
            if (key % 256 == 1)
            {
            	break; // (27)escape key
            }
        }

        cap.release();
 return 0;
}