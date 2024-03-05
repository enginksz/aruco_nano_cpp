#include "/home/eng/Desktop/aruco_nano/aruco_nano.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>
// Include the ROS library
#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

cv::Mat camMatrix;  
cv::Mat distCoeff;  
ros::Publisher markerPosePub; 
ros::Publisher imagePub; 


int main(int argc, char **argv) {
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;
        
    markerPosePub = nh.advertise<geometry_msgs::PointStamped>("/aruco_pose", 10);
    imagePub = nh.advertise<sensor_msgs::Image>("/aruco_image", 10);


       cv::VideoCapture cap(2,200);
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
            
            camMatrix = (cv::Mat_<double>(3,3) << 342.6247913936494, 0.0, 325.61639417926665, 0.0, 343.57334085674063, 263.2417118412549, 0.0, 0.0, 1.0);
            distCoeff = (cv::Mat_<double>(4,1) << 0.002263354089641115, -0.06543755873692235, 0.07994321345991563, -0.03897666753502699);
    

            float markerSize = 0.12; //5cm
            std::pair<cv::Vec3d, cv::Vec3d> r_t; // rotation and translation vectors

            for (const auto &m : markers)
                r_t = m.estimatePose(camMatrix, distCoeff, markerSize);

                auto Rvec = r_t.first;
                auto Tvec = r_t.second;
                cout << "Rvec: " << Rvec << endl;
                cout << "Tvec: " << Tvec << endl;
                cv::aruco::drawAxis(frame, camMatrix, distCoeff, Rvec, Tvec, 0.1);

                cv::resize(frame, frame, cv::Size(640, 480));
                cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

                
                geometry_msgs::PointStamped pointStampedMsg;
                pointStampedMsg.header.stamp = ros::Time::now();
                pointStampedMsg.header.frame_id = "camera_frame";
                pointStampedMsg.point.x = Tvec[0];
                pointStampedMsg.point.y = Tvec[1];
                pointStampedMsg.point.z = Tvec[2];


                markerPosePub.publish(pointStampedMsg);
                sensor_msgs::ImagePtr latestImageMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                imagePub.publish(latestImageMsg);



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