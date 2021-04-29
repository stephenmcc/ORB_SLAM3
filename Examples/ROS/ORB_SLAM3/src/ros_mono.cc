/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <cstdlib>

#include"Converter.h"

//for pubbing
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include<tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void SetPub(ros::Publisher* pub);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher* orb_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    // string imgTopic = "/camera/image_raw";
    string imgTopic = "/zed/zed_node/left_raw/image_raw_color";
    ros::Subscriber sub = nodeHandler.subscribe(imgTopic, 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("orb_pose", 100);
    igb.SetPub(&pose_pub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//method for assigning publisher
void ImageGrabber::SetPub(ros::Publisher* pub)
{
    orb_pub = pub;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat T_ = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (!T_.empty())
    {
        cv::Size s = T_.size();
        if ((s.height >= 3) && (s.width >= 3)) {
            cv::Mat R_, t_ ;

            R_ = T_.rowRange(0,3).colRange(0,3).t();
            t_ = -R_*T_.rowRange(0,3).col(3);
            vector<float> q = ORB_SLAM3::Converter::toQuaternion(R_);
            float scale_factor=1.0;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_.at<float>(0, 0)*scale_factor, t_.at<float>(0, 1)*scale_factor, t_.at<float>(0, 2)*scale_factor));
            tf::Quaternion tf_quaternion(q[0], q[1], q[2], q[3]);
            transform.setRotation(tf_quaternion);
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id ="ORB_SLAM3_MONO_INERTIAL";
            tf::poseTFToMsg(transform, pose.pose);
            orb_pub->publish(pose);
        }
    }
}


