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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include <cstdlib>
#include"Converter.h"

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include<tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    void SetPub(ros::Publisher* pub);

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
    ros::Publisher* orb_pub;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }


    if(argc==4)
    {
        std::string sbEqual(argv[3]);
        if(sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO

    // string imgTopic = "/camera/image_raw";
    string imgTopic = "/zed/zed_node/left_raw/image_raw_color";

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img0 = n.subscribe(imgTopic, 100, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("orb_pose", 100);
    igb.SetPub(&pose_pub);

    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    ros::spin();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

//method for assigning publisher
void ImageGrabber::SetPub(ros::Publisher* pub)
{
    orb_pub = pub;
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        cv::Mat im;
        double tIm = 0;
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tIm = img0Buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            {
                this->mBufMutex.lock();
                im = GetImage(img0Buf.front());
                img0Buf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if(mbClahe)
                mClahe->apply(im,im);

            cv::Mat T_ = mpSLAM->TrackMonocular(im,tIm,vImuMeas);

            ros::Time::now();

//      cv::Mat pose = mpSLAM->TrackMonocular(im,tIm,vImuMeas);

            // Publish camera transform
//            static tf::TransformBroadcaster br;
//            static tf::Transform transform;
//            transform.setOrigin(tf::Vector3(pose.at<float>(0,3), pose.at<float>(1,3), pose.at<float>(2,3)));
//            tf::Quaternion q;
//////      q.setRPY(0, 0, msg->theta);
//            transform.setRotation(q);
//            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera"));


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
                /*
                if (pub_tf)
                  {
                    static tf::TransformBroadcaster br_;
                    br_.sendTransform(tf::StampedTransform(transform, ros::Time(tIm), "world", "ORB_SLAM3_MONO_INERTIAL"));
                  }
                */

                geometry_msgs::PoseStamped pose;
                //pose.header.stamp = img0Buf.front()->header.stamp;
                pose.header.frame_id ="ORB_SLAM3_MONO_INERTIAL";
                tf::poseTFToMsg(transform, pose.pose);
                orb_pub->publish(pose);
            }

        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}
