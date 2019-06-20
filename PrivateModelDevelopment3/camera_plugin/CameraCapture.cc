
#include "gazebo/gazebo.hh"

#include "CameraPlugin.hh"
#include "CameraCapture.hh"

#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_listener.h"

namespace gazebo
{

void CameraCapture::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    	gzmsg << "Load CameraCapture" << std::endl;
    CameraPlugin::Load(_parent, _sdf);

    int argc = 0;
    char** argv = NULL;

    ros::init(argc, argv, "SC");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    pub = it.advertise("/lmaze/view",1);

}

void CameraCapture::OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
	//ROS_INFO(" New Frame Sent ");

    cv::Mat image(_height, _width, CV_8UC3);
    std::memcpy(image.data, _image, _width*_height*_depth);

	std_msgs::Header h;
	h.stamp = ros::Time::now();

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(h, "bgr8", image).toImageMsg();
    pub.publish(msg);
}

GZ_REGISTER_SENSOR_PLUGIN(CameraCapture)
}
