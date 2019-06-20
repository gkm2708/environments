#include "CameraPlugin.hh"

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "image_transport/image_transport.h"
#include "geometry_msgs/PoseStamped.h"

#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"


using namespace std;

namespace gazebo

{
class CameraCapture : public CameraPlugin
{
public:
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    void OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format);

	void OnNewPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

    image_transport::Publisher pub;

	//bool toggle;
	//int skip;
	//int skipCounter;

    //void QueueThread() ;
	//int skipCounter;
	//int skipParam;
/*
	ros::Time t_base;


	tf::Transform refTransform;
	tf::Transform rotTransform;
	tf::Transform relTransform;



private: std::unique_ptr<ros::NodeHandle> rosNode;
private: ros::Subscriber rosSub;
private: ros::CallbackQueue rosQueue;
private: thread rosQueueThread;
*/


};
// Register this plugin with the simulator
}
