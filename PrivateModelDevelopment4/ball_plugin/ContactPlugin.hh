#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <chrono>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3.h"

using namespace std;
using namespace std::chrono;

namespace gazebo
{
	class ContactPlugin : public SensorPlugin
  	{
    	public:
 			std::unique_ptr<ros::NodeHandle> rosNode;
			ros::Subscriber rosSub;
			ros::CallbackQueue rosQueue;
			thread rosQueueThread;

			//ContactPlugin();
			//~ContactPlugin();
			void OnReset(const geometry_msgs::Vector3::ConstPtr& msg);
			ros::NodeHandle nh;
			ros::Publisher pubReward;
			//ros::Publisher pubGC;
			void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
			int goal_i = -1;
			int goal_j = -1;
    	private: 
		    void QueueThread() ;
	
			sensors::ContactSensorPtr parentSensor;
			event::ConnectionPtr updateConnection;

			virtual void OnUpdate();
  	};
}
#endif
