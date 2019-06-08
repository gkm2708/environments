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

using namespace std::chrono;

namespace gazebo
{
	class ContactPlugin : public SensorPlugin
  	{
    	public:
 
			//ContactPlugin();
			//~ContactPlugin();

			ros::NodeHandle nh;
			ros::Publisher pubReward;
			//ros::Publisher pubGC;
			void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    	private: 

			sensors::ContactSensorPtr parentSensor;
			event::ConnectionPtr updateConnection;

			virtual void OnUpdate();
  	};
}
#endif
