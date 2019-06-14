/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/PoseStamped.h"







using namespace std;

namespace gazebo
{

class BallControllerPlugin : public ModelPlugin {
    
	private: 
		std::unique_ptr<ros::NodeHandle> rosNode;
		ros::Subscriber rosSub;
		ros::CallbackQueue rosQueue;
		thread rosQueueThread;

	public :
    	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    	void OnWorldUpdateBegin();

    	physics::ModelPtr Model;
		ros::NodeHandle nh;
		ros::Publisher pub;
    	event::ConnectionPtr updateConnectionOn;
};
}
