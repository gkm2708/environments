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
#include "BallControllerPlugin.hh"
#include <gazebo/transport/transport.hh>
#include <unistd.h>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/PoseStamped.h"


#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

using namespace std;

namespace gazebo
{

void BallControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    
	gzmsg << "Load Ball Controller" << std::endl;

    Model = _parent;

    //  ################## ROS Node #############################
    if (!ros::isInitialized()) {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "BC",ros::init_options::NoSigintHandler);
     }

    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&BallControllerPlugin::OnWorldUpdateBegin, this));
	pub = nh.advertise<geometry_msgs::PoseStamped>("/BC/pose", 1);
}


















void BallControllerPlugin::OnWorldUpdateBegin(){
    //gzmsg << "BallControllerPlugin OnWorldUpdateBegin" << std::endl;


	geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();

	pose_stamped.pose.position.x = Model->GetWorldPose().pos.x;
	pose_stamped.pose.position.y = Model->GetWorldPose().pos.y;
	pose_stamped.pose.position.z = Model->GetWorldPose().pos.z;
	pose_stamped.pose.orientation.x = Model->GetWorldPose().rot.x;
	pose_stamped.pose.orientation.y = Model->GetWorldPose().rot.y;
	pose_stamped.pose.orientation.z = Model->GetWorldPose().rot.z;
	pose_stamped.pose.orientation.w = Model->GetWorldPose().rot.w;

	pub.publish(pose_stamped);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallControllerPlugin)
}
