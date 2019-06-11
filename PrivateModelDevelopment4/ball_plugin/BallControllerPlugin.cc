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
	//this->rosNode.reset(new ros::NodeHandle("CP"));

	//ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/LP/goalPos",1,boost::bind(&BallControllerPlugin::OnReset, this, _1),ros::VoidPtr(), &this->rosQueue);
	//this->rosSub = this->rosNode->subscribe(so);
	//this->rosQueueThread =  thread(std::bind(&BallControllerPlugin::QueueThread, this));

    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&BallControllerPlugin::OnWorldUpdateBegin, this));
	pub = nh.advertise<geometry_msgs::PoseStamped>("/BC/pose", 1);

  	/*pubReward = nh.advertise<geometry_msgs::Vector3Stamped>("/BC/reward", 1);
	std::fstream fs;

    fs.open(maze_filename, std::fstream::in);

    if (fs.good()) {

        std::string line;
        std::getline(fs, line);
        MAZE_SIZE = line.size();
	}*/
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


  	/*geometry_msgs::Vector3Stamped reward3Dstamped;

 	reward3Dstamped.vector.x = 0;
	reward3Dstamped.vector.y = 0;
	reward3Dstamped.vector.z = 0;

	if(pose_stamped.pose.position.x <= ((MAZE_SIZE-2*goal_i)*scaleX/2) && 
		pose_stamped.pose.position.x >= ((MAZE_SIZE-2*goal_i-2)*scaleX/2) &&
		pose_stamped.pose.position.y <= ((MAZE_SIZE-2*goal_j)*scaleY/2) &&
		pose_stamped.pose.position.y >= ((MAZE_SIZE-2*goal_j-2)*scaleY/2) ){

 	reward3Dstamped.vector.x = 1.0;
	reward3Dstamped.vector.y = 0;
	reward3Dstamped.vector.z = 0;
	//gzmsg << "2" << std::endl;

	}

	reward3Dstamped.header.stamp = ros::Time::now();
	pubReward.publish(reward3Dstamped);*/

}

void BallControllerPlugin::QueueThread()
	{
	    static const double timeout = 0.01;
	    while (this->rosNode->ok())
	    {
	        this->rosQueue.callAvailable(ros::WallDuration(timeout));
	    }
	}

/*void BallControllerPlugin::OnReset(const geometry_msgs::Vector3::ConstPtr& msg){
    goal_i = (int)msg->x; 	
    goal_j = (int)msg->y;  

}*/

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BallControllerPlugin)
}
