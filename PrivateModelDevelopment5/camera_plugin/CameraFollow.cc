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
#include "CameraFollow.hh"
#include <gazebo/transport/transport.hh>
#include <unistd.h>
#include <gazebo/msgs/msgs.hh>


#include "ros/ros.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include <thread>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "tf/transform_listener.h"


#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif


namespace gazebo
{

void CameraFollow::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
    gzmsg << "Load Camera Controller" << std::endl;

    Model = _parent;
    std::string modelName = Model->GetName();
    gzmsg << modelName << std::endl;


//  ################## ROS Node #############################
    if (!ros::isInitialized()) {
         int argc = 0;
         char **argv = NULL;
         ros::init(argc, argv, "CF",
         ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("CF"));

    ros::SubscribeOptions soMaze = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/LC/pose",1,boost::bind(&CameraFollow::OnMazeUpdate, this, _1),ros::VoidPtr(), &this->rosQueueMaze);
    this->rosSubMaze = this->rosNode->subscribe(soMaze);
    this->rosQueueThreadMaze =  thread(std::bind(&CameraFollow::QueueThreadMaze, this));

    ros::SubscribeOptions soBall = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/BC/pose",1,boost::bind(&CameraFollow::OnBallUpdate, this, _1),ros::VoidPtr(), &this->rosQueueBall);
    this->rosSubBall = this->rosNode->subscribe(soBall);
    this->rosQueueThreadBall =  thread(std::bind(&CameraFollow::QueueThreadBall, this));

    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&CameraFollow::OnWorldUpdateBegin, this));
    std::fstream fs;
    fs.open(maze_filename, std::fstream::in);
        std::string line;
        std::getline(fs, line);
        int MAZE_SIZE = line.size();
	float scaleX = 0.05;
	float floorHeight = 0.025; 		// should be equal to z axis scaling of cube for Wall Model; currently "0.05"


	relTransform.setOrigin(tf::Vector3(0, 0, MAZE_SIZE*scaleX-floorHeight-scaleX/10));
	relTransform.setRotation(tf::Quaternion(0,0,0,1));

	refTransform.setOrigin(tf::Vector3(0, 0, 0));
	refTransform.setRotation(tf::Quaternion(0,0,0,1));

	refTransform = refTransform.inverse();
	offsetTransform = refTransform.inverseTimes(relTransform);

	gzmsg 	<< " Offset transform " 	
			<<	offsetTransform.getOrigin().getX() << " " 
							<<	offsetTransform.getOrigin().getY() << " " 
							<<	offsetTransform.getOrigin().getZ() << " " 
							<<	offsetTransform.getRotation().getX() << " " 
							<<	offsetTransform.getRotation().getY() << " " 
							<<	offsetTransform.getRotation().getZ() << " " 
							<<	offsetTransform.getRotation().getW() << std::endl;
}


// Called by the world update start event
void CameraFollow::OnBallUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //gzmsg << "CameraFollow OnBallUpdate" << std::endl;
	_poseBall = ignition::math::Pose3d(
						msg->pose.position.x,
						msg->pose.position.y,
						msg->pose.position.z,
						msg->pose.orientation.w,
						msg->pose.orientation.x,
						msg->pose.orientation.y,
						msg->pose.orientation.z);

}


void CameraFollow::OnMazeUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //gzmsg << "CameraFollow OnMazeUpdate" << std::endl;
	_poseMaze = ignition::math::Pose3d(
						msg->pose.position.x,
						msg->pose.position.y,
						msg->pose.position.z,
						msg->pose.orientation.w,
						msg->pose.orientation.x,
						msg->pose.orientation.y,
						msg->pose.orientation.z);
}

void CameraFollow::QueueThreadMaze() {
    //gzmsg << "CameraFollow QueueThreadMaze" << std::endl;
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueueMaze.callAvailable(ros::WallDuration(timeout));
  }
}

void CameraFollow::QueueThreadBall() {
    //gzmsg << "CameraFollow QueueThreadBall" << std::endl;
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueueBall.callAvailable(ros::WallDuration(timeout));
  }
}

void CameraFollow::OnWorldUpdateBegin(){
    //gzmsg << "CameraFollow OnWorldUpdateBegin" << std::endl;

	/*refTransform.setOrigin(tf::Vector3(
										_poseBall.Pos().X(),
										_poseBall.Pos().Y(),
										_poseBall.Pos().Z()));*/


	refTransform.setOrigin(tf::Vector3(0,0,_poseMaze.Pos().Z()));

	refTransform.setRotation(tf::Quaternion(
										_poseMaze.Rot().X(),
										_poseMaze.Rot().Y(),
										_poseMaze.Rot().Z(),
										_poseMaze.Rot().W()));


	refTransform = refTransform.inverse();
	relTransform = refTransform.inverseTimes(offsetTransform);


	Model->SetWorldPose(math::Pose(
											math::Vector3(
												relTransform.getOrigin().getX(),
												relTransform.getOrigin().getY(),
												relTransform.getOrigin().getZ()),
											math::Quaternion(
												relTransform.getRotation().getW(),
												relTransform.getRotation().getX(),
												relTransform.getRotation().getY(),
												relTransform.getRotation().getZ())));

    Model->SetAngularVel(ignition::math::Vector3d(0,0,0));
    Model->SetLinearVel(ignition::math::Vector3d(0,0,0));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CameraFollow)
}
