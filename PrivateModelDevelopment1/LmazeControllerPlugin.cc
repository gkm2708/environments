/* Open-Source License with no liability whatsoever */

#include "LmazeControllerPlugin.hh"

#include <ignition/math/Pose3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include <thread>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"


#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace gazebo
{


void LmazeControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
	gzmsg << "Load LMaze Controller main" << std::endl;

    int MAZE_SIZE = 11;
    Model = _parent;

#if GAZEBO_MAJOR_VERSION >= 8
    std::string modelName = Model->GetName();
    math::Vector3d InitialPos = Model->WorldPose().Pos();
#else
    std::string modelName = Model->GetName();
    math::Vector3 InitialPos = Model->GetWorldPose().pos;
#endif

	gzmsg << modelName << std::endl;

    //  ################## ROS Node #############################
    if (!ros::isInitialized()) {
		int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LC",
        ros::init_options::NoSigintHandler);
	}

    this->rosNode.reset(new ros::NodeHandle("LC"));

    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/interface/action",1,boost::bind(&LmazeControllerPlugin::OnVelUpdate, this, _1),ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    this->rosQueueThread =  thread(std::bind(&LmazeControllerPlugin::QueueThread, this));

	ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Bool>("/LP/reset",1,boost::bind(&LmazeControllerPlugin::OnReset, this, _1),ros::VoidPtr(), &this->rosQueue1);
 	this->rosSub1 = this->rosNode->subscribe(so1);
	this->rosQueueThread1 =  thread(std::bind(&LmazeControllerPlugin::QueueThread1, this));


    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&LmazeControllerPlugin::OnWorldUpdateBegin, this));

	pub = rosNode->advertise<geometry_msgs::PoseStamped>("/LC/pose", 1);
 	link = Model->GetLink("LinkBaseBoard");

}

void LmazeControllerPlugin::OnWorldUpdateBegin(){
	ROS_INFO(" CONTROL %f, %f, %f ", c_msg_x, c_msg_y, c_msg_z );
	// Publish Maze Pose



	// stablize current pose
#if GAZEBO_MAJOR_VERSION >= 8
    Model->SetWorldPose(math::Pose3d(InitialPos, Model->WorldPose().Rot()));
#else
    Model->SetWorldPose(math::Pose(InitialPos, Model->GetWorldPose().rot));
#endif




	geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();

  	pose_stamped.pose.position.x = link->GetWorldPose().pos.x;
	pose_stamped.pose.position.y = link->GetWorldPose().pos.y;
	pose_stamped.pose.position.z = link->GetWorldPose().pos.z;

	pose_stamped.pose.orientation.x = link->GetWorldPose().rot.x;
	pose_stamped.pose.orientation.y = link->GetWorldPose().rot.y;
	pose_stamped.pose.orientation.z = link->GetWorldPose().rot.z;
	pose_stamped.pose.orientation.w = link->GetWorldPose().rot.w;    

	pub.publish(pose_stamped);


	// Model position in the beginning was in 
	//  ** InitialPos
	// Model pose in this cycle is InitialPos and InitialRot as built below
	math::Quaternion InitialRot = Model->GetWorldPose().rot;
	// build the model pose as per expectation
	


	// Build rotation from the value received


	// to
	relTransform.setOrigin(tf::Vector3(0, 0, 0));
	relTransform.setRotation(tf::Quaternion(0,0,0,1));
	// from 
	refTransform.setOrigin(tf::Vector3(0, 0, 0));
	refTransform.setRotation(tf::Quaternion(c_msg_x,c_msg_y,c_msg_z));
	// offset
	refTransform = refTransform.inverse();

	offsetTransform = refTransform.inverseTimes(relTransform);


	// Apply to model pose as above 

	refTransform.setOrigin( tf::Vector3( InitialPos.X(), InitialPos.Y(), InitialPos.Z() ) );
	refTransform.setRotation( tf::Quaternion( InitialRot.x, InitialRot.y, InitialRot.z, InitialRot.w ) );

	refTransform = refTransform.inverse();

	relTransform = refTransform.inverseTimes(offsetTransform);

	// set this moel pose

#if GAZEBO_MAJOR_VERSION >= 8
    Model->SetWorldPose(math::Pose3d(
											math::Vector3(
												relTransform.getOrigin().getX(),
												relTransform.getOrigin().getY(),
												relTransform.getOrigin().getZ()),
											math::Quaternion(
												relTransform.getRotation().getW(),
												relTransform.getRotation().getX(),
												relTransform.getRotation().getY(),
												relTransform.getRotation().getZ())));
#else
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
#endif

	c_msg_x = 0.0;
	c_msg_y = 0.0;
	c_msg_z = 0.0;

}

void LmazeControllerPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
void LmazeControllerPlugin::QueueThread1()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue1.callAvailable(ros::WallDuration(timeout));
  }
}
void LmazeControllerPlugin::OnVelUpdate(const geometry_msgs::Vector3::ConstPtr& msg){
	ROS_INFO(" received velocity [%f, %f, %f]",_msg_x, _msg_y, _msg_z);


    	_msg_x = msg->x; 
        _msg_y = msg->y; 
        _msg_z = msg->z; 	



	/* 

		// Simple Integrative Policy execution with max vel limit

	if( ((_msg_x + msg->x) > -0.05) && ((_msg_x + msg->x) < 0.05) )
    	_msg_x = _msg_x + msg->x; 	
	if( ((_msg_y + msg->y) > -0.05) && ((_msg_y + msg->y) < 0.05) )
        _msg_y = _msg_y + msg->y;  
	if( ((_msg_z + msg->z) > -0.05) && ((_msg_z + msg->z) < 0.05) )
        _msg_z = _msg_z + msg->z; 

	*/

	/*
		// Non Integrative Policy execution with relatively high velocity

	if( ((_msg_x + msg->x) > -0.05) && ((_msg_x + msg->x) < 0.05) )
    	_msg_x = msg->x; 	
	if( ((_msg_y + msg->y) > -0.05) && ((_msg_y + msg->y) < 0.05) )
        _msg_y = msg->y;  
	if( ((_msg_z + msg->z) > -0.05) && ((_msg_z + msg->z) < 0.05) )
        _msg_z = msg->z;

	*/

	/*	// Integrative Policy execution with direction based preemption

	if( (_msg_x >= 0 && msg->x < 0) || (_msg_x <= 0 && msg->x > 0) ) {
	    	_msg_x = -msg->x; 	
	} else 	if( ((_msg_x + msg->x) > -0.02) && ((_msg_x + msg->x) < 0.02) ){
    	_msg_x = _msg_x + msg->x;
	}

	if( (_msg_y >= 0 && msg->y < 0) || (_msg_y <= 0 && msg->y > 0) ) {
    	_msg_y = -msg->y; 	
	} else 	if( ((_msg_y + msg->y) > -0.02) && ((_msg_y + msg->y) < 0.02) ){
    	_msg_y = _msg_y + msg->y;
	}

	if( (_msg_z >= 0 && msg->z < 0) || (_msg_z <= 0 && msg->z > 0) ) {
    	_msg_z = -msg->z; 	
	} else 	if( ((_msg_z + msg->z) > -0.02) && ((_msg_z + msg->z) < 0.02) ){
    	_msg_z = _msg_z + msg->z;
	}*/

	c_msg_x = _msg_x;
	c_msg_y = _msg_y;
	c_msg_z = _msg_z;

}
void LmazeControllerPlugin::OnReset(const std_msgs::Bool::ConstPtr& msg){
	ROS_INFO(" reset velocity ");
	_msg_x = 0; 
    _msg_y = 0;  
    _msg_z = 0;
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LmazeControllerPlugin)
}
