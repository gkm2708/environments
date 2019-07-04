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
#include <math.h>       /* sin */

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

namespace gazebo
{


void LmazeControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
	gzmsg << "Load LMaze Controller main" << std::endl;

    	std::fstream fs;
    	fs.open(maze_filename, std::fstream::in);
    	if (fs.good()) {
    	    std::string line;
    	    std::getline(fs, line);
    	    MAZE_SIZE = line.size();
		}

    Model = _parent;

#if GAZEBO_MAJOR_VERSION >= 8
    std::string modelName = Model->GetName();
//    math::Vector3d InitialPos = Model->WorldPose().Pos();
#else
    std::string modelName = Model->GetName();
//    math::Vector3 InitialPos = Model->GetWorldPose().pos;
#endif

	    double cradius = 0.006;
	    float scaleX = 0.05;
	    float scaleY = 0.05;
	    float groundOffset = MAZE_SIZE*scaleX;
	    float floorHeight = 0.025; 		// should be equal to z axis scaling of cube for Wall Model; currently "0.05"
	    float floorThickness = 0.001; 	// should be equal to z axis scaling of cube for floor Model; currently "0.0155"


	InitialPos = math::Vector3(MAZE_SIZE*scaleX/2, MAZE_SIZE*scaleY/2, groundOffset+2*cradius+floorThickness/2);


	gzmsg << modelName << std::endl;

    //  ################## ROS Node #############################
    if (!ros::isInitialized()) {
		int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LC",
        ros::init_options::NoSigintHandler);
	}

    this->rosNode.reset(new ros::NodeHandle("LC"));

	// to execute actions
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/interface/action",1,boost::bind(&LmazeControllerPlugin::OnVelUpdate, this, _1),ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    this->rosQueueThread =  thread(std::bind(&LmazeControllerPlugin::QueueThread, this));

	// to reset velocities
	ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Bool>("/LP/reset",1,boost::bind(&LmazeControllerPlugin::OnReset, this, _1),ros::VoidPtr(), &this->rosQueue1);
 	this->rosSub1 = this->rosNode->subscribe(so1);
	this->rosQueueThread1 =  thread(std::bind(&LmazeControllerPlugin::QueueThread1, this));


	// for gravity compensation
    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/BC/pose",1,boost::bind(&LmazeControllerPlugin::OnBallUpdate, this, _1),ros::VoidPtr(), &this->rosQueue2);
    this->rosSub2 = this->rosNode->subscribe(so2);
    this->rosQueueThread2 =  thread(std::bind(&LmazeControllerPlugin::QueueThread2, this));



    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&LmazeControllerPlugin::OnWorldUpdateBegin, this));
	pub = rosNode->advertise<geometry_msgs::PoseStamped>("/LC/pose", 1);

    // ##################### Values for PID Controller #############################

    t0 = common::Time::GetWallTime();
    t2 = common::Time::GetWallTime();

    double linear_p = 100.0;
    double linear_i = 1;
    double linear_d = 0.1;
    double linear_imax = 1.0;
    double angular_p = 100.0;
    double angular_i = 1;
    double angular_d = 0.1;
	double angular_imax = 0.5;

    double _maxTorque = 100.0;
    double _maxForce = 50.0;
  
    link = Model->GetLink("LinkMaze");

    for (int i = 0; i < 3; i++)  {
        common::PID controller_translation(linear_p, linear_i, linear_d, linear_imax, -linear_imax, _maxForce, -_maxForce);
        common::PID controller_rotation(angular_p, angular_i, angular_d, angular_imax, -angular_imax, _maxTorque, -_maxTorque);
        this->controllers.push_back(controller_translation);
        this->controllers.push_back(controller_rotation);
    }
}

void LmazeControllerPlugin::OnWorldUpdateBegin(){
	//ROS_INFO(" CONTROL CYCLE");
	// Publish Maze Pose

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

	math::Quaternion _orient = math::Quaternion(pose_stamped.pose.orientation.x,
												pose_stamped.pose.orientation.y,
												pose_stamped.pose.orientation.z, 
												pose_stamped.pose.orientation.w);
	math::Vector3 _or = _orient.GetAsEuler();

    // ##################### Values for PID Controller #############################

	if(c_msg_x != 0 || c_msg_y != 0 || c_msg_z != 0) {

		// find dt from last to current step
	    common::Time t1 = common::Time::GetWallTime();
		common::Time dt = t1 - t0;
	    double dtd = (t1 - t0).common::Time::Double();
	    t0 = common::Time::GetWallTime();
	
		// Model's Current position
		double _cx = Model->GetWorldPose().pos.x;
		double _cy = Model->GetWorldPose().pos.y;
		double _cz = Model->GetWorldPose().pos.z;

		// error between current and initial position divided by time lapse 
		// gives the velocity with which the board is falling down under gravity
		double _xt = ( _cx - InitialPos.x)/dtd;
	    double _yt = ( _cy - InitialPos.y)/dtd;
	    double _zt = ( _cz - InitialPos.z)/dtd;


	    // Calculate the error between actual and target linear velocities
	    math::Vector3 curLinearVel = link->GetWorldLinearVel();
	    math::Vector3 targetLinearVel = math::Vector3(-_xt,-_yt,-_zt);
	    math::Vector3 linearError = curLinearVel - targetLinearVel;
	
	    // Calculate the error between actual and target Angular velocities
	    math::Vector3 curAngularVel = link->GetWorldAngularVel();
	    math::Vector3 targetAngularVel = math::Vector3(5*c_msg_x, 5*c_msg_y, -curAngularVel.z);
	    math::Vector3 angularError = curAngularVel - targetAngularVel;
	
	    // Get forces to apply from controllers
	    math::Vector3 worldForce;
	    math::Vector3 worldTorque;

		worldForce.x = this->controllers[0].Update(linearError.x, dt);
	    worldTorque.x = this->controllers[1].Update(angularError.x, dt); // + gcTorque.x;
	    worldForce.y = this->controllers[2].Update(linearError.y, dt);
	    worldTorque.y = this->controllers[3].Update(angularError.y, dt); // + gcTorque.y;
	    worldForce.z = this->controllers[4].Update(linearError.z, dt);
	    worldTorque.z = this->controllers[5].Update(angularError.z, dt); // + gcTorque.z
	
		//gzmsg << " Control Internal " << worldForce.x << " " << worldForce.y << " " << worldForce.z << " " << worldTorque.x << " " << worldTorque.y << " " << worldTorque.z <<  std::endl;

	    link->AddForce(worldForce);
	    link->AddTorque(worldTorque);
	}
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
void LmazeControllerPlugin::QueueThread2()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue2.callAvailable(ros::WallDuration(timeout));
  }
}
void LmazeControllerPlugin::OnVelUpdate(const geometry_msgs::Vector3::ConstPtr& msg){
	//ROS_INFO(" received velocity [%f, %f, %f]",_msg_x, _msg_y, _msg_z);
	//gzmsg << " set velocity " << _msg_x << " " << _msg_y << " "<< _msg_z << std::endl;

    _msg_x = msg->x; 	
    _msg_y = msg->y;  
    _msg_z = msg->z;

	c_msg_x = _msg_x;
	c_msg_y = _msg_y;
	c_msg_z = _msg_z;
}
void LmazeControllerPlugin::OnReset(const std_msgs::Bool::ConstPtr& msg){
	gzmsg << " reset velocity " << std::endl;
	_msg_x = 0; 
    _msg_y = 0;  
    _msg_z = 0;
}

void LmazeControllerPlugin::OnBallUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO(" New Ball Pose ");

	gcTorque.x = msg->pose.position.y*9.81;
	gcTorque.y = -msg->pose.position.x*9.81;
	gcTorque.z = 0;


	//gcTorque.x = msg->pose.position.y*9.81*(4/3*3.14*0.015*0.015*0.015);
	//gcTorque.y = -msg->pose.position.x*9.81*(4/3*3.14*0.015*0.015*0.015);
	//gcTorque.z = 0;
    
	link->AddTorque(gcTorque);
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LmazeControllerPlugin)
}


		// check if your board has gone beyond limits and apply velocities only if not.

		/*if( _or.y > 0.001 ) 		{ c_msg_x = -2*(_or.y - 0.001); gzmsg << " Condition 1 " << _or.y << ", " << c_msg_x << std::endl;}
		else if( _or.y < -0.001 ) 	{ c_msg_x = 2*(_or.y + 0.001); 	gzmsg << " Condition 2 " << _or.y << ", " << c_msg_x << std::endl;} 
		else 						{								gzmsg << _or.y << ", " << c_msg_x << std::endl;}

		if( _or.x > 0.001 ) 		{ c_msg_y = -2*(_or.x - 0.001); gzmsg << " Condition 3 " << _or.x << ", " << c_msg_y << std::endl;}
		else if( _or.x < -0.001 ) 	{ c_msg_y = 2*(_or.x + 0.001); 	gzmsg << " Condition 4 " << _or.x << ", " << c_msg_y << std::endl;} 
		else 						{								gzmsg << _or.x << ", " << c_msg_y << std::endl;}
		*/
		//if(_or.y >= 0 && _msg_x > 0 && _msg_y == 0) { c_msg_x = 2*(_or.y - _msg_x); 	gzmsg << " Rev Condition 1 " << _or.y << ", " << c_msg_x << std::endl;}
		//if(_or.y <= 0 && _msg_x < 0 && _msg_y == 0) { c_msg_x = 2*(_or.y - _msg_x); 	gzmsg << " Rev Condition 2 " << _or.y << ", " << c_msg_x << std::endl;}		
		//if(_or.x >= 0 && _msg_y > 0 && _msg_x == 0) { c_msg_y = 2*(_or.y - _msg_y); 	gzmsg << " Rev Condition 3 " << _or.x << ", " << c_msg_y << std::endl;}
		//if(_or.x <= 0 && _msg_y < 0 && _msg_x == 0) { c_msg_y = 2*(_or.y - _msg_y); 	gzmsg << " Rev Condition 4 " << _or.x << ", " << c_msg_y << std::endl;}


	/*
	gzmsg << " Model Pose " << Model->GetWorldPose() << std::endl;
	gzmsg << " Link Pose " << link->GetWorldPose() << std::endl;
	gzmsg << " Link Pose " << link->GetWorldPose().pos.x << " " << link->GetWorldPose().pos.y << " " << link->GetWorldPose().pos.z << " "
			<< link->GetWorldPose().rot.x << " " << link->GetWorldPose().rot.y << " "<< link->GetWorldPose().rot.z << " "<< link->GetWorldPose().rot.w << std::endl;

	if(c_msg_x > 0)
		link->SetWorldPose(math::Pose(math::Vector3(link->GetWorldPose().pos.x, link->GetWorldPose().pos.y, link->GetWorldPose().pos.z),
							math::Quaternion(0.087, 0, 0, 0.996)));
	if(c_msg_x < 0)
		link->SetWorldPose(math::Pose(math::Vector3(link->GetWorldPose().pos.x, link->GetWorldPose().pos.y, link->GetWorldPose().pos.z),
							math::Quaternion(-0.087, 0, 0, 0.996)));
	if(c_msg_y > 0)
		link->SetWorldPose(math::Pose(math::Vector3(link->GetWorldPose().pos.x, link->GetWorldPose().pos.y, link->GetWorldPose().pos.z),
							math::Quaternion(0, 0.087, 0, 0.996)));
	if(c_msg_y < 0)
		link->SetWorldPose(math::Pose(math::Vector3(link->GetWorldPose().pos.x, link->GetWorldPose().pos.y, link->GetWorldPose().pos.z),
							math::Quaternion(0, -0.087, 0, 0.996)));
	*/
	//double _yawt = -(_or.z/dtd);

