/* Open-Source License with no liability whatsoever */

#include "LmazeControllerPlugin.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>


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



	    double cradius = 0.006;
	    float scaleX = 0.05;
	    float scaleY = 0.05;
	    float groundOffset = MAZE_SIZE*scaleX;
	    float floorHeight = 0.025; 		// should be equal to z axis scaling of cube for Wall Model; currently "0.05"
	    float floorThickness = 0.001; 	// should be equal to z axis scaling of cube for floor Model; currently "0.0155"

#if GAZEBO_MAJOR_VERSION >= 8
    std::string modelName = Model->GetName();
//    math::Vector3d InitialPos = Model->WorldPose().Pos();
	InitialPos = math::Vector3d(MAZE_SIZE*scaleX/2, MAZE_SIZE*scaleY/2, groundOffset+2*cradius+floorThickness/2);

#else
    std::string modelName = Model->GetName();
//    math::Vector3 InitialPos = Model->GetWorldPose().pos;
	InitialPos = math::Vector3(MAZE_SIZE*scaleX/2, MAZE_SIZE*scaleY/2, groundOffset+2*cradius+floorThickness/2);

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
    double angular_p = 60.0;
    double angular_i = 1;
    double angular_d = 0.1;
	double angular_imax = 0.5;

    double _maxTorque = 100.0;
    double _maxForce = 50.0;
  
    link = Model->GetLink("LinkMainBasement");

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

#if GAZEBO_MAJOR_VERSION >= 8
	math::Pose3d linkPose = link->WorldPose();
  	pose_stamped.pose.position.x = linkPose.Pos().X();
	pose_stamped.pose.position.y = linkPose.Pos().Y();
	pose_stamped.pose.position.z = linkPose.Pos().Z();
	pose_stamped.pose.orientation.x = linkPose.Rot().X();
	pose_stamped.pose.orientation.y = linkPose.Rot().Y();
	pose_stamped.pose.orientation.z = linkPose.Rot().Z();
	pose_stamped.pose.orientation.w = linkPose.Rot().W();    

	math::Quaterniond _orient = math::Quaterniond(linkPose.Rot().W(),
												linkPose.Rot().X(),
												linkPose.Rot().Y(), 
												linkPose.Rot().Z());
	math::Vector3d _or = _orient.Euler();

	pub.publish(pose_stamped);


    // ##################### Values for PID Controller #############################

	if(c_msg_x != 0 || c_msg_y != 0 || c_msg_z != 0) {

	// find dt from last to current step
    common::Time t1 = common::Time::GetWallTime();
	common::Time dt = t1 - t0;
    double dtd = (t1 - t0).common::Time::Double();
    t0 = common::Time::GetWallTime();


	math::Pose3d modelPose = Model->WorldPose();
	double _cx = modelPose.Pos().X();
	double _cy = modelPose.Pos().Y();
	double _cz = modelPose.Pos().Z();
	double _xt = ( _cx - InitialPos.X())/dtd;
    double _yt = ( _cy - InitialPos.Y())/dtd;
    double _zt = ( _cz - InitialPos.Z())/dtd;


	double _yawt = -(_or.Z()/dtd);

    // Calculate the error between actual and target velocities
    math::Vector3d curLinearVel = link->WorldLinearVel();
    math::Vector3d targetLinearVel = math::Vector3d(-_xt,-_yt,-_zt);
    math::Vector3d linearError = curLinearVel - targetLinearVel;
	
    math::Vector3d curAngularVel = link->WorldAngularVel();
    math::Vector3d targetAngularVel = math::Vector3d(c_msg_x, c_msg_y, -curAngularVel.Z());
    //math::Vector3 targetAngularVel = math::Vector3(c_msg_x, c_msg_y, 0);
    //math::Vector3 targetAngularVel = math::Vector3(c_msg_x, c_msg_y, _yawt);
    //math::Vector3 targetAngularVel = math::Vector3(c_msg_x, c_msg_y, -_or.z);
    math::Vector3d angularError = curAngularVel - targetAngularVel;
	
    // Get forces to apply from controllers
    math::Vector3d worldForce = math::Vector3d(this->controllers[0].Update(linearError.X(), dt),
												this->controllers[2].Update(linearError.Y(), dt),
												this->controllers[4].Update(linearError.Z(), dt));
    math::Vector3d worldTorque = math::Vector3d(this->controllers[1].Update(angularError.X(), dt),
												this->controllers[3].Update(angularError.Y(), dt),
												this->controllers[5].Update(angularError.Z(), dt));




	/*worldForce.x = this->controllers[0].Update(linearError.X(), dt);
    worldTorque.x = this->controllers[1].Update(angularError.X(), dt); // + gcTorque.x
    #worldForce.y = this->controllers[2].Update(linearError.Y(), dt);
    #worldTorque.y = this->controllers[3].Update(angularError.Y(), dt); // + gcTorque.y
    #worldForce.z = this->controllers[4].Update(linearError.Z(), dt);
    #worldTorque.z = this->controllers[5].Update(angularError.Z(), dt); // + gcTorque.z

	ROS_INFO(" CONTROL TORQUE %f, %f, %f ", worldTorque.x , worldTorque.y , worldTorque.z );*/

    link->AddForce(worldForce);
    link->AddTorque(worldTorque);
	}


#else
	math::Pose3 linkPose = link->GetWorldPose();
  	pose_stamped.pose.position.x = linkPose.pos.x;
	pose_stamped.pose.position.y = linkPose.pos.y;
	pose_stamped.pose.position.z = linkPose.pos.z;
	pose_stamped.pose.orientation.x = linkPose.rot.x;
	pose_stamped.pose.orientation.y = linkPose.rot.y;
	pose_stamped.pose.orientation.z = linkPose.rot.z;
	pose_stamped.pose.orientation.w = linkPose.rot.w;    

	math::Quaternion _orient = math::Quaternion(linkPose.rot.x,
												linkPose.rot.y,
												linkPose.rot.z, 
												linkPose.rot.w);
	math::Vector3 _or = _orient.GetAsEuler();



	pub.publish(pose_stamped);


    // ##################### Values for PID Controller #############################

	if(c_msg_x != 0 || c_msg_y != 0 || c_msg_z != 0) {

	// find dt from last to current step
    common::Time t1 = common::Time::GetWallTime();
	common::Time dt = t1 - t0;
    double dtd = (t1 - t0).common::Time::Double();
    t0 = common::Time::GetWallTime();


	math::Pose3 modelPose = Model->GetWorldPose();
	double _cx = modelPose.pos.x;
	double _cy = modelPose.pos.y;
	double _cz = modelPose.pos.z;
	double _xt = ( _cx - InitialPos.x)/dtd;
    double _yt = ( _cy - InitialPos.y)/dtd;
    double _zt = ( _cz - InitialPos.z)/dtd;

	double _yawt = -(_or.z/dtd);

    // Calculate the error between actual and target velocities
    math::Vector3 curLinearVel = link->GetWorldLinearVel();
    math::Vector3 targetLinearVel = math::Vector3(-_xt,-_yt,-_zt);
    math::Vector3 linearError = curLinearVel - targetLinearVel;
	
    math::Vector3 curAngularVel = link->GetWorldAngularVel();
    math::Vector3 targetAngularVel = math::Vector3(c_msg_x, c_msg_y, -curAngularVel.z);
    math::Vector3 angularError = curAngularVel - targetAngularVel;
	
    // Get forces to apply from controllers
    math::Vector3 worldForce;
    math::Vector3 worldTorque;

	worldForce.x = this->controllers[0].Update(linearError.x, dt);
    worldTorque.x = this->controllers[1].Update(angularError.x, dt); // + gcTorque.x
    worldForce.y = this->controllers[2].Update(linearError.y, dt);
    worldTorque.y = this->controllers[3].Update(angularError.y, dt); // + gcTorque.y
    worldForce.z = this->controllers[4].Update(linearError.z, dt);
    worldTorque.z = this->controllers[5].Update(angularError.z, dt); // + gcTorque.z

	//ROS_INFO(" CONTROL TORQUE %f, %f, %f ", worldTorque.x , worldTorque.y , worldTorque.z );

    link->AddForce(worldForce);
    link->AddTorque(worldTorque);
	}
#endif

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
    /*_msg_x = msg->x/3; 	
    _msg_y = msg->y/3;  
    _msg_z = msg->z/3;*/

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

#if GAZEBO_MAJOR_VERSION >= 8
	gcTorque = math::Vector3d(msg->pose.position.y*9.81, -msg->pose.position.x*9.81, 0);
#else
	gcTorque = math::Vector3(msg->pose.position.y*9.81, -msg->pose.position.x*9.81, 0);

	//gcTorque.x = msg->pose.position.y*9.81;
	//gcTorque.y = -msg->pose.position.x*9.81;
	//gcTorque.z = 0;
#endif

    
	link->AddTorque(gcTorque);
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LmazeControllerPlugin)
}
