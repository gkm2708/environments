/* Open-Source License with no liability whatsoever */

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
#include "std_msgs/Bool.h"

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

using namespace std;
namespace gazebo
{

class LmazeControllerPlugin : public ModelPlugin {
	private: 
		std::unique_ptr<ros::NodeHandle> rosNode;

		ros::Subscriber rosSub;
		ros::CallbackQueue rosQueue;
		thread rosQueueThread;

		ros::Subscriber rosSub1;
		ros::CallbackQueue rosQueue1;
		thread rosQueueThread1;

		ros::Subscriber rosSub2;
		ros::CallbackQueue rosQueue2;
		thread rosQueueThread2;

	public :
	    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

	    void OnVelUpdate(const geometry_msgs::Vector3::ConstPtr& msg);
		void OnReset(const std_msgs::Bool::ConstPtr& msg);
	    void OnBallUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg);

	    void OnWorldUpdateBegin();

	    void QueueThread() ;
	    void QueueThread1() ;
	    void QueueThread2() ;

	    physics::JointControllerPtr jointController;

	    event::ConnectionPtr updateConnectionOn;

	    std::vector<common::PID> controllers;
	    std::string maze_filename = "/homes/gkumar/rl/PrivateModelDevelopment1/sample_labyrinth_maze.mz";
	    common::Time t0;
	    common::Time t2;

	    math::Vector3 InitialPos;
	    int MAZE_SIZE = 0;
	    float _msg_x, _msg_y, _msg_z;
	    float _torque_x, _torque_y, _torque_z;
	    float c_msg_x, c_msg_y, c_msg_z;
	    int update_num = 0;

		ros::Publisher pub;

	    std::string centerLink;
	    physics::ModelPtr Model;
	    physics::LinkPtr link;

};
}

