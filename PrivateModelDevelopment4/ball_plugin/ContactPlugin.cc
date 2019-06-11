#include "ContactPlugin.hh"
#include <chrono>
#include <map>
#include <iostream>
#include <list>


namespace gazebo
{


void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {
	// Get the parent sensor.
	this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  	// Make sure the parent sensor is valid.
  	if (!this->parentSensor) {
    	gzerr << "ContactPlugin requires a ContactSensor.\n";
    	return;
  	}

  	// Connect to the sensor update event.
  	this->updateConnection = this->parentSensor->ConnectUpdated( std::bind(&ContactPlugin::OnUpdate, this));

  	// Make sure the parent sensor is active.
  	this->parentSensor->SetActive(true);

  	if (!ros::isInitialized()) {
	   	int argc = 0;
	   	char **argv = NULL;
	   	ros::init(argc, argv, "CP",
       	ros::init_options::NoSigintHandler);
   	}

	this->rosNode.reset(new ros::NodeHandle("CP"));

	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Vector3>("/LP/goalPos",1,boost::bind(&ContactPlugin::OnReset, this, _1),ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);
	this->rosQueueThread =  thread(std::bind(&ContactPlugin::QueueThread, this));

  	pubReward = nh.advertise<geometry_msgs::Vector3Stamped>("/CP/reward", 1);

}




//Reward -0.01 0 1
void ContactPlugin::OnUpdate()
{
  	// Get all the contacts.
  	msgs::Contacts contacts;
  	contacts = this->parentSensor->Contacts();

  	geometry_msgs::Vector3Stamped reward3Dstamped;

 	reward3Dstamped.vector.x = 0.0;
	reward3Dstamped.vector.y = 0;
	reward3Dstamped.vector.z = 0;

  	for ( int i = 0; i < contacts.contact_size(); ++i)
  	{
		//std::list<float> torqueVal;
		if(contacts.contact(i).collision2() == "ground_plane::link::collision" 
			|| contacts.contact(i).collision2() == "LMAZE::LinkBaseBoard::BASEMENT_COLISSION_BASE") {
			reward3Dstamped.vector.x = -0.1; 
			reward3Dstamped.vector.y = 0;
			reward3Dstamped.vector.z = 0;
		}
		else if(contacts.contact(i).collision2() == "LMAZE::LinkMainBasement::Base_Tiles"+std::to_string(goal_i)+"_"+std::to_string(goal_j)) {
			reward3Dstamped.vector.x = 1.0; 
			reward3Dstamped.vector.y = 0;
			reward3Dstamped.vector.z = 0;
			}
		//else { gzmsg << " else of contact" << contacts.contact(i).collision2() << std::endl ;			}

  	}
		    reward3Dstamped.header.stamp = ros::Time::now();
			pubReward.publish(reward3Dstamped);
}


	void ContactPlugin::QueueThread()
	{
	    static const double timeout = 0.01;
	    while (this->rosNode->ok())
	    {
	        this->rosQueue.callAvailable(ros::WallDuration(timeout));
	    }
	}

void ContactPlugin::OnReset(const geometry_msgs::Vector3::ConstPtr& msg){

    goal_i = (int)msg->x; 	
    goal_j = (int)msg->y;  
	gzmsg << goal_i << " " << goal_j << std::endl;
}
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
}


