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
#include "LmazePlugin.hh"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/TransportIface.hh"
#include "geometry_msgs/PoseStamped.h"

#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3Stamped.h"

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif


namespace gazebo
{


	std::string starting_tag = "<sdf version='1.6'><model name='LMAZE'>";
	std::string ending_tag = "<plugin name='lmaze_controller_plugin' filename='/homes/gkumar/rl/PrivateModelDevelopment4/build/liblmaze_controller_plugin.so'> \
		        	    		<update_rate>0.0</update_rate> \
							</plugin> \
        										<static>0</static> \
        										<allow_auto_disable>1</allow_auto_disable> \
        										</model>\
        										</sdf>";

    // ******************************************************************************************************
    // ************************************************* LOAD  **********************************************
    // ******************************************************************************************************


	void LmazePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){

		this->gzNode = transport::NodePtr(new transport::Node());
		this->gzNode->Init();
		gzmsg << this->gzNode->GetId() << std::endl;
		visualPub = this->gzNode->Advertise<msgs::Visual>("/gazebo/default/visual");
		gzmsg << visualPub << std::endl;

    	World = _parent;

    	loadSDF();

		if (!ros::isInitialized())	{
	        int argc = 0;
	        char **argv = NULL;
	        ros::init(argc, argv, "LP",ros::init_options::NoSigintHandler);
	    }


	    this->rosNode.reset(new ros::NodeHandle("LP"));

	    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>("/interface/reset",1,boost::bind(&LmazePlugin::OnReset, this, _1),ros::VoidPtr(), &this->rosQueue);
	    this->rosSub = this->rosNode->subscribe(so);
	    this->rosQueueThread =  thread(std::bind(&LmazePlugin::QueueThread, this));

	    /*ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Bool>("/interface/initialize",1,boost::bind(&LmazePlugin::OnInit, this, _1),ros::VoidPtr(), &this->rosQueue1);
	    this->rosSub1 = this->rosNode->subscribe(so1);
	    this->rosQueueThread1 =  thread(std::bind(&LmazePlugin::QueueThread1, this));*/

	    ros::SubscribeOptions so2 = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/BC/pose",1,boost::bind(&LmazePlugin::OnBallUpdate, this, _1),ros::VoidPtr(), &this->rosQueue2);
	    this->rosSub2 = this->rosNode->subscribe(so2);
	    this->rosQueueThread2 =  thread(std::bind(&LmazePlugin::QueueThread2, this));


		pub = nh.advertise<std_msgs::Bool>("/LP/reset", 1);
		pubGoal = nh.advertise<geometry_msgs::Vector3>("/LP/goalPos", 1);
  		pubReward = nh.advertise<geometry_msgs::Vector3Stamped>("/LP/reward", 1);

		if(goal_i == -1 && goal_j == -1 ){

			goalRandom = 1;
			rnd();

		}
	    updateConnectionOn = event::Events::ConnectWorldUpdateBegin(boost::bind(&LmazePlugin::OnWorldUpdateBegin, this));
    	gzmsg << "Load LMaze Model" << std::endl;
	}





void LmazePlugin::QueueThread2()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue2.callAvailable(ros::WallDuration(timeout));
  }
}

void LmazePlugin::OnBallUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg){
	//ROS_INFO(" New Ball Pose ");

	ballPose.x = msg->pose.position.x;
	ballPose.y = msg->pose.position.y;
	ballPose.z = msg->pose.position.z;
    //link->AddTorque(gcTorque);
}


void LmazePlugin::OnWorldUpdateBegin(){
    //gzmsg << "LmazePlugin OnWorldUpdateBegin" << std::endl;

  	geometry_msgs::Vector3Stamped reward3Dstamped;


	if(ballPose.x <= ((MAZE_SIZE-2*goal_i)*scaleX/2) && 
		ballPose.x >= ((MAZE_SIZE-2*goal_i-2)*scaleX/2) &&
		ballPose.y <= ((MAZE_SIZE-2*goal_j)*scaleY/2) &&
		ballPose.y >= ((MAZE_SIZE-2*goal_j-2)*scaleY/2) ){

	//gzmsg << " goal " << std::endl;
 	reward3Dstamped.vector.x = 1.0;
	reward3Dstamped.vector.y = 0;
	reward3Dstamped.vector.z = 0;
	reward3Dstamped.header.stamp = ros::Time::now();
	pubReward.publish(reward3Dstamped);
	//World->SetPaused(1);
	//std::this_thread::sleep_for(2);
	//World->Reset();
	//World->SetPaused(0);

	} else {
 	reward3Dstamped.vector.x = 0;
	reward3Dstamped.vector.y = 0;
	reward3Dstamped.vector.z = 0;
	reward3Dstamped.header.stamp = ros::Time::now();
	pubReward.publish(reward3Dstamped);
	}


}

    // ******************************************************************************************************
    // ********************************************** LOAD SDF **********************************************
    // ******************************************************************************************************


	void LmazePlugin::loadSDF(){

    	gzmsg << " Inside LmazePlugin \t\t Load Method" << std::endl;

    	// Global
    	std::fstream fs;
    	fs.open(maze_filename, std::fstream::in);
    	if (fs.good()) {

    	    std::string line;
    	    std::string tempSDF = "";

    	    std::getline(fs, line);
    	    MAZE_SIZE = line.size();

		    groundOffset = MAZE_SIZE*scaleX;

    	    fs.close();

    	    tempSDF = tempSDF + drawKinObj("0 0 0.0005 0 -0 0");                                                           // a kinematic only object with no physics for the base

    	    tempSDF = tempSDF + drawJointSphere(); 																		// draw a sphere at the center of cylinder

			tempSDF = tempSDF + drawBaseBoard("0 0 "
    	                                     + std::to_string(groundOffset+2*cradius+floorThickness/2)
    	                                     + " 0 -0 0"); 																// draw basement and walls evenly

    	    tempSDF = tempSDF + drawBasement("0 0 "
    	                                     + std::to_string(groundOffset+2*cradius+floorThickness+floorHeight/2)
    	                                     + " 0 -0 0"); 																// draw basement and walls evenly
			
	        sdf::SDF unitSDF;

    	    unitSDF.SetFromString(starting_tag 
									+ tempSDF 
									+ ending_tag);
    	    World->InsertModelSDF(unitSDF);

	    } else {gzmsg << "filestream error" << std::endl;}

		#if GAZEBO_MAJOR_VERSION >= 8
    		std::string worldName = World->Name();
		#else
    		std::string worldName = World->GetName();
		#endif
			gzmsg << worldName << std::endl;
	}



    // ******************************************************************************************************
    // *************************************   BUILD KINEMATIC ONLY LINK  ***********************************
    // ******************************************************************************************************
	// A Kinematic object to hold the spherical object for ball joint in the air



	std::string LmazePlugin::drawKinObj(std::string pose) {
    				return 		"<link name='LinkKinObj'> <kinematic>true</kinematic> \
						 			<pose>"
            					+ pose
            					+ "</pose> \
            						<collision name='collision'> \
            						<geometry><box><size>0.001 0.001 0.001</size></box></geometry> \
            						<surface><friction><ode><mu>0.1</mu><mu2>0</mu2></ode></friction></surface> \
            						</collision> \
            						</link>";
	} 

	std::string LmazePlugin::drawCylObj(std::string pose) {
    				return 		"<link name='LinkCylObj'><kinematic>true</kinematic>  \
						 			<pose>"
            					+ pose
            					+ "</pose> \
            						<collision name='collisionCyl'> \
            						<geometry><cylinder>\
									<radius>"
								+ std::to_string(MAZE_SIZE/2*scaleX)
								+	"</radius>\
									<length>"
								+ std::to_string(MAZE_SIZE*3/32*scaleX)
								+	"</length>\
									</cylinder></geometry> \
            						<surface><friction><ode><mu>0.1</mu><mu2>0</mu2></ode></friction></surface> \
            						</collision> \
            						</link> \
									<self_collide>true</self_collide>";
//					            + "<joint name='JointCylBase' type='fixed'><parent>ground_plane:link</parent><child>linkCylObj</child></joint>";
	} 




    // ******************************************************************************************************
    // *******************************************   BUILD JOINT SPHERE  ************************************
    // ******************************************************************************************************
	// A sphere at a (X, Y, Z) i.e. a function of maze size


	std::string LmazePlugin::drawJointSphere()	{
	    std::string tempSDF = "";

	    std::string link_tag_until_pose1 = "<link name='LinkJointSphere'> <pose>";

	    std::string link_tag_until_radius1 = "</pose>\
            <collision name='collision'> \
            <geometry> \
            <sphere> \
            <radius>";

        std::string link_tag_until_cradius1 =	"</radius> \
            </sphere> \
            </geometry> \
			<surface> \
	        <friction> \
	          <ode> \
	            <mu>0.01</mu> \
	            <mu2>0.01</mu2> \
	          </ode> \
	        </friction> \
	        </surface> \
            </collision> \
            <visual name='visual'> \
            <geometry> \
            <sphere> \
            <radius>";

        std::string link_tag_until_end1 = "</radius> \
            </sphere> \
            </geometry> \
            <material> \
            <script>Gazebo/Black</script> \
            </material> \
            </visual> \
            </link> ";

        tempSDF = link_tag_until_pose1
			+ "0 0 "
            + std::to_string(groundOffset+cradius) + " 0 -0 0"
            + link_tag_until_radius1 + std::to_string(cradius)
            + link_tag_until_cradius1 + std::to_string(cradius)
            + link_tag_until_end1;
		
    	tempSDF = tempSDF + "<joint name='JointSphereKinObj' type='fixed'>\
												<parent>LinkKinObj</parent>\
												<child>LinkJointSphere</child></joint>";
		
		/*tempSDF = tempSDF + "<joint name='JointSphereCylObj' type='fixed'>\
												<parent>LinkCylObj</parent>\
												<child>LinkJointSphere</child></joint>";*/

	    return tempSDF;
	}




    // ******************************************************************************************************
    // ****************************************   BUILD BASE BOARD LINK  ************************************
    // ******************************************************************************************************


	std::string LmazePlugin::drawBaseBoard(std::string pose){

	    std::string link_tag_Outer = "<link name='";
			//  + linkName
		std::string link_tag_Outer1 = "'><pose frame=''> ";
			//	+ Above JointSphere
    	std::string link_tag_Outer2 = "</pose>";
			//	+ Iterate over below tags for multiple collission and visuals
    	std::string visual_tag_until_name = "<visual name='";
    		//  + visual name
    	std::string visual_tag_until_pose = "'><pose>";
    		//  + visual pose
    	std::string visual_tag_until_size = "</pose> <geometry> <box> <size>";
    		//	+ visual size as in scaling factor
    	std::string visual_tag_until_end = "</size>\
            </box> \
            </geometry> \
            <material> \
            <lighting>1</lighting> \
            <script> \
            <uri>file://media/materials/scripts/gazebo.material</uri> \
            <name>Gazebo/Yellow</name> \
            </script> \
            <shader type='pixel'/> \
            </material> \
            <transparency>0</transparency> \
            <cast_shadows>0</cast_shadows> \
            </visual>";

    		// +
    	std::string colission_tag_until_name = "<collision name='";
    		//colission name
    	std::string colission_tag_until_pose = "'><pose>";
    		//collision pose
    	std::string colission_tag_until_size = "</pose><geometry> <box> <size>";
    		//collision name
    	std::string colission_tag_until_end = "</size>\
            </box> \
            </geometry> \
			<surface> \
	        <friction> \
	          <ode> \
	            <mu>0.01</mu> \
	            <mu2>0.01</mu2> \
	          </ode> \
	        </friction> \
	        </surface> \
            </collision>";
			// Finally add closing tag to decare it as one link
    	std::string link_tag_Outer10 = "</link>";

	    std::string scale_xyz = std::to_string(scaleX*MAZE_SIZE) + " " 
							+ std::to_string(scaleY*MAZE_SIZE) + " " 
							+ std::to_string(floorThickness);

		// Touch This carefully ;)
    	return link_tag_Outer																									//  Only the outer link value
		            + "LinkBaseBoard"
            + link_tag_Outer1
					+ pose
            + link_tag_Outer2
            + colission_tag_until_name																							// Collission tag for base board
		            + "BASEMENT_COLISSION_BASE"
            + colission_tag_until_pose
		            + "0 0 0 0 -0 0"                                                                         					//
            + colission_tag_until_size
		            + scale_xyz
            + colission_tag_until_end
            + visual_tag_until_name																								// visual tag for base board
		            + "BASEMENT_VISUAL_BASE"
            + visual_tag_until_pose
		            + "0 0 0 0 -0 0"                                                                                			//
            + visual_tag_until_size
		            + scale_xyz                                                          										// size
            + visual_tag_until_end
            + link_tag_Outer10																								 	// finished link
            		+ "<joint name='JointBaseBoard' type='ball'>\
							<parent>LinkJointSphere</parent>\
							<child>LinkBaseBoard</child>\
						</joint>";
																							// joint to the JointSphere
	}




    // ******************************************************************************************************
    // ***************************************   BUILD REST OF THE BOARD  ***********************************
    // ******************************************************************************************************


	// A Maze board consisting of 
	//								a base board, (As one link for collission detection)
	//								side walls,
	//								and top board with pits or solid surface 
	//											with unit drawn from samplele_labyrinth_maze.mz file
	//											as one single link

	/*
	<inertial> \
            <mass>7.85</mass> \
			</inertial> \
	*/


	std::string LmazePlugin::drawBasement(std::string pose){

	    std::string link_tag_Outer = "<link name='";
			//  + linkName
		std::string link_tag_Outer1 = "'> \
			<pose frame=''> ";
			//	+ Above JointSphere
    	std::string link_tag_Outer2 = "</pose>";
			//	+ Iterate over below tags for multiple collission and visuals
    	std::string visual_tag_until_name = "<visual name='";
    		//  + visual name
    	std::string visual_tag_until_pose = "'><pose>";
		    //  + visual pose
	    std::string visual_tag_until_size = "</pose> <geometry> <box> <size>";
    		//	+ visual size as in scaling factor
    	std::string visual_tag_until_end = "</size>\
            </box> \
            </geometry> \
            <material> \
            <lighting>1</lighting> \
            <script> \
            <uri>file://media/materials/scripts/gazebo.material</uri> \
            <name>Gazebo/Yellow</name> \
            </script> \
            <shader type='pixel'/> \
            </material> \
            <transparency>0</transparency> \
            <cast_shadows>0</cast_shadows> \
            </visual>";

    		// +
    	std::string colission_tag_until_name = "<collision name='";
    		//colission name
    	std::string colission_tag_until_pose = "'><pose>";
    		//collision pose
    	std::string colission_tag_until_size = "</pose><geometry> <box> <size>";
    		//collision name
    	std::string colission_tag_until_end = "</size>\
            </box> \
            </geometry> \
			<surface> \
	        <friction> \
	          <ode> \
	            <mu>0.01</mu> \
	            <mu2>0.01</mu2> \
	          </ode> \
	        </friction> \
	        </surface> \
            </collision>";
			// FInally add closing tag to decare it as one link
    	std::string link_tag_Outer10 = "</link>";

			// Build temp as intermediate collission and visual object 
    	std::string tempSDF = "";


			// size of walls making the basement
    	std::string scale_xyz = std::to_string(scaleX) + " " + std::to_string(scaleY) + " " + std::to_string(floorHeight);
		std::string link_name = "Basement_Walls_";

		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   BUILD WALLS   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			// Outer wall building loop
    	for (int i=MAZE_SIZE-1; i >= 0; i--){
    	    for (int j=0, k=MAZE_SIZE-1; j < MAZE_SIZE; j++, k--){
    	        if (i == 0 || i == MAZE_SIZE-1 || j == 0 || j == MAZE_SIZE-1){
    	            //gzmsg << i << " " << j << std::endl;
    	            tempSDF = tempSDF
							// collission tag generation
    	                    + colission_tag_until_name
			                        + link_name + std::to_string(i) + "_" + std::to_string(k)										// name for collission (same for visual)
    	                    + colission_tag_until_pose
    		                        + std::to_string((2*i-MAZE_SIZE)*scaleX/2 + scaleX/2) + " "
    		                        + std::to_string((2*k-MAZE_SIZE)*scaleX/2 + scaleX/2) + " "											// Pose of the
    		                        + " 0 0 -0 0"                                                                       				// wall block
    	                    + colission_tag_until_size
    	                    		+ scale_xyz					                                                              		// size
    	                    + colission_tag_until_end
								// visual tag generation
   		                    + visual_tag_until_name
			                        + link_name + std::to_string(i) + "_" + std::to_string(k)										// name for visual (same as collission)
  	                        + visual_tag_until_pose
		                            + std::to_string((2*i-MAZE_SIZE)*scaleX/2 + scaleX/2) + " "
        		                    + std::to_string((2*k-MAZE_SIZE)*scaleX/2 + scaleX/2) + " "											// Pose of the
        		                    + " 0 0 -0 0"                                                                       				// wall block
        	                + visual_tag_until_size
        	                		+ scale_xyz					                                                              		// size
        	                + visual_tag_until_end;
        	    }
        	}
    	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   BUILT WALLS   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++   REDEFINE VARIABLES FOR BASE   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// the variables are redefined because unlike basement and walls, base consist of non-standard shapes driven by mesh.
	// The meshes used in this section have a corner frame instead of center
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	    visual_tag_until_name = "<visual name='";
	    //visual name
	    visual_tag_until_pose = "'><pose>";
	    //visual pose
	    std::string visual_tag_until_uri = "</pose> <geometry> <mesh> <uri>";
	    //visual size as in scaling factor
	    std::string visual_tag_until_scale = "</uri><scale>";
	    std::string visual_tag_until_material = "</scale>\
	            </mesh> \
	            </geometry> \
	            <material> \
	            <lighting>1</lighting> \
	            <script> \
	            <uri>file://media/materials/scripts/gazebo.material</uri> \
	            <name>Gazebo/";
	    visual_tag_until_end= "</name> \
	            </script> \
	            <shader type='pixel'/> \
	            </material> \
	            <transparency>0</transparency> \
	            <cast_shadows>0</cast_shadows> \
	            </visual>";
	
	            // simple join
	    colission_tag_until_name = "<collision name='";
	    //colission name
	    colission_tag_until_pose = "'><pose>";
	    //collision pose
	    std::string colission_tag_until_uri = "</pose> <geometry> <mesh> <uri>";
	    //visual size as in scaling factor
	    std::string colission_tag_until_scale = "</uri><scale>";
	    colission_tag_until_end = "</scale>\
	            </mesh> \
	            </geometry> \
				<surface> \
	        <friction> \
	          <ode> \
	            <mu>0.01</mu> \
	            <mu2>0.01</mu2> \
	          </ode> \
	        </friction> \
	        </surface> \
	            </collision>";
	
	    // Maze Links
	    link_name = "Base_Tiles";
	    std::string material = "";
	    std::string resource_uri = "";
	    std::string pose_3d = "";
	    std::string line = "";
	    std::fstream fs;
		//int goal_i = -1;
		//int goal_j = -1;
	
	
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++   REDEFINE VARIABLES FOR MAZE BASE   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	

		// Read file
		// if goal block not found
		// then randomize.
		// else fixed
	
	    fs.open(maze_filename, std::fstream::in);
	
	    if (fs.good()) {
	
//	        for (int i=MAZE_SIZE-1; i >= 0; i--){ // Outer-Most Loop
	        for (int i=0; i < MAZE_SIZE; i++){ // Outer-Most Loop
	            // check filestream
	            if (!std::getline(fs, line)) {
	                gzmsg  << "getline failed" << std::endl;
	                break;
	            }
	            // maze base
				// publish base "base_colission_name" for contact plugin
				// enable collide without contact for other links except goal block
				// reduce load
	            for (int j=0; j < MAZE_SIZE; j++){
	                if (line.at(j) == '|' || line.at(j) == '_'){
	                    resource_uri = "/homes/gkumar/rl/PrivateModelDevelopment4/models/cube.dae";
	                    scale_xyz = std::to_string(scaleX) + std::to_string(scaleY) + std::to_string(floorHeight);
	                    pose_3d = std::to_string((MAZE_SIZE-2*i)*scaleX/2 - scaleX) + " "
	                            + std::to_string((MAZE_SIZE-2*j)*scaleY/2 - scaleY) + " "                                                   // Pose
	                            + std::to_string(floorThickness/2+floorHeight/2) + " 0 0 0";

	                    material = "Green";
	                tempSDF = tempSDF
	                        + colission_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(MAZE_SIZE-1-j)
	                        + colission_tag_until_pose
	                                + pose_3d
	                        + colission_tag_until_uri
	                                + resource_uri
	                        + colission_tag_until_scale
	                                + scale_xyz
	                        + colission_tag_until_end
	                        + visual_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(j)
	                        + visual_tag_until_pose
	                                + pose_3d
	                        + visual_tag_until_uri
	                                + resource_uri
	                        + visual_tag_until_scale
	                                + scale_xyz                                                             // size
	                        + visual_tag_until_material
	                                + material
	                        + visual_tag_until_end;

	                } else if (line.at(j) == 'O'){
	                    resource_uri = "/homes/gkumar/rl/PrivateModelDevelopment4/models/cubePit.dae";
	                    scale_xyz = std::to_string(scaleX) + std::to_string(scaleY) + std::to_string(floorThickness);
	                    pose_3d = std::to_string((MAZE_SIZE-2*i)*scaleX/2 - scaleX) + " "
	                            + std::to_string((MAZE_SIZE-2*j)*scaleY/2 - scaleY) + " "                                                   // Pose
	                            + std::to_string(floorThickness/2+floorHeight/2) + " 0 0 0";

	                    material = "Yellow";
	                	tempSDF = tempSDF
	                        + visual_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(j)
	                        + visual_tag_until_pose
	                                + pose_3d
	                        + visual_tag_until_uri
	                                + resource_uri
	                        + visual_tag_until_scale
	                                + scale_xyz                                                             // size
	                        + visual_tag_until_material
	                                + material
	                        + visual_tag_until_end;
	                } else if (line.at(j) == 'X'){
	                    resource_uri = "/homes/gkumar/rl/PrivateModelDevelopment4/models/cubePit.dae";
	                    scale_xyz = std::to_string(scaleX) + std::to_string(scaleY) + std::to_string(floorThickness);
	                    pose_3d = std::to_string((MAZE_SIZE-2*i)*scaleX/2 - scaleX) + " "
	                            + std::to_string((MAZE_SIZE-2*j)*scaleY/2 - scaleY) + " "                                                   // Pose
	                            + std::to_string(floorThickness/2+floorHeight/2) + " 0 0 0";

		                    material = "Blue";
							goal_i = i;
							goal_j = j;

			                tempSDF = tempSDF
	                        + visual_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(j)
	                        + visual_tag_until_pose
	                                + pose_3d
	                        + visual_tag_until_uri
	                                + resource_uri
	                        + visual_tag_until_scale
	                                + scale_xyz                                                             // size
	                        + visual_tag_until_material
	                                + material
	                        + visual_tag_until_end;
	                } else if (line.at(j) == 'S' || line.at(j) == ' ' || line.at(j) == 'X'){
	                    resource_uri = "/homes/gkumar/rl/PrivateModelDevelopment4/models/cube.dae";
	                    scale_xyz = std::to_string(scaleX) + std::to_string(scaleY) + std::to_string(floorThickness);
	                    pose_3d = std::to_string((MAZE_SIZE-2*i)*scaleX/2 - scaleX) + " "
	                            + std::to_string((MAZE_SIZE-2*j)*scaleY/2 - scaleY) + " "                                                   // Pose
	                            + std::to_string(floorThickness/2+floorHeight/2) + " 0 0 0";
	                    material = "White";
						if ( line.at(j) == ' ' ) {
							std::list<int> temp;
							temp.push_back(i);
							temp.push_back(j);
							blanks.push_back(temp);					
						}
		                tempSDF = tempSDF
	                        + colission_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(MAZE_SIZE-1-j)
	                        + colission_tag_until_pose
	                                + pose_3d
	                        + colission_tag_until_uri
	                                + resource_uri
	                        + colission_tag_until_scale
	                                + scale_xyz
	                        + colission_tag_until_end
	                        + visual_tag_until_name
	                                + link_name + std::to_string(i) + "_" + std::to_string(j)
	                        + visual_tag_until_pose
	                                + pose_3d
	                        + visual_tag_until_uri
	                                + resource_uri
	                        + visual_tag_until_scale
	                                + scale_xyz                                                             // size
	                        + visual_tag_until_material
	                                + material
	                        + visual_tag_until_end;
	                }
	            }
	        }
	    
	
		    return link_tag_Outer
    	        +"LinkMainBasement"
    	        + link_tag_Outer1
    	        + pose
    	        + link_tag_Outer2
    	        + tempSDF
    	        + link_tag_Outer10
    	        + "<joint name='JointBasementSphere' type='fixed'><parent>LinkBaseBoard</parent><child>LinkMainBasement</child></joint>";


		} else {
		    return link_tag_Outer
    	        +"LinkMainBasement"
    	        + link_tag_Outer1
    	        + pose
    	        + link_tag_Outer2
    	        + tempSDF
    	        + link_tag_Outer10
    	        + "<joint name='JointBasementSphere' type='fixed'><parent>LinkBaseBoard</parent><child>LinkMainBasement</child></joint>";
		}

	}



	 

	void LmazePlugin::rnd(){


			std::string name = "LMAZE::LinkMainBasement::Base_Tiles"+std::to_string(goal_i)+"_"+std::to_string(goal_j);
			msgs::Visual visualMsg;
  			visualMsg.set_name(name);
  			visualMsg.set_parent_name(World->GetName());
    		visualMsg.set_transparency(0);

			// Initiate material
        	if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL) {
            	gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
            	visualMsg.set_allocated_material(materialMsg);
        	}


        	// Set color
        	gazebo::common::Color newColor(1, 1, 1, 0);
        	gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
        	gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);

	        gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
     	   if (materialMsg->has_ambient())
     	   {
     	     materialMsg->clear_ambient();
     	   }
     	   materialMsg->set_allocated_ambient(colorMsg);
     	   if (materialMsg->has_diffuse())
     	   {
     	     materialMsg->clear_diffuse();
     	   }
     	   materialMsg->set_allocated_diffuse(diffuseMsg);
  			visualPub->Publish(visualMsg);









			std::list<int> temp;
			std::random_device dev;
		    std::mt19937 rng(dev());
		    std::uniform_int_distribution<std::mt19937::result_type> dist6(0,blanks.size());
			int random = dist6(rng);

		    std::list<std::list<int>> _blanks;
			_blanks = blanks;
			//gzmsg << random << std::endl;

			for(int i = 0; i < random-1; i++) {
				_blanks.pop_front();
				//temp = _blanks.front();
				//int temp_i = temp.front();
				//temp.pop_front();
				//int temp_j = temp.front();

				//gzmsg << temp_i << " " << temp_j << std::endl; 
			}

			temp = _blanks.front();
			goal_i = temp.front();
			temp.pop_front();
			goal_j = temp.front();








		  	geometry_msgs::Vector3 goalPos;
		 	goalPos.x = goal_i;
			goalPos.y = goal_j;
			goalPos.z = 0;

			pubGoal.publish(goalPos);








			std::string name_ = "LMAZE::LinkMainBasement::Base_Tiles"+std::to_string(goal_i)+"_"+std::to_string(goal_j);
			msgs::Visual visualMsg_;
  			visualMsg_.set_name(name_);
  			visualMsg_.set_parent_name(World->GetName());
    		visualMsg_.set_transparency(0);

			// Initiate material
        	if ((!visualMsg_.has_material()) || visualMsg_.mutable_material() == NULL) {
            	gazebo::msgs::Material *materialMsg_ = new gazebo::msgs::Material;
            	visualMsg_.set_allocated_material(materialMsg_);
        	}


        	// Set color
        	gazebo::common::Color newColor_(0, 0, 1, 0);
        	gazebo::msgs::Color *colorMsg_ = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor_));
        	gazebo::msgs::Color *diffuseMsg_ = new gazebo::msgs::Color(*colorMsg_);

	        gazebo::msgs::Material *materialMsg_ = visualMsg_.mutable_material();
     	   if (materialMsg_->has_ambient())
     	   {
     	     materialMsg_->clear_ambient();
     	   }
     	   materialMsg_->set_allocated_ambient(colorMsg_);
     	   if (materialMsg_->has_diffuse())
     	   {
     	     materialMsg_->clear_diffuse();
     	   }
     	   materialMsg_->set_allocated_diffuse(diffuseMsg_);

            gzmsg  << " Random Goal " << name_ << std::endl;
  			visualPub->Publish(visualMsg_);

			//gzmsg << World->GetModel("LMAZE")->GetChildLink("LinkMainBasement")->



	}

    // ******************************************************************************************************
    // ***********************************   BUILD ON DEMAND RESET OF  WORLD  *******************************
    // ******************************************************************************************************


	void LmazePlugin::OnReset(const std_msgs::Bool::ConstPtr& msg){

		// if goal position is randomized then again randomize
		if (goalRandom) {
			rnd();
			//gzmsg << "changing color" << std::endl;
		}

		// else simple reset

	    bool resetMsg = msg->data;

  		std_msgs::Bool resetStatus;
		resetStatus.data = TRUE;

		//gzmsg << "  ############# RESET ############### Learner " << std::endl;
	    if(resetMsg) {
			// publish velocity reset for controller
			pub.publish(resetStatus);
			World->SetPaused(resetMsg);
			World->Reset();
			World->SetPaused(!resetMsg);		// init msg says True to initialize hence fed as !initMsg to say SetPaused(False)
		}
	}
	void LmazePlugin::OnPause(const std_msgs::Bool::ConstPtr& msg){
	    bool pauseMsg = msg->data; 		
		gzmsg << "  ############# INITIALIZE ########## Learner " << std::endl;
		World->SetPaused(pauseMsg);
	}
    // ******************************************************************************************************
    // **************************************************   ROS QUEUE  **************************************
    // ******************************************************************************************************

	void LmazePlugin::QueueThread()
	{
	    static const double timeout = 0.01;
	    while (this->rosNode->ok())
	    {
	        this->rosQueue.callAvailable(ros::WallDuration(timeout));
	    }
	}
	void LmazePlugin::QueueThread1()
	{
	    static const double timeout = 0.01;
	    while (this->rosNode->ok())
	    {
	        this->rosQueue1.callAvailable(ros::WallDuration(timeout));
	    }
	}
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(LmazePlugin)
}

