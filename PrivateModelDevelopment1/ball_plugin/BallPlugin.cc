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


/*				   <pose>0 0 0 0 0 0</pose> \
                   <frame name = 'Ball_Center'>0 0 0 0 0 0</frame>\
*/



#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "BallPlugin.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{

void BallPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){

    gzmsg << "Load Ball Model" << std::endl;

    World = _parent;

    std::string starting_tag = "<sdf version='1.6'> \
            <model name='BALL'> \
            <frame name='WorldBase'>0 0 0 0 0 0</frame> \
            <pose>";

            /*frame=''*/
	/*
			<inertial> \
            <mass>7.85</mass> \
			</inertial> \
*/

            std::string link_tag_ball_until_end = "</pose> \
            <link name ='ball_link'> \
            <collision name='collision'> \
            <geometry> \
            <sphere><radius>0.015</radius></sphere> \
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
            <sphere><radius>0.015</radius></sphere> \
            </geometry> \
            <material> \
            <lighting>1</lighting> \
            <script> \
            <uri>file://media/materials/scripts/gazebo.material</uri> \
            <name>Gazebo/Red</name> \
            </script> \
            <shader type='pixel'/> \
            </material> \
            <transparency>0</transparency> \
            <cast_shadows>0</cast_shadows> \
            </visual> \
			<sensor name='my_contact' type='contact'> \
        	    <update_rate>30.0</update_rate> \
			  		<contact> \
				    	<collision>collision</collision> \
			        </contact> \
					<plugin name='my_plugin' filename='/homes/gkumar/rl/PrivateModelDevelopment1/build/libcontact.so'> \
		        	    <update_rate>0.0</update_rate> \
					</plugin> \
        	   </sensor> \
            </link>";

            std::string ending_tag = "<plugin name='ball_controller_plugin' filename='/homes/gkumar/rl/PrivateModelDevelopment1/build/libball_controller_plugin.so'> \
		        	    <update_rate>30.0</update_rate> \
					</plugin> \
			<static>0</static> \
            <allow_auto_disable>1</allow_auto_disable> \
            </model> \
            </sdf>";

            gzmsg << " Inside BallPlugin \t\t Load Method" << std::endl;

    std::fstream fs;

    int pos_j = 0;
    int pos_i = 0;
	bool found = false;
    fs.open(maze_filename, std::fstream::in);

    if (fs.good()) {

        std::string line;
        std::string tempSDF = "";

        std::getline(fs, line);
        MAZE_SIZE = line.size();

        for (int i=0; i < MAZE_SIZE; i++){

            if (!fs) {
                gzmsg  << "getline failed" << std::endl;
                return;
            }

            for (int j=0; j < MAZE_SIZE; j++){
                if (line.at(j) == 'S')	{
					pos_i = i;
					pos_j = j;
					found = true;
					gzmsg << i << j<< std::endl;
                }
            }
            if(!fs.eof()) {
                std::getline(fs, line);
            }
            if(found) {
				break;
            }

        }

        sdf::SDF unitSDF;
		gzmsg << pos_i << " " << pos_j << std::endl;
        unitSDF.SetFromString(starting_tag 
								+ std::to_string((pos_i-(MAZE_SIZE - 1)/2)*scaleX) + " "
                              	+ std::to_string((pos_j-(MAZE_SIZE - 1)/2)*scaleY) + " "
                              	+ std::to_string(MAZE_SIZE*scaleX+2*cradius+2*floorThickness+floorHeight+0.005+0.005)
                              	+ " 0 0 0" 
								+ link_tag_ball_until_end + ending_tag);
        World->InsertModelSDF(unitSDF);
		gzmsg << std::to_string((pos_i-0.5)*scaleX) + " "
                              	+ std::to_string((pos_j+0.5)*scaleY) + " "
                              	+ std::to_string(MAZE_SIZE*scaleX+2*cradius+2*floorThickness+floorHeight+0.005+0.005)
                              	+ " 0 0 0 " + std::to_string(pos_i) + " " + std::to_string(pos_j) <<std::endl;

	
    } else {gzmsg << "filestream error";}
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(BallPlugin)
}
