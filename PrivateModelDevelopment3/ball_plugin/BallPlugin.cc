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
#include "BallPlugin.hh"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <random>

using namespace std;

namespace gazebo
{

void BallPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){

    starting_tag = "<sdf version='1.6'> \
            <model name='BALL'> \
            <frame name='WorldBase'>0 0 0 0 0 0</frame> \
            <pose>";

    link_tag_ball_until_end = "</pose> \
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
					<plugin name='my_plugin' filename='/homes/gkumar/rl/PrivateModelDevelopment2/build/libcontact.so'> \
		        	    <update_rate>0.0</update_rate> \
					</plugin> \
        	   </sensor> \
			</link>";



/*



            




			

*/
    ending_tag = "<plugin name='ball_controller_plugin' filename='/homes/gkumar/rl/PrivateModelDevelopment2/build/libball_controller_plugin.so'> \
		        	    <update_rate>30.0</update_rate> \
					</plugin> \
					<static>0</static> \
            <allow_auto_disable>1</allow_auto_disable> \
            </model> \
            </sdf>";


    gzmsg << "Load Ball Model" << std::endl;

    World = _parent;
    std::fstream fs;
    pos_j = -1;
    pos_i = -1;
	found = false;

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
					break;
                } else if(line.at(j) == ' ') {
					std::list<int> temp;
					temp.push_back(i);
					temp.push_back(j);
					blanks.push_back(temp);					
				}
            }
            if(!fs.eof()) std::getline(fs, line);
			if(found) break;
        }

		if(found) {
	        sdf::SDF unitSDF;
			gzmsg << "Starting position of ball found to be ========= " << pos_i << " " << pos_j << std::endl;
	        unitSDF.SetFromString(starting_tag 
								+ std::to_string((pos_i-(MAZE_SIZE - 1)/2)*scaleX) + " "
                              	+ std::to_string((pos_j-(MAZE_SIZE - 1)/2)*scaleY) + " "
                              	+ std::to_string(MAZE_SIZE*scaleX+2*cradius+2*floorThickness+floorHeight+0.005+0.005)
                              	+ " 0 0 0" 
								+ link_tag_ball_until_end + ending_tag);
	        World->InsertModelSDF(unitSDF);
		} else if (!found) {
			std::list<int> temp;
			std::random_device dev;
		    std::mt19937 rng(dev());
		    std::uniform_int_distribution<std::mt19937::result_type> dist6(1,blanks.size());
			int random = dist6(rng);

		    std::list<std::list<int>> _blanks;
			_blanks = blanks;

			for(int i = 1; i < random; i++) {
				_blanks.pop_front();
			}

			temp = _blanks.front();
			pos_i = temp.front();
			temp.pop_front();
			pos_j = temp.front();
            gzmsg  << "Starting position of ball not found !!!!!!!!" << random << " " << pos_i << " " << pos_j << std::endl;

	        sdf::SDF unitSDF;
	        unitSDF.SetFromString(starting_tag 
								+ std::to_string((pos_i-(MAZE_SIZE - 1)/2)*scaleX) + " "
                              	+ std::to_string((pos_j-(MAZE_SIZE - 1)/2)*scaleY) + " "
                              	+ std::to_string(MAZE_SIZE*scaleX+2*cradius+2*floorThickness+floorHeight+0.005+0.005)
                              	+ " 0 0 0" 
								+ link_tag_ball_until_end + ending_tag);
	        World->InsertModelSDF(unitSDF);

		    updateConnectionOn = event::Events::ConnectWorldReset(boost::bind(&BallPlugin::OnWorldReset, this));
		}
    } else {gzmsg << "filestream error";}
	fs.close();
}

void BallPlugin::OnWorldReset(){

	std::list<int> temp;
    std::list<std::list<int>> _blanks;
	_blanks = blanks;

	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_int_distribution<std::mt19937::result_type> dist6(1,_blanks.size());
			
	int random = dist6(rng);

	for(int i = 1; i < random; i++) {
		_blanks.pop_front();
	}

	temp = _blanks.front();
	pos_i = temp.front();
	temp.pop_front();
	pos_j = temp.front();

    gzmsg  << "!!!! " << random << " " << pos_i << " " << pos_j << std::endl;

	Model = World->GetModel("BALL");
	Model->SetWorldPose( math::Pose( 	(pos_i-(MAZE_SIZE - 1)/2)*scaleX, 
										(pos_j-(MAZE_SIZE - 1)/2)*scaleY, 
										MAZE_SIZE*scaleX+2*cradius+2*floorThickness+floorHeight+0.005+0.005,
										0, 0, 0 ) );
}



// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(BallPlugin)
}
