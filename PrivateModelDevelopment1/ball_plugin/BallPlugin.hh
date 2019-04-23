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
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{


class BallPlugin : public WorldPlugin {
    
public :
    int MAZE_SIZE = 0;

    float scaleX = 0.05;
    float scaleY = 0.05;
    double cradius = 0.006;
    float floorThickness = 0.001; 	// should be equal to z axis scaling of cube for floor Model; currently "0.0155"
    float floorHeight = 0.025; 	// should be equal to z axis scaling of cube for floor Model; currently "0.0155"

    std::string maze_filename = "/homes/gkumar/rl/PrivateModelDevelopment1/sample_labyrinth_maze.mz";
    
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    physics::WorldPtr World;
    physics::ModelPtr Model;

};
}
