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

    std::string maze_filename = "/homes/gkumar/rl/PrivateModelDevelopment4/sample_labyrinth_maze.mz";
    
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

	event::ConnectionPtr updateConnectionOn;

    physics::WorldPtr World;
    physics::ModelPtr Model;

    std::string starting_tag;
    std::string link_tag_ball_until_end;
    std::string ending_tag;
    std::list<std::list<int>> blanks;
    int pos_j, pos_i;
	bool found;
	void OnWorldReset();

};
}
