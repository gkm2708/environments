source /usr/share/gazebo/setup.sh

export GAZEBO_PLUGIN_PATH=${PWD}/build:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=${PWD}/build:$LD_LIBRARY_PATH
export GAZEBO_MODEL_PATH=${PWD}/models:/home/gaurav/model_editor_models:$GAZEBO_MODEL_PATH

gazebo -u --verbose lmaze_plugin.world
