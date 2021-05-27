##### Setup
1. Go to your carla directory, make sure that you have exported the UE4 root, using the command `export UE4_ROOT=~/UnrealEngine_4.24`, to wherever you keep the Unreal Engine directory. You'll want to add this variable to your `~/.bashrc` or `~/.profile` to avoid repeating this step outside of the current shell session.

2. While still in your carla directory, `make launch && make PythonAPI` if it's a first time setup otherwise `make launch-only`. Once the map editor opens (and any map changes are made) click on the play button and wait for the map to load completely. 

##### Segmentation/segmentation_controller.py
 - To run the segmentation controller, make sure Carla/Unreal-Engine is running, and run `python3 Segmentation/segmentation_controller.py`