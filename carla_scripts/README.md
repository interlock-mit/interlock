#### **Certificate, Spawning, and Visualizer** <br>
##### Setup
1. Go to your carla directory, make sure that you have exported the UE4 root, using the command `export UE4_ROOT=~/UnreaEngine_4.24`, to wherever you keep the Unreal Engine directory. You'll want to add this variable to your `~/.bashrc` or `~/.profile` to avoid repeating this step outside of the current shell session.

2. While still in your carla directory, `make launch && make PythonAPI` if it's a first time setup otherwise `make launch-only`. Once the map editor opens (and any map changes are made) click on the play button and wait for the map to load completely. 

- To get the carla visualizer running (carla must be running already), run `sudo docker run -it --network="host" -e CARLAVIZ_HOST_IP=localhost -e CARLA_SERVER_IP=localhost -e CARLA_SERVER_PORT=2000 mjxu96/carlaviz:0.9.9`, assuming the docker image you have is `mjxu96/carlaviz:0.9.9` if not replace it with the correct docker image name. Open your web browser and go to http://127.0.0.1:8080/. carlaviz runs by default in port 8080.

- To run the script responsible for the certificate, go to the interlock directory and run `python3 -m carla_scripts.lidarcamera`. Note that the visualizer needs to be running for the script to function properly.

- To spawn various obstacles/objects, from the interlock directory go to the carla_scripts directory and run `python3 spawn_actor.py`

##### Spawn_Actor.py
- To repeatedly spawn/test objects quicker, save them as a preset; you can view the presets in the `saves.json` file.

- If you want to quickly spawn multiple objects (i.e. snow simulated by small trash), use the command line interface to save each object as its own preset, then in `saves.json` put all related objects into a nested json with a name starting with 'pack'. Reference `saves.json` and the 'pack_inclined_pole' as an example. 

- If you want to switch an object's physics off/on, search for `.set_simulate_physics()`. The default is set to False so objects are not effected by gravity and fall, though they will still be visible to LiDAR.

- Potential Error -> _Inconsistent RuntimeError because of collision at spawn position_: <br> - Trying to spawn objects too close to each other? Can easily happen without noticing as some actors are larger than expected (e.g. cars/trucks spawn in each other).
<br> - Used all points in carla's default spawnpoints list? Though we typically use custom spawnpoints so this is less likely. <br> - **_FIX_**: Need to destroy all actors/blueprints using the destroy methods or restarting UE. This gets rid of any 'invisible' actors that may have been spawned and never destroyed due to abruptions. Also may need to switch physics on some actors to avoid collisions due to falling, though this is less likely.  
