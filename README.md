
# Install
Here you can find a complete guide to easy start the package:
https://docs.google.com/document/d/1Wk9M4t_eeKe954cxiUu8mPfhVFPXv3B58g5WXuZCgPI/edit?usp=sharing

## Requirements:
    - SuperROS
    - Python 2.7
    - MongoDB
    - ros-kinetic-joy
    - PyQT4
    


# Use TMUX (recommended)

## Install:
```
sudo apt-get update
sudo apt-get install tmux
```

## Open a new session:

```
tmux
```

## CREATE .bash file: 

```
cd
mkdir -p ~/bin
touch ~/bin/comau_robot_control.bash
chmod +x ~/bin/comau_robot_control.bash
nano ~/bin/comau_robot_control.bash
```

Example:

```
#!/bin/bash 
tmux send-keys 'cd ros/robot_ws/' Enter
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'roslaunch rocup comau.launch' Enter

tmux split-window -h
tmux send-keys 'cd ros/robot_ws/' Enter	
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'rosrun rocup gui_robot.py _robot_name:="comau_smart_six"' Enter

tmux split-window -v
tmux send-keys 'cd ros/robot_ws/' Enter	
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'rosrun rocup comau_target_follower' Enter

tmux select-pane -L
tmux split-window -v
tmux send-keys 'cd ros/robot_ws/' Enter	
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'rosrun rocup comau_supervisor.py' Enter

tmux split-window -v
tmux send-keys 'cd ros/robot_ws/' Enter	
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'rosrun rocup comau_trajectory_motion.py' Enter

tmux select-pane -R
tmux split-window -v
tmux send-keys 'cd ros/robot_ws/' Enter	
tmux send-keys 'source devel/setup.bash' Enter
tmux send-keys 'rosrun rocup comau_direct_motion.py' Enter
```


# START: 
```
source bin/comau_robot_control.bash
```

# KILL SESSION:
```
tmux kill-session
```


#TODOs:  
- remove unused import in all the files
- remove all the robots (leave only comau)
- search all the TODO in code
- leave only one robot for example
- in search where we use "/link6" and replace with a parameters "EEF_LINK_NAME = 'link6'" tha we define in global_parametrs
- global_parametrs in a json file
- instruction list in a file (json or txt)
- direct_motion must be generalized better
- trajectory (points, time, hz) in robot_motion_state_machine 
- integrate global_error_list in the rest of the project
- simplify procedure to add a new robot (maybe with gui)

