#!/bin/sh

# tools
alias gzsh="gedit ~/.zshrc"
alias szsh="source ~/.zshrc"

alias gbash="gedit ~/.bashrc"
alias sbash="source ~/.bashrc"

# ros
alias tf="cd /var/tmp && rosrun tf view_frames && evince frames.pdf & ; cd -"


# workspace specific
alias makeaa274="cd $AA274_AUTOWARE_WS_DIR && catkin build -DCMAKE_BUILD_TYPE=Release ; cd -"
alias cleanaa274="cd $AA274_AUTOWARE_WS_DIR && catkin build clean ; cd -"
alias updateaa274="cd $AA274_AUTOWARE_WS_DIR/src && rosdep install -iy --from-paths src --ignore-src ; cd -"

# startup
alias vifware_online="roslaunch vifware_launch online.launch"
alias vifware_offline="roslaunch vifware_launch offline.launch"
alias vifware_simulation="roslaunch vifware_launch simulation.launch"
