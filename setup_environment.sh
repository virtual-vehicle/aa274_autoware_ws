#!/bin/sh

write_bash()
{

if [ $SHELL = '/bin/zsh' ]; then
  echo "$1" >> ~/.zshrc
elif [ $SHELL = '/bin/bash' ]; then
   echo "$1" >> ~/.bashrc
else
  echo "no zsh or bash used"
fi

}


echo "Setting up the AA274 workspace!"

AA274_AUTOWARE_WS_DIR="$HOME/ros/aa274_autoware_ws"
AUTOWARE_WS_DIR="$HOME/ros/autoware.ai"

# write to bash configuration
write_bash "# automatically added by '$(readlink -f "$0")'"
write_bash "export AA274_AUTOWARE_WS_DIR=\"$AA274_AUTOWARE_WS_DIR\""
write_bash "export AUTOWARE_WS_DIR=\"$AUTOWARE_WS_DIR\""
write_bash "source \$AA274_AUTOWARE_WS_DIR/config_ws/environment.sh \$AA274_AUTOWARE_WS_DIR \$AUTOWARE_WS_DIR" 
write_bash "source \$AA274_AUTOWARE_WS_DIR/config_ws/extentions.sh \$AA274_AUTOWARE_WS_DIR \$AUTOWARE_WS_DIR" 
write_bash "# --"


echo "- added environment variable to bashrc/zshrc"
echo "- added source of environment.sh to bashrc/zshrc"
echo "- added source of extentions.sh to bashrc/zshrc"
echo "Finished"
