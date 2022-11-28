#!/bin/sh


# source workspace
if [ $SHELL = '/bin/zsh' ]; then
  #  echo "--- AA274_WS_DIR sourced ($AA274_WS_DIR) ---"
    source $AA274_WS_DIR/devel/setup.zsh
elif [ $SHELL = '/bin/bash' ]; then
  #  echo "--- AA274_WS_DIR sourced ($AA274_WS_DIR) ---"
    source $AA274_WS_DIR/devel/setup.bash
else
  echo "no zsh or bash used"
fi

# workspace specific
alias makeaa274="cd $AA274_WS_DIR && catkin_make -DCMAKE_BUILD_TYPE=Release ; cd -"
alias cleanaa274="cd $AA274_WS_DIR && catkin_make clean ; cd -"



