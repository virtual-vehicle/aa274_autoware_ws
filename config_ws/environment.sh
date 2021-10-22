#!/bin/sh

# workspace directory
AA274_AUTOWARE_WS_DIR=${1}
AUTOWARE_WS_DIR=${2}

# source workspace
if [ $SHELL = '/bin/zsh' ]; then
    echo "--- AUTOWARE.AI sourced ($AUTOWARE_WS_DIR) ---"
    source $AUTOWARE_WS_DIR/devel/setup.zsh
    echo "--- AA274_AUTOWARE_WS sourced ($AA274_AUTOWARE_WS_DIR) ---"
    source $AA274_AUTOWARE_WS_DIR/devel/setup.zsh
elif [ $SHELL = '/bin/bash' ]; then
    echo "--- AUTOWARE.AI sourced ($AUTOWARE_WS_DIR) ---"
    source $AUTOWARE_WS_DIR/devel/setup.bash
    echo "--- AA274_AUTOWARE_WS sourced ($AA274_AUTOWARE_WS_DIR) ---"
    source $AA274_AUTOWARE_WS_DIR/devel/setup.bash
else
  echo "no zsh or bash used"
fi

