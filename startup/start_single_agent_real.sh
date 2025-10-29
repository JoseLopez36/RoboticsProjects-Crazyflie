#!/bin/bash

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln single_agent_real_session.yml .tmuxinator.yml

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=single_agent_setup.sh

# start tmuxinator
tmuxinator single_agent_real_session.yml setup_name=$SETUP_NAME