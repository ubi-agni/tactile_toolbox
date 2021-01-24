#!/bin/bash
set -e  # fail on any error

export TRAVIS_FOLD_COUNTER=1

# Display command in Travis console and fold output in dropdown section
function travis_run() {
  local command=$@

  echo -e "\e[0Ktravis_fold:start:command$TRAVIS_FOLD_COUNTER \e[34m$ $command\e[0m"
  # actually run command
  $command || exit 1 # kill build if error
  echo -e -n "\e[0Ktravis_fold:end:command$TRAVIS_FOLD_COUNTER\e[0m"

  let "TRAVIS_FOLD_COUNTER += 1"
}

#######################################
# Same as travis_run except ignores errors and does not break build
function travis_run_true() {
  local command=$@

  echo -e "\e[0Ktravis_fold:start:command$TRAVIS_FOLD_COUNTER \e[34m$ $command\e[0m"
  # actually run command
  $command # ignore errors
  echo -e -n "\e[0Ktravis_fold:end:command$TRAVIS_FOLD_COUNTER\e[0m"

  let "TRAVIS_FOLD_COUNTER += 1"
}

REPO="${BASH_SOURCE[0]%/*}"

### setup catkin workspace
mkdir -p /tmp/catkin/src
cd /tmp/catkin/src

travis_run wstool init
travis_run wstool merge file://$REPO/rosinstall
travis_run wstool update

# link in source
ln -s $REPO .

### fetch system packages
travis_run apt-get -qq update
if [ "$ROS_PYTHON_VERSION" == "3" ] ; then
  travis_run apt-get -qq install -y sudo python3-rosdep python3-catkin-tools
else
  travis_run apt-get -qq install -y sudo python-rosdep python-catkin-tools
fi

travis_run rosdep update
travis_run rosdep install -r -y -q -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO

### build with catkin
cd ..
export PYTHONIOENCODING=UTF-8
export TERM=xterm

travis_run catkin config --extend /opt/ros/$ROS_DISTRO --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Wno-deprecated-declarations -Dtactile_filters_DIR=$(pwd)/install/share --
travis_run catkin build --no-status --continue-on-failure --summarize

source install/setup.bash

travis_run catkin config --blacklist urdfdom_headers urdfdom tactile_filters
travis_run catkin build --no-status --continue-on-failure --summarize --make-args tests --
travis_run catkin run_tests --no-status --continue-on-failure --summarize
catkin_test_results
result=$?

echo "Travis script has finished successfully"
HIT_ENDOFSCRIPT=true
exit $result
