#!/bin/bash

# Get parameters
declare -i k=$(rosparam get /nubot/number)
declare -i length=$(rosparam get /field/length)
declare -i width=$(rosparam get /field/width)
football_name=$(rosparam get /football/name)
robot_prefix=$(rosparam get /field/robot_prefix)
echo "field length: $length"
echo "field width: $width"

# Generate random number. input:RANGE output:[-RANGE/2,RANGE/2]
function rand
{
    RANGE=$1+1
    pos=$RANDOM
    let "pos = pos % $RANGE - $1 / 2"
    echo $pos
}

# spawn football
rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/football/model.sdf -sdf \
                              -model ${football_name} \
                              -x -8.0 -y 5.0 -z 0.0 \
                               /

rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/nubot/model.sdf -sdf \
                              -model ${robot_prefix}1 \
	                          -x -8.42 -y 3.0 -z 0.0 \                      
# spawn k nubots
for ((i=2; i<=k; ++i))
do
	rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/nubot/model.sdf -sdf \
        -model ${robot_prefix}${i} \
	-x $(rand $length) -y $(rand $width) -z 0.0 \
	/
done

# run coachinfo_publisher
# rosrun coachinfo_publisher coachinfo_publisher_node 

# rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/football/model.sdf -sdf -model football -x 2 -y 0 -z 0
# rosrun gazebo_ros spawn_model -file $(rospack find nubot_description)/models/nubot/model.sdf -sdf -model nubot0 -x 3 -y 1 -z 0
