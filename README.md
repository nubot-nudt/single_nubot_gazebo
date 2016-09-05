# Description   
This simulation system can simulate ONE robot soccer player for RoboCup Middle Size League. It can be adapted for other purposes. Note that this package is only designed for demonstration. If you want to test multi-robot cooperation strategies, please refer to another repository: [gazebo_visual](https://github.com/nubot-nudt/gazebo_visual). However, the tutorial regarding compliation and etc. is still useful.   

Please read the paper ["Weijia Yao et al., A Simulation System Based on ROS and Gazebo for RoboCup Middle Size League, 2015"](https://www.trustie.net/organizations/23/publications) for more information.   
   
- Maintainer status: maintained
- Maintainer: Weijia Yao <abcgarden@126.com>
- Author: Weijia Yao <abcgarden@126.com>
- License: Apache
- Bug / feature tracker: https://github.com/nubot-nudt/single_nubot_gazebo/issues
- Source: git https://github.com/nubot-nudt/single_nubot_gazebo (branch: master)   

# Recommended Operating Environment
1. Ubuntu 14.04; 
2. ROS Indigo or ROS Jade. (It is recommended to install ROS Jade)
3. Gazebo 5.0 or above;
4. gazebo_ros_pkgs; (please read the **NOTE** below for more information)  
5. If you decide to use coach4sim with a GUI, you should make sure you have installed Qt5. The recommended install place is /opt. 
Other versions of Ubuntu, ROS or Gazebo may also work, but we have not tested yet.

**NOTE:** 
Concerning how to install appropriate **gazebo_ros_pkgs**, please read the following according to your own situation:   
 - 1.  If you decide to use **ROS Indigo**, please read the following:   
If you choose "desktop-full" install of ROS Indigo, there is a Gazebo 2.0 included initially. In order to install Gazebo 5.0/5.1, you should first remove Gazebo 2.0 by running:   
(**The following command is dangerous; it might delete the whole ROS, so please do it carefully or you may find other ways to delete gazebo2**)   
` $ sudo apt-get remove gazebo2* `    
Then you should be able to install Gazebo 5.0 now. To install gazebo_ros_pkgs compatible with Gazebo
5.0/5.1, run this command:   
` $ sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control`   
HOWEVER,    if the above command does now work, these packages may be moved to other places. You can check out [gazebo_ros](https://github.com/ros-simulation/gazebo_ros_pkgs.git) and download and install the correct version.   
 - 2. If you decide to use **ROS Jade** with **gazebo 5.0 or 5.1**, read the following   
ROS Jade has gazebo_ros_pkgs with it; so you don't have to install gazebo_ros_pkgs again.  
However, you should do the following steps to fix some of the bugs in ROS Jade related to Gazebo:        
  -  (a) `$ sudo gedit /opt/ros/jade/lib/gazebo_ros/gazebo`    
In this file, go to line 24 and delete the last '/'. So    
`setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo/`    
is changed to     
`setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo`    
You can read this link for more [information](http://answers.ros.org/question/215796/problem-for-install-gazebo_ros_package/)   
  -  (b) Install Gazebo 5.     
   `$ sudo apt-get install gazebo5`     
If this fails, try to run the ['gazebo5_install.sh'](https://github.com/nubot-nudt/simatch/blob/master/gazebo5_install.sh)(obtained from Gazebo's official website).    
Read for more [information](http://answers.ros.org/question/217970/ros-jade-and-gazebo-50-migration-problem/)   
  -  (c) Optional: copy resource files to the new gazebo folder.    
   `$ sudo cp -r /usr/share/gazebo-5.0/* /usr/share/gazebo-5.1`      
 - 3. If you decide to use **ROS Jade** with **gazebo 7.1**, read the following,    
  -  (1) Install gazebo 7.0 by running [gazebo7_install.sh](https://github.com/nubot-nudt/simatch/blob/master/gazebo7_install.sh)(obtained from Gazebo's official website);      
  -  (2) Then run this in the terminal:   
  -  ` $ sudo apt-get install ros-jade-gazebo7-ros-pkgs`   

# Complie
1. Go to the package root directory (single_nubot_gazebo)
2. If you already have CMakeLists.txt in the "src" folder, then you can skip this step. 
   If not, run these commands:
       
    ```
    $ cd src
    $ catkin_init_workspace
    $ cd ..
    ```
3. $ ./configure   
You may encounter errors related to Git. In this case, if you did not use Git, you could just ignore these errors.   
4. $ catkin_make   

# Tutorials

## Part I. Overview
The robot movement is realized by a Gazebo model plugin which is called "NubotGazebo" generated by source files "nubot_gazebo.cc" and "nubot_gazebo.hh". Basically the essential part of the plugin is realizing basic motions: omnidirectional locomotion, ball-dribbling and ball-kicking.

Basically, this plugin subscribes to topic **"/nubotcontrol/velcmd"** for omnidirecitonal movement and subscribes to service **"/BallHandle"** and **"/Shoot"** for ball-dribbling and ball-kicking respectively. You can customize this code for your robot based on these messages and services as a convenient interface. The types and definitions of the topics and servivces are listed in the following table:
         
Topic/Service	|	Type	|	Definition |
:-------------: |:-------:|:------------|
**/nubotcontrol/velcmd**	|	nubot_common/VelCmd 	|	float32 Vx <br> float32 Vy <br>  float32 w   |
**/BallHandle**   |  nubot_common/BallHandle       |  int64 enable <br> --- <br>  int64 BallIsHolding |
**/Shoot**        |  nubot_common/Shoot            | int64 strength <br> int64 ShootPos <br>  --- <br> int64 ShootIsDone |   
For the definition of /BallHandle service, when "enable" equals to a non-zero number, a dribble request would be sent. If the robot meets the conditions to dribble the ball, the service response "BallIsHolding" is true.    
   
For the definition of /Shoot service, when "ShootPos" equals to -1, this is a ground pass. In this case, "strength" is the inital speed you would like the soccer ball to have. When "ShootPos" equals to 1, this is a lob shot. In this case, "strength" is useless since the strength is calculated by the Gazebo plugin automatically and the soccer ball would follow a parabola path to enter the goal area. If the robot successfully kicks the ball out even if it failed to goal, the service response "ShootIsDone" is true.   
   

As for ball-dribbling, there are three ways for a robot to dribble a ball, i.e.
            
Method  | Description
:-----: | -------------
(a) Setting ball pose continually  | This is the most accurate one; nubot would hardly lose control of the ball, but the visual effect is not very good (the ball does not rotate).
(b) Setting ball secant velocity  | This is less acurate than method (a) but more accurate than method (c).
(c) Setting ball tangential velocity |  This is the least accurate. If the robot moves fast, such as 3 m/s, it would probably lose control of the ball. However, this method achieves the best visual effect under low-speed condition.
**By default, we use method (c) for ball-dribbling.**
    
 As for Gaussian noise, **by default, Gaussian noise is NOT added**, but you can add it by changing the flag in the function update_model_info() in the file nubot_gazebo.cc;
 
## Part II. Single robot automatic movement
 The robot will do motions according to states transfer graph. Steps are as follows:
 1. Go to the package root directory (single_nubot_gazebo)
 2. source the setup.bash file:   
   ` $ source devel/setup.bash`
 3.  `$ roslaunch nubot_gazebo sdf_nubot.launch`   
 
>  **Note:** Every time you open a new terminal, you have to do step 2. You can also write this command into the ~/.bashrc file so that you don't have to source it every time.

Finally, the robot rotates and translates with trajectory planning. That is, the robot accelerates at constant acceleration and stays at constant speed when it reaches the maximum velocity.    
   
You could click the 'Edit->Reset World' from the menu (or press ctrl-shift-r) to reset the simulation world and the robot would do the basic motions again.    
   
When the robot finally reaches its last state: HOME, you could run   
` $ rosrun nubot_gazebo nubot_teleop_keyboard`   
to control the movement of the robot.   
 
## Part III. Keyboad control robot movement
 1. In nubot_gazebo.cc, comment "nubot_auto_control();" and uncomment "nubot_be_control();" in function UpdateChild().
 2. Compile again and follow steps 1-3 listed in Part II.
 3. ` $ rosrun nubot_gazebo nubot_teleop_keyboard`   

## Part IV. NubotGazebo API   
For the detailed infomation and usage of the NubotGazebo class, please refer to the [doc/](https://github.com/nubot-nudt/single_nubot_gazebo/tree/master/doc) folder. 
 
## Part V. How you could use it to do more stuff
The main purpose of the simulation system is to test multi-robot collaboration algorithm. So to achieve this purpose, you need to know how to control the movement of each robot in the simulation. As you have experienced in turotial Part III to control a robot by keyboard, you could read its [source code](https://github.com/nubot-nudt/single_nubot_gazebo/blob/master/src/nubot_simulation/nubot_gazebo/plugins/nubot_teleop_keyboard.cc) and make use of the topic publishing and service calling. In a word, if you want to control the movement of  the robots, publish velocity commands on the topic "/nubotcontrol/velcmd". If you want the robot to dribble the ball, after it is close enough to the ball, call the ROS service named "/BallHandle" and kick the ball by calling the service named"/Shoot" . The types and definitions of theses topics and services are presented in Part I.

## Part VI. Appendix
  1. To launch an empty soccer field:   
  ` $ roslaunch nubot_gazebo empty_field.launch`
  2. To launch the simulation world with rqt_plot of nubot or ball's velocity:  
  ` $ roslaunch nubot_gazebo sdf_nubot.launch plot:=true`
  
## Q&A
