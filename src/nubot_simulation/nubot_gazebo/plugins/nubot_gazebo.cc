/*
 * Copyright (C) 2015 NuBot team of National University of Defense Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: Gazebo model plugin for NuBot basic motions realization.
 * Author: Weijia Yao
 * Date: Jun 2015
 */

// NOTICE: GAZEBO uses ISO units, e.g.  meters for length.

#include "nubot_gazebo.hh"
#include "vector_angle.hh"

#define RUN -1
#define FLY 1
#define ZERO_VECTOR math::Vector3::Zero
#define PI 3.14159265

const math::Vector3 kick_vector_nubot(1,0,0);    // Normalized vector from origin to kicking mechanism in nubot refercence frame.
                                                 // It is subject to nubot model file
const double        goal_x = 9.0;
const double        goal_height = 1.0;
const math::Vector3 right_goal(goal_x,0.0,0.0);
const double        g = 9.8;
const double        m = 0.41;                   // ball mass (kg)

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(NubotGazebo)

NubotGazebo::NubotGazebo()
{}

NubotGazebo::~NubotGazebo()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish. 
  this->message_queue_.clear();
  this->service_queue_.clear();
  // Disable the queue, meaning any calls to addCallback() will have no effect. 
  this->message_queue_.disable();
  this->service_queue_.disable();
  this->message_callback_queue_thread_.join();
  this->service_callback_queue_thread_.join();
  this->rosnode_->shutdown();
  delete this->rosnode_;
}


void NubotGazebo::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{   
  // Get the world name.
  this->world_ = _model->GetWorld();
  this->nubot_model_ = _model;
  model_name_ = nubot_model_->GetName();
  this->robot_namespace_ = nubot_model_->GetName();

  ROS_INFO("%s has %d plugins",model_name_.c_str(),nubot_model_->GetPluginCount());

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libnubot_gazebo.so' in the gazebo_ros package)");
    return;
  }
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  
  // load parameters
  rosnode_->param("/football/name",          football_name_,            std::string("football"));
  rosnode_->param("/football/chassis_link",  football_chassis_,         std::string("football::ball"));
  rosnode_->param("/nubot/max_linear_vel",   max_linear_vel_,           3.0);
  rosnode_->param("/nubot/max_angular_vel",  max_angular_vel_,          120.0);
  rosnode_->param("/nubot/distance_thres",   dribble_distance_thres_,   0.5);
  rosnode_->param("/nubot/angle_thres",      dribble_angle_thres_,      30.0);
  rosnode_->param("/nubot/kick_ball_vel",    kick_ball_vel_,            20.0);
  rosnode_->param("/field/robot_prefix",     robot_prefix_,             std::string("bot"));

  // Load the football model 
  this->football_model_ = world_->GetModel(football_name_);

  if (!football_model_)
    ROS_ERROR("model [%s] does not exist", football_name_.c_str());
  else 
  {
    football_link_ = football_model_->GetLink(football_chassis_);
    if(!football_link_)
    {
      ROS_ERROR("link [%s] does not exist!", football_chassis_.c_str());
    }
  }    
  // Subscribers.
  ros::SubscribeOptions so1 = ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
    "/gazebo/model_states", 100, boost::bind( &NubotGazebo::model_states_CB,this,_1),
    ros::VoidPtr(), &this->message_queue_);
  this->ModelStates_sub_ = this->rosnode_->subscribe(so1);

  ros::SubscribeOptions so2 = ros::SubscribeOptions::create<nubot_common::VelCmd>(
    "nubotcontrol/velcmd", 100, boost::bind( &NubotGazebo::vel_cmd_CB,this,_1),
    ros::VoidPtr(), &this->message_queue_);
  this->Velcmd_sub_ = this->rosnode_->subscribe(so2);

  // Service Servers
  ros::AdvertiseServiceOptions aso1 = ros::AdvertiseServiceOptions::create<nubot_common::BallHandle>(
              "BallHandle", boost::bind(&NubotGazebo::ball_handle_control_service, this, _1, _2),
              ros::VoidPtr(), &this->service_queue_);
  ballhandle_server_ =   this->rosnode_->advertiseService(aso1);

  ros::AdvertiseServiceOptions aso2 = ros::AdvertiseServiceOptions::create<nubot_common::Shoot>(
              "Shoot", boost::bind(&NubotGazebo::shoot_control_servive, this, _1, _2),
              ros::VoidPtr(), &this->service_queue_);
  shoot_server_ =   this->rosnode_->advertiseService(aso2);

  reconfigureServer_ = new dynamic_reconfigure::Server<nubot_gazebo::NubotGazeboConfig>(*this->rosnode_);
  reconfigureServer_->setCallback(boost::bind(&NubotGazebo::config, this, _1, _2));

  // Custom Callback Queue Thread. Use threads to process message and service callback queue
  this->message_callback_queue_thread_ = boost::thread( boost::bind( &NubotGazebo::message_queue_thread,this ) );
  this->service_callback_queue_thread_ = boost::thread( boost::bind( &NubotGazebo::service_queue_thread,this ) );

  // This event is broadcast every simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&NubotGazebo::UpdateChild, this));
}

void NubotGazebo::Init()
{
    ROS_DEBUG("%s Init(): Initialization begins!", model_name_.c_str());
    
    last_current_time_ = 0;
    is_hold_ball_ = false;
    dribble_flag_ = false;
    shot_flag_ = false;
    count_ = 0;                     // used as a timer
    ModelStatesCB_flag_ = false;
    state_ = CHASE_BALL;
    sub_state_ = MOVE_BALL;
    judge_nubot_stuck_ = false;
    desired_trans_vector_ = ZERO_VECTOR;
    desired_rot_vector_ = ZERO_VECTOR;
    traj_plan_linear_.Reset();
    traj_plan_rot_.Reset();
}

void NubotGazebo::Reset()
{
    ROS_DEBUG("%s Reset() running now!", model_name_.c_str());

    last_current_time_ = 0;
    is_hold_ball_ = false;
    dribble_flag_ = false;
    shot_flag_ = false;
    count_ = 0;                  // used as a timer
    ModelStatesCB_flag_ = false;
    state_ = CHASE_BALL;
    sub_state_ = MOVE_BALL;
    judge_nubot_stuck_ = false;
    desired_trans_vector_ = ZERO_VECTOR;
    desired_rot_vector_ = ZERO_VECTOR;
    traj_plan_linear_.Reset();
    traj_plan_rot_.Reset();
}

void NubotGazebo::UpdateChild()
{
    this->msgCB_lock_.lock(); // lock access to fields that are used in ROS message callbacks
    this->srvCB_lock_.lock();

    if(update_model_info())         // delay in model_states messages publishing
    {                               // so after receiving model_states message, then nubot moves.
       /********** EDIT BEGINS **********/

        nubot_be_control();
        // nubot_auto_control();

       /**********  EDIT ENDS  **********/
    }
    if(ball_decay_flag_)
    {
        math::Vector3 free_ball_vel = football_state_.twist.linear;
        ball_vel_decay(free_ball_vel, 0.3);
    }
    ball_decay_flag_ = true;

    this->srvCB_lock_.unlock();
    this->msgCB_lock_.unlock();
}

void NubotGazebo::message_queue_thread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    // Invoke all callbacks currently in the queue. If a callback was not ready to be called,
    // pushes it back onto the queue. This version includes a timeout which lets you specify
    // the amount of time to wait for a callback to be available before returning.
    this->message_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void NubotGazebo::service_queue_thread()
{
    static const double timeout = 0.01;
    while (this->rosnode_->ok())
      this->service_queue_.callAvailable(ros::WallDuration(timeout));
}

// Actually PID tuning is not useful in this simulation since realistic physics mechanisms are not simulated
// PID is not used in this code but it is commented in function dribble_ball();
void NubotGazebo::config(nubot_gazebo::NubotGazeboConfig &config, uint32_t level)
{
    dribble_P_      = config.P;
    dribble_I_      = config.I;
    dribble_D_      = config.D;
    I_term_max_     = config.I_max;
    I_term_min_     = config.I_min;
    cmd_max_        = config.cmd_max;
    cmd_min_        = config.cmd_min;
    ROS_INFO("Reconfig request: P:%f I:%f D:%f I_term_max:%f I_term_min:%f",
              dribble_P_, dribble_I_, dribble_D_, I_term_max_, I_term_min_);
    dribble_pid_.Init(dribble_P_, dribble_I_, dribble_D_, I_term_max_, I_term_min_, cmd_max_, cmd_min_);
}

void NubotGazebo::model_states_CB(const gazebo_msgs::ModelStates::ConstPtr& _msg)
{
  this->msgCB_lock_.lock();

  ModelStatesCB_flag_ = true;
  model_count_ = world_->GetModelCount();
  // Resize message fields
  this->model_states_msg_.name.resize(model_count_);
  this->model_states_msg_.pose.resize(model_count_);
  this->model_states_msg_.twist.resize(model_count_);

  for(int i=0; i<model_count_;i++)
  {
    this->model_states_msg_.name[i] = _msg->name[i];
    this->model_states_msg_.pose[i] = _msg->pose[i];   // Reference fram: world
    this->model_states_msg_.twist[i] = _msg->twist[i];     
    if(this->model_states_msg_.name[i] == football_name_ ) 
        football_index_ =  i;             
    else if(this->model_states_msg_.name[i] == model_name_)
        nubot_index_ = i;                     
  }

   this->msgCB_lock_.unlock();
}

bool NubotGazebo::update_model_info(void)
{  
    rosnode_->param("/nubot/distance_thres",    dribble_distance_thres_,    0.50);
    rosnode_->param("/nubot/angle_thres",      dribble_angle_thres_,      30.0);

   if(ModelStatesCB_flag_)
   {
        receive_sim_time_ = world_->GetSimTime();
      // Get football and nubot's pose and twist
#if 1 // no Gaussian noise
        football_state_.model_name = football_name_ ;
        football_state_.pose.position.x     =  model_states_msg_.pose[football_index_].position.x;
        football_state_.pose.position.y     =  model_states_msg_.pose[football_index_].position.y;
        football_state_.pose.position.z     =  model_states_msg_.pose[football_index_].position.z;
        football_state_.twist.linear.x      =  model_states_msg_.twist[football_index_].linear.x;
        football_state_.twist.linear.y      =  model_states_msg_.twist[football_index_].linear.y;
        football_state_.twist.linear.z      =  model_states_msg_.twist[football_index_].linear.z;

        nubot_state_.model_name = model_name_ ;
        nubot_state_.pose.position.x    =  model_states_msg_.pose[nubot_index_].position.x;
        nubot_state_.pose.position.y    =  model_states_msg_.pose[nubot_index_].position.y;
        nubot_state_.pose.position.z    =  model_states_msg_.pose[nubot_index_].position.z;
        nubot_state_.pose.orientation.w =  model_states_msg_.pose[nubot_index_].orientation.w;
        nubot_state_.pose.orientation.x =  model_states_msg_.pose[nubot_index_].orientation.x;
        nubot_state_.pose.orientation.y =  model_states_msg_.pose[nubot_index_].orientation.y;
        nubot_state_.pose.orientation.z =  model_states_msg_.pose[nubot_index_].orientation.z;
        nubot_state_.twist.linear.x     =  model_states_msg_.twist[nubot_index_].linear.x;
        nubot_state_.twist.linear.y     =  model_states_msg_.twist[nubot_index_].linear.y;
        nubot_state_.twist.linear.z     =  model_states_msg_.twist[nubot_index_].linear.z;
        nubot_state_.twist.angular.x    =  model_states_msg_.twist[nubot_index_].angular.x;
        nubot_state_.twist.angular.y    =  model_states_msg_.twist[nubot_index_].angular.y;
        nubot_state_.twist.angular.z    =  model_states_msg_.twist[nubot_index_].angular.z;
#endif
#if 0 // add gaussian noise
        static double scalar = 0.0194;
        football_state_.model_name = football_name_ ;
        football_state_.pose.position.x     =  model_states_msg_.pose[football_index_].position.x + scalar*rand_.GetDblNormal(0,1);
        football_state_.pose.position.y     =  model_states_msg_.pose[football_index_].position.y + scalar*rand_.GetDblNormal(0,1);
        football_state_.pose.position.z     =  model_states_msg_.pose[football_index_].position.z;
        football_state_.twist.linear.x      =  model_states_msg_.twist[football_index_].linear.x + scalar*rand_.GetDblNormal(0,1);
        football_state_.twist.linear.y      =  model_states_msg_.twist[football_index_].linear.y + scalar*rand_.GetDblNormal(0,1);
        football_state_.twist.linear.z      =  model_states_msg_.twist[football_index_].linear.z;

        nubot_state_.model_name = model_name_ ;
        nubot_state_.pose.position.x    =  model_states_msg_.pose[nubot_index_].position.x + scalar*rand_.GetDblNormal(0,1);
        nubot_state_.pose.position.y    =  model_states_msg_.pose[nubot_index_].position.y + scalar*rand_.GetDblNormal(0,1);
        nubot_state_.pose.position.z    =  model_states_msg_.pose[nubot_index_].position.z;
        nubot_state_.pose.orientation.w =  model_states_msg_.pose[nubot_index_].orientation.w;
        nubot_state_.pose.orientation.x =  model_states_msg_.pose[nubot_index_].orientation.x;
        nubot_state_.pose.orientation.y =  model_states_msg_.pose[nubot_index_].orientation.y;
        nubot_state_.pose.orientation.z =  model_states_msg_.pose[nubot_index_].orientation.z;
        nubot_state_.twist.linear.x     =  model_states_msg_.twist[nubot_index_].linear.x + scalar*rand_.GetDblNormal(0,1);
        nubot_state_.twist.linear.y     =  model_states_msg_.twist[nubot_index_].linear.y + scalar*rand_.GetDblNormal(0,1);
        nubot_state_.twist.linear.z     =  model_states_msg_.twist[nubot_index_].linear.z;
        nubot_state_.twist.angular.x    =  model_states_msg_.twist[nubot_index_].angular.x;
        nubot_state_.twist.angular.y    =  model_states_msg_.twist[nubot_index_].angular.y;
        nubot_state_.twist.angular.z    =  model_states_msg_.twist[nubot_index_].angular.z + scalar*rand_.GetDblNormal(0,1);
#endif

        // calculate vector from nubot to football
        nubot_football_vector_ = football_state_.pose.position - nubot_state_.pose.position;
        nubot_football_vector_.z = 0;                                                       // not consider z element
        nubot_football_vector_length_ = nubot_football_vector_.GetLength();                 // not consider z element

        // transform kick_vector_nubot in world frame
        math::Quaternion    rotation_quaternion = nubot_state_.pose.orientation;
        math::Matrix3       RotationMatrix3 = rotation_quaternion.GetAsMatrix3();
        kick_vector_world_ = RotationMatrix3 * kick_vector_nubot; // vector from nubot origin to kicking mechanism in world frame
        return 1;
   }
   else
   {
        ROS_INFO("%s update_model_info(): Waiting for model_states messages!", model_name_.c_str());
       return 0;
   }
}

void NubotGazebo::vel_cmd_CB(const nubot_common::VelCmd::ConstPtr& cmd)
{
   this->msgCB_lock_.lock();

    Vx_cmd_ = cmd->Vx;
    Vy_cmd_ = cmd->Vy;
    w_cmd_  = cmd->w;
    // The following calculation is based on robot body frame.
    math::Vector3 Vx_nubot = Vx_cmd_ * kick_vector_world_;
    math::Vector3 Vy_nubot = Vy_cmd_ * (math::Vector3(0,0,1).Cross(kick_vector_world_));
    math::Vector3 linear_vector = Vx_nubot + Vy_nubot;
    math::Vector3 angular_vector(0,0,w_cmd_);

    nubot_locomotion(linear_vector, angular_vector);

    this->msgCB_lock_.unlock();
}

bool NubotGazebo::ball_handle_control_service(nubot_common::BallHandle::Request  &req,
                                            nubot_common::BallHandle::Response &res)
{
    this->srvCB_lock_.lock();

    dribble_flag_ = req.enable ? 1 : 0;
    if(dribble_flag_)
    {
        if(!get_is_hold_ball())     // when dribble_flag is true, it does not necessarily mean that I can dribble it now.
        {                           // it just means the dribble ball mechanism is working.
            dribble_flag_ = false;
            res.BallIsHolding = false;
        }
        else
            res.BallIsHolding = true;
    }
    else
        res.BallIsHolding = get_is_hold_ball();

    this->srvCB_lock_.unlock();
    return true;
}

bool NubotGazebo::shoot_control_servive( nubot_common::Shoot::Request  &req,
                                       nubot_common::Shoot::Response &res )
{
    this->srvCB_lock_.lock();

    if(get_is_hold_ball())
    {
        dribble_flag_ = false;
        shot_flag_ = true;
        mode_ = (int)req.ShootPos;           //TODO: mode:-1,1,others
        force_ = (double)req.strength;       // FIXME: need conversion from force to velocity
        ROS_INFO("%s shoot_control_service(): ShootPos:%d strength:%f",model_name_.c_str(), mode_, force_);
        res.ShootIsDone = 1;
    }
    else
    {
        shot_flag_ = false;
        res.ShootIsDone = 0;
    }

    this->srvCB_lock_.unlock();
    return true;
}

void NubotGazebo::nubot_locomotion(math::Vector3 linear_vel_vector, math::Vector3 angular_vel_vector)
{
    desired_trans_vector_ = linear_vel_vector;
    desired_rot_vector_   = angular_vel_vector;
    // planar movement
    desired_trans_vector_.z = 0;
    desired_rot_vector_.x = 0;
    desired_rot_vector_.y = 0;
    this->nubot_model_->SetLinearVel(desired_trans_vector_);
    this->nubot_model_->SetAngularVel(desired_rot_vector_);
    judge_nubot_stuck_ = 1;                                                 // only afetr nubot tends to move can I judge if it is stuck
}


void NubotGazebo::kick_ball(int mode, double vel=20.0)
{
   math::Vector3 kick_vector_planar(kick_vector_world_.x, kick_vector_world_.y, 0.0);

   if(mode == RUN)
   {
        math::Vector3 vel_vector = kick_vector_planar * vel;
        set_ball_vel(vel_vector, ball_decay_flag_);
   }
   else if(mode == FLY)
   {
       /* math formular: y = a*x^2 + b*x + c;
          a = -g/(2*vx*vx), c = 0, b = kick_goal_height/D + g*D/(2.0*vx*vx)
          mid_point coordinates:[-b/(2*a), (4a*c-b^2)/(4a) ]
       */
        static const double kick_goal_height = goal_height - 0.22 - 0.02;                   // FIXME: can be tuned
        nubot::DPoint point1(nubot_state_.pose.position.x,nubot_state_.pose.position.y);
        nubot::DPoint point2(nubot_state_.pose.position.x + kick_vector_world_.x,
                             nubot_state_.pose.position.y + kick_vector_world_.y);
        nubot::DPoint point3(football_state_.pose.position.x,football_state_.pose.position.y);
        nubot::Line_ line1(point1, point2);
        nubot::Line_ line2(1.0, 0.0, kick_vector_world_.x>0 ? -goal_x : goal_x);            // nubot::Line_(A,B,C);

        nubot::DPoint crosspoint = line1.crosspoint(line2);
        double d = crosspoint.distance(point3);
        double vx_thres = d*sqrt(g/2/kick_goal_height);
        double vx = vx_thres/2.0>vel ? vel : vx_thres/2.0;                                  // initial x velocity.CAN BE TUNED
        double b = kick_goal_height/d + g*d/(2.0*vx*vx);

        math::Vector3 kick_vector(vx*kick_vector_world_.x, vx*kick_vector_world_.y, b*vx);
        set_ball_vel(kick_vector, ball_decay_flag_);
        ROS_INFO("%s crosspoint:(%f %f) vx: %f", model_name_.c_str(), crosspoint.x_, crosspoint.y_, vx);
        ROS_INFO("kick_vector:%f %f %f",kick_vector.x, kick_vector.y, kick_vector.z);
   }
   else
   {
       ROS_ERROR("%s kick_ball(): Incorrect mode!", model_name_.c_str());
   }
}


bool NubotGazebo::get_is_hold_ball(void)
{
    #if 1
    bool near_ball, allign_ball;
    double angle_error_degree;
    math::Vector3 norm = nubot_football_vector_;
    norm.z=0.0; norm.Normalize();
    kick_vector_world_.z=0.0;
    angle_error_degree = get_angle_PI(kick_vector_world_,norm)*(180/PI);

    allign_ball = (angle_error_degree <= dribble_angle_thres_/2.0
                    && angle_error_degree >= -dribble_angle_thres_/2.0) ?
                    1 : 0;
    near_ball = nubot_football_vector_length_ <= dribble_distance_thres_ ?
                1 : 0;
    return (near_ball && allign_ball);
    #endif
}

bool NubotGazebo::get_rot_vector(math::Vector3 target_point_world, double degree_thres)
{
    const double traj_const_ang_accel = 120.0;           // rotation accel for parabolic trajectory. degree/s^2. CAN TUNE
    double angle_error_degree;
    static bool is_finished;
    static math::Vector3 last_target_point(10.0,0.0,0.0);
    static common::Time last_current_time = world_->GetSimTime();
    static double traj_ang;

    // STEP 1: calculate cos angle between nubot_target_vector_ and kick_vector_world_
    math::Vector3 nubot_target_vector = target_point_world - nubot_state_.pose.position;
    math::Vector3 nubot_target_vector_norm = nubot_target_vector;
    nubot_target_vector_norm.z = 0;
    nubot_target_vector_norm.Normalize();
    kick_vector_world_.z = 0;
    angle_error_degree = get_angle_PI(kick_vector_world_,nubot_target_vector_norm)*(180/PI);

    // STEP 2: return calculated rotation vector
    if( angle_error_degree <= degree_thres/2.0 && angle_error_degree >= -degree_thres/2.0)
    {
        ROS_WARN("NO NEED FOR ROTATION!");
        traj_ang = 0.0;
        desired_rot_vector_ = ZERO_VECTOR;
        is_finished = 1;
    }
    else
    {
        if((last_target_point - target_point_world).GetLength() > 0.5)
        {
            last_current_time = world_->GetSimTime();
            double angle = angle_error_degree>0? angle_error_degree : -angle_error_degree;
            traj_plan_rot_.Init(traj_const_ang_accel,max_angular_vel_,angle - degree_thres );
        }
        double _dt = (world_->GetSimTime() - last_current_time).Double();
        traj_ang = traj_plan_rot_.VelUpdate(_dt);
        last_current_time = world_->GetSimTime();
        last_target_point = target_point_world;

        traj_ang = (traj_ang <= 10.0 && !is_finished) ? 10.0 : traj_ang;      // Bug fixed for error caused by friction

        math::Vector3 z_norm;
        if(angle_error_degree > 0)
            z_norm.Set(0.0,0.0,1.0);
        else
            z_norm.Set(0.0,0.0,-1.0);
        math::Vector3 nubot_rotation_vector = z_norm * (traj_ang * PI/180);
        desired_rot_vector_ = nubot_rotation_vector;

        is_finished = 0;
    }
    return is_finished;
}

bool NubotGazebo::get_trans_vector(math::Vector3 target_point_world, double metre_thres)
{
    const double traj_const_lin_accel = 2;         // linear accel for parabolic trajectory.  m/s^2. CAN TUNE
    static bool is_finished;
    static math::Vector3 last_target_point(10.0,0.0,0.0);
    static double traj_lin;
    static common::Time last_current_time = world_->GetSimTime();
    math::Vector3 nubot_target_vector = target_point_world - nubot_state_.pose.position;

    double nubot_target_vector_length = nubot_target_vector.GetLength ();
    ROS_INFO("%s nubot_target_vector_length: %f", model_name_.c_str(), nubot_target_vector_length);

    if(nubot_target_vector_length <= metre_thres)
    {
        ROS_WARN("NO NEED FOR TRANSLATION!");
        desired_trans_vector_ = ZERO_VECTOR;
        traj_lin = 0.0;
        is_finished = 1;
    }
    else
    {
        ROS_INFO("last_target:%f %f, target_point:%f %f",last_target_point.x, last_target_point.y, target_point_world.x, target_point_world.y);
        if((last_target_point - target_point_world).GetLength() > 0.5)                 // CAN TUNE
        {
            last_current_time = world_->GetSimTime();
            traj_plan_linear_.Init(traj_const_lin_accel, max_linear_vel_,
                                   nubot_target_vector_length - metre_thres );
        }
        //double _dt = (world_->GetSimTime() - last_current_time).Double();
        double _dt=0.007;
        traj_lin = traj_plan_linear_.VelUpdate(_dt);
        //last_current_time = world_->GetSimTime();
        last_target_point = target_point_world;

        traj_lin = (traj_lin <= 0.2 && !is_finished) ? 0.2 : traj_lin;      // Bug fixed for error caused by friction

        math::Vector3 nubot_translation_vector = nubot_target_vector;
        nubot_translation_vector = nubot_translation_vector.Normalize() * traj_lin;
        desired_trans_vector_ = nubot_translation_vector;
        is_finished = 0;
    }
    return is_finished;
}

void NubotGazebo::dribble_ball(void)
{
// There are two ways of ball-dribbling: 1. Set pose; 2. Set tangential velocity 3. Set secant velocity
#if 0   // 1. Set pose
    math::Quaternion    target_rot = nubot_model_->GetWorldPose().rot;
    math::Matrix3       RotationMatrix3 = target_rot.GetAsMatrix3();
    kick_vector_world_ = RotationMatrix3 * kick_vector_nubot;
    math::Vector3       relative_pos = kick_vector_world_* 0.43;
    math::Vector3       target_pos = nubot_model_->GetWorldPose().pos+ relative_pos;
    math::Pose          target_pose(target_pos, target_rot);
    football_model_->SetWorldPose(target_pose);
    //ROS_WARN("nubot_ball_distance:%f", (nubot_model_->GetWorldPose().pos - football_model_->GetWorldPose().pos).GetLength());
    football_state_.twist.linear = nubot_state_.twist.linear;
#endif

#if 1   // 2. Set tangential velocity
    static const double desired_bot_ball_len = dribble_distance_thres_;

    math::Vector3 actual_nubot_linvel = nubot_state_.twist.linear;
    math::Vector3 actual_nubot_rotvel = nubot_state_.twist.angular;
    if(football_state_.pose.position.z > 0.3)   // if football is in the air, cannot dribble
    {
        ROS_ERROR("dribble_ball(): ball is in the air at %f; return!", football_state_.pose.position.z);
        return;
    }

    math::Vector3     actual_ball_linear_vel = football_state_.twist.linear;
    actual_ball_linear_vel.z = 0;

    nubot_football_vector_.z = 0;                                                                // don't point to the air
    math::Vector3     perpencular_vel = actual_nubot_rotvel.Cross(nubot_football_vector_);       //debug
    math::Vector3     desired_football_vel = actual_nubot_linvel + perpencular_vel;
    desired_football_vel.z = 0;
    math::Vector3  output_vel = desired_football_vel;

    #if 0       // no need to use PI controller; it works much better without a controller
    /* velocity PID */
    double dt = (world_->GetSimTime() - last_current_time_).Double();
    double cmd = dribble_pid_.Update( desired_football_vel.GetLength() - actual_ball_linear_vel.GetLength(), dt);// error = target - actual
    last_current_time_ = world_->GetSimTime();

    output_vel = output_vel.Normalize() * cmd + desired_football_vel;      // feedfoward control
    #endif
    /* displacement pid */
    #if 0
    double dt = (world_->GetSimTime() - last_current_time_).Double();
    math::Vector3 nubot_target_vector_norm = nubot_football_vector_;
    nubot_target_vector_norm.Normalize();
    double cos_vector_angle = nubot_target_vector_norm.Dot(kick_vector_world_);
    double actual_bot_ball_len = cos_vector_angle>0 ? nubot_football_vector_length_ : -nubot_football_vector_length_;   // judge ball is in front of bot or behind bot
    double cmd = dribble_pid_.Update( desired_bot_ball_len - actual_bot_ball_len, dt);  // error = target - actual
    last_current_time_ = world_->GetSimTime();

    output_vel = output_vel.Normalize() * (cmd/dt) + desired_football_vel;       // feedfoward control
    #endif

    set_ball_vel(output_vel, ball_decay_flag_);
#endif

#if 0   // 3. Set secant velocity
        math::Vector3 actual_nubot_linvel = nubot_model_->GetWorldLinearVel();
        math::Vector3 actual_nubot_rotvel = nubot_model_->GetWorldAngularVel();
        math::Vector3 nubot_football_vector = football_model_->GetWorldPose().pos - nubot_model_->GetWorldPose().pos;
        actual_nubot_linvel.z=0; actual_nubot_rotvel.x=0; actual_nubot_rotvel.y=0;
        static common::Time last_current_time = world_->GetSimTime();
        const double dt = (world_->GetSimTime() - last_current_time).Double();
        double theta = actual_nubot_rotvel.GetLength() * dt;
        math::Matrix3 rota(cos(theta), -sin(theta), 0,
                           sin(theta), cos(theta),  0,
                           0,          0,           1);
        math::Vector3 b_vector = rota*nubot_football_vector;
        math::Vector3 c_vector = b_vector - nubot_football_vector;
        math::Vector3 perpencular_vel = c_vector/dt;
        set_ball_vel(perpencular_vel, ball_decay_flag_);
        last_current_time = world_->GetSimTime();
#endif
}

void NubotGazebo::nubot_be_control(void)
{
    if(nubot_state_.pose.position.z < 0.2)      // not in the air
        {
            if(dribble_flag_)       // dribble_flag_ is set by BallHandle service
            {
                 ROS_ERROR("%s is calling dribble_ball() in nubot_gazebo",model_name_.c_str());
                 dribble_ball();
            }

            if(shot_flag_)
            {
                ROS_ERROR("%s is callling kick_ball() in nubot_gazebo",model_name_.c_str());
                kick_ball(mode_, force_);
                shot_flag_ = false;
            }
        }
        else
        {
            ROS_FATAL("%s in the air!",model_name_.c_str());
        }
}

void NubotGazebo::nubot_auto_control(void)
{
    static math::Vector3 target_point;
    static math::Vector3 nubot_trans_vector;
    static math::Vector3 nubot_rot_vector;
    mode_ = FLY;   // kick_ball mode

    if(nubot_state_.pose.position.z > 0.01)          // if nubot is in the air, let it drop.
        return;

    is_hold_ball_ = get_is_hold_ball();

    switch(state_)
    {
       case CHASE_BALL:
       {
            if(!get_rot_vector  (football_state_.pose.position, 0.5))
            {
                nubot_locomotion(ZERO_VECTOR, desired_rot_vector_);
            }
            else if(!get_trans_vector(football_state_.pose.position, 0.40))
            {
                nubot_locomotion(desired_trans_vector_, ZERO_VECTOR);
            }
            else
            {
               state_ = DRIBBLE_BALL;
               ROS_ERROR("Go to next DRIBBLE_BALL state");
            }
            break; //CHASE_BALL break
       }
       case DRIBBLE_BALL:
       {
            if(!is_hold_ball_)
            {
                 football_model_->SetLinearVel(ZERO_VECTOR);
                 state_ = CHASE_BALL;
                 sub_state_ = MOVE_BALL;
                 ROS_WARN("%s Lose control of the ball!!!", model_name_.c_str());
            }
            else
                dribble_ball();

           switch(sub_state_)
           {
               case MOVE_BALL:
               {
                   target_point = math::Vector3(4.0, -2.0, 0.0);// goal0_pos - math::Vector3(2.0, 1.0, 0);
                   if(!get_trans_vector(target_point, 0.1))
                   {
                       nubot_locomotion(desired_trans_vector_, ZERO_VECTOR);
                   }
                   else
                   {
                       desired_trans_vector_ = ZERO_VECTOR;
                       desired_rot_vector_ = ZERO_VECTOR;
                       ROS_FATAL("Go to next state: ROTATE_BALL");
                       sub_state_ = ROTATE_BALL;
                   }
                   break;
               }
               case ROTATE_BALL:
               {
                   target_point = right_goal;
                   if(!get_rot_vector(target_point, 0.5))
                   {
                       nubot_locomotion(ZERO_VECTOR, desired_rot_vector_);
                   }
                   else
                   {
                        desired_trans_vector_ = ZERO_VECTOR;
                        desired_rot_vector_ = ZERO_VECTOR;
                        sub_state_ = MOVE_BALL;
                        state_ = KICK_BALL;
                   }
                   break;
               }
               default:
               {
                    break;
               }
           }
           break;   //DRIBBLE_BALL break
       }
       case KICK_BALL:
       {
           target_point = right_goal;
           switch(mode_)
           {
           case RUN:
                kick_ball(RUN, kick_ball_vel_);
                state_ = RESET;
               break;
           case FLY:
               kick_ball(FLY, kick_ball_vel_);
               state_ = RESET;
               break;
           default:
               state_ = RESET;
               break;
           }
           break;      //KICK_BALL break
       }
       case RESET:
        {

            if(!get_trans_vector(math::Vector3(-4.0,-2.0,0.0), 0.1))
            {
                nubot_locomotion(desired_trans_vector_, ZERO_VECTOR);
            }
            else
                state_ = HOME;
            break;
        }
       case HOME:
       {
            nubot_locomotion(ZERO_VECTOR,ZERO_VECTOR);
            break;
       }
    }   
    ROS_INFO("%s state_: %d, sub_state_:%d", model_name_.c_str(), state_, sub_state_);
}   


void NubotGazebo::ball_vel_decay(math::Vector3 vel, double mu)
{
    static double last_vel_len = vel.GetLength();
    double vel_len = vel.GetLength();

    if(vel_len > 0.0)
    {
        if(football_state_.pose.position.z <= 0.12 &&
                !(last_vel_len - vel_len > 0) )
        {
            static double force = -mu*m*g;
            football_link_->AddForce(vel.Normalize()*force);
        }
    }
    else if(vel_len < 0)
    {
        vel_len = 0.0;
        vel = ZERO_VECTOR;
        set_ball_vel(vel, ball_decay_flag_);
    }

    last_vel_len = vel_len;
}

void NubotGazebo::set_ball_vel(math::Vector3 &vel, bool &ball_decay_flag)
{
    football_model_->SetLinearVel(vel);
    ball_decay_flag_ = false;                        // setting linear vel to ball indicates it is not free rolling
                                                     // so no additional friction is applied now
}
