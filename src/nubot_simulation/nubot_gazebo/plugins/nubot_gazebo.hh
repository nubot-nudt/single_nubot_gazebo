#ifndef NUBOT_GAZEBO_HH
#define NUBOT_GAZEBO_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>             // the core gazebo header files, including gazebo/math/gzmath.hh
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/callback_queue.h>         // Custom Callback Queue
#include <ros/subscribe_options.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include "nubot_common/VelCmd.h"
#include "nubot_common/Shoot.h"
#include "nubot_common/BallHandle.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

#include "nubot/core/core.hpp"

#include <nubot_gazebo/NubotGazeboConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pthread.h>
#include "nubot_PID.hh"
#include "parabolic_transition_planning.hh"


enum nubot_state
{
    CHASE_BALL,
    DRIBBLE_BALL,
    KICK_BALL,
    RESET,
    HOME
};

enum nubot_substate
{
    MOVE_BALL,
    ROTATE_BALL
};

namespace gazebo{
    /// \class NubotGazebo
    /// \brief A basic motions realization in Gazebo.
   struct Pose
   {
       math::Vector3    position;
       math::Quaternion orientation;
   };
   struct Twist
   {
       math::Vector3    linear;
       math::Vector3    angular;
   };

   struct model_state
   {
       std::string model_name;
       Pose        pose;
       Twist       twist;
       std::string reference_frame;
   };                                       // similar to the type gazebo_msgs::ModelState;
                                            // use these structs for easy handling of model states.
  class NubotGazebo : public ModelPlugin
  {      
    private:

        physics::WorldPtr           world_;             // A pointer to the gazebo world.
        physics::ModelPtr           nubot_model_;       // Pointer to the model
        physics::ModelPtr           football_model_;    // Pointer to the football
        physics::LinkPtr            football_link_;     //Pointer to the football link
        physics::LinkPtr            nubot_link_;
        
        ros::NodeHandle*            rosnode_;           // A pointer to the ROS node. 
        ros::Subscriber             ModelStates_sub_;
        ros::Subscriber             Velcmd_sub_;
        ros::ServiceServer          ballhandle_server_;
        ros::ServiceServer          shoot_server_;

        boost::thread               message_callback_queue_thread_;     // Thead object for the running callback Thread.
        boost::thread               service_callback_queue_thread_;
        boost::mutex                msgCB_lock_;        // A mutex to lock access to fields that are used in ROS message callbacks
        boost::mutex                srvCB_lock_;        // A mutex to lock access to fields that are used in ROS service callbacks
        ros::CallbackQueue          message_queue_;     // Custom Callback Queue. Details see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
        ros::CallbackQueue          service_queue_;     // Custom Callback Queue
        event::ConnectionPtr        update_connection_;         // Pointer to the update event connection
        
        gazebo_msgs::ModelStates    model_states_msg_;          // Container for the ModelStates msg      
        model_state                 nubot_state_;
        model_state                 football_state_;
        common::Time                  receive_sim_time_;
        common::Time                  last_current_time_;

        math::Vector3               desired_rot_vector_;
        math::Vector3               desired_trans_vector_;
        math::Vector3               nubot_football_vector_;
        math::Vector3               kick_vector_world_;
        double                      nubot_football_vector_length_;
        std::string                 robot_namespace_;   // robot namespace. Not used yet.
        std::string                 model_name_;
        std::string                 football_name_;
        std::string                 football_chassis_;
        std::string                 robot_prefix_;
        unsigned int                football_index_;
        unsigned int                nubot_index_;

        double                      max_linear_vel_;
        double                      max_angular_vel_;
        double                      dribble_distance_thres_;
        double                      dribble_angle_thres_;
        double                      kick_ball_vel_;
        double                      Vx_cmd_;
        double                      Vy_cmd_;
        double                      w_cmd_;
        double                      force_;                     //kick ball force
        double                      dribble_P_;                 // P gain
        double                      dribble_I_;                 // I gain
        double                      dribble_D_;                 // D gain
        double                      I_term_max_;                     // maximum I term
        double                      I_term_min_;                     // minimum I term
        double                      cmd_max_;
        double                      cmd_min_;
        int                         mode_;                      //kick ball mode
        common::Time                last_update_time_;
        
        unsigned int                model_count_;                // Number of models
        bool                        dribble_flag_;
        bool                        shot_flag_;
        bool                        ModelStatesCB_flag_;         // Indicate receiving messages
        bool                        judge_nubot_stuck_;          // decide when to judge
        bool                        is_hold_ball_;
        bool                        ball_decay_flag_;
        int                         count_;

        nubot_state                 state_;
        nubot_substate              sub_state_;
        nubot::PID                  dribble_pid_;
        nubot::ParaTrajPlanning   traj_plan_linear_;
        nubot::ParaTrajPlanning   traj_plan_rot_;
        math::Rand                  rand_;
        dynamic_reconfigure::Server<nubot_gazebo::NubotGazeboConfig> *reconfigureServer_;

        /// \brief ModelStates message callback function
        /// \param[in] _msg model_states msg shared pointer
        void model_states_CB(const gazebo_msgs::ModelStates::ConstPtr& _msg);

        /// \brief VelCmd message callback function
        /// \param[in] cmd VelCmd msg shared pointer
        void vel_cmd_CB(const nubot_common::VelCmd::ConstPtr& cmd);

        /// \brief Ball handling service server function
        /// \param[in] req ball handle service request
        /// \param[out] res ball handle service response
        bool ball_handle_control_service(nubot_common::BallHandle::Request  &req,
                                      nubot_common::BallHandle::Response &res);

        /// \brief Ball shooting service server function
        /// \param[in] req ball handle service request
        /// \param[out] res ball handle service response
        bool shoot_control_servive(nubot_common::Shoot::Request  &req,
                                 nubot_common::Shoot::Response &res);

        /// \brief Custom message callback queue thread
        void message_queue_thread();

        /// \brief Custom service callback queue thread
        void service_queue_thread();

        /// \brief Updating models' states; By default, Gaussian noise is not added,
        ///        but you can add it by changing the flag in this function.
        /// \return 1: updating model info success 0: not success
        bool update_model_info(void);

        /// \brief Nubot moving fuction: rotation + translation
        /// \param[in] linear_vel_vector translation velocity 3D vector
        /// \param[in] angular_vel_vector rotation velocity 3D vector
        void nubot_locomotion(math::Vector3 linear_vel_vector, math::Vector3 angular_vel_vector);  // rotation + translation

        /// \brief Nubot rotates; kicking mechanism towards ball.
        /// \param[in] target_point_world target point coordinates in world frame
        /// \param[in] degree_thres       angle error threshold in degree
        /// \return    bool value indicate if reach the target orientation. 1: yes; 0:no
        bool get_rot_vector(math::Vector3 target_point_world, double degree_thres);

        /// \brief Nubot chasing football function.
        /// \param[in]   target_point_world target point coordinates in world frame
        /// \param[in]   metre_thres        distance error threshold in metres
        /// \return      bool value indicate if reach the target position. 1:yes; 0:no
        bool get_trans_vector(math::Vector3 target_point_world, double metre_thres);

        /// \brief Nubot dribbling ball function. The football follows nubot movement.
        /// \brief Three ways of ball-dribbling are provided: 1. Set ball pose;
        ///        2. Set tangential velocity; 3. Set secant velocity.
        void dribble_ball(void);

        /// \brief Nubot kicking ball. For more information, read the paper
        ///        "Weijia Yao et al., A Simulation System Based on ROS and Gazebo for RoboCup Middle Size League, 2015"
        /// \param[in] mode kick ball mode FLY or RUN
        /// \param[in] vel initial velocity of the ball kicked
        void kick_ball(int mode, double vel);   // including shooting the goal and passing the ball

        /// \brief  Get the value of flag is_hold_ball_
        /// \return 1: is holding ball 0: is not holding ball
        bool get_is_hold_ball(void);

        /// \brief Single nubot predefined autonomous motions
        void nubot_auto_control(void);

        /// \brief Robot motion controlled by real-robot code; or controlled by a keyboard
        /// \brief provide an interface, e.g. use messages to control robot velocity,
        /// \brief use services to control robot perform ball-shooting or ball-dribbling
        void nubot_be_control(void);

        /// \brief dynmaic recofigure calback function
        void config(nubot_gazebo::NubotGazeboConfig &config, uint32_t level);

        /// \brief a work-around for no rolling-friction in ODE. For more information, please read ODE manual.
        /// \param[in] vel football's current 3D velocity
        /// \param[in] mu friction coefficient
        void ball_vel_decay(math::Vector3 vel, double mu);

        /// \brief replace football_model_->SetLinearVel() with a flag to indicate vel decay
        /// \param[in] vel                  football's desired 3D velocity
        /// \param[out] ball_decay_flag     flag to indicate whether or not the slow down football
        void set_ball_vel(math::Vector3 &vel, bool &ball_decay_flag);
        
    public:        
        /// \brief Constructor. Will be called firstly
        NubotGazebo();

        /// \brief Destructor
        virtual ~NubotGazebo();
    
    protected:   
        /// \brief Load the controller.
        /// Required by model plugin. Will be called secondly
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) ;

        /// \brief Update the controller. It is running every simulation iteration.
        ///        So you can put your core code here.(in this code, either nubot_be_control() or nubot_auto_control())
        virtual void UpdateChild();

        /// \brief Model Initialization(after the Load function).
        /// Not required by model plugin. Will be called thirdly
        virtual void Init();

        /// \brief Model Reset function.
        /// Not required by model plugin. It is triggered when the world resets.
        virtual void Reset();
  };
}

#endif //! NUBOT_GAZEBO_HH
