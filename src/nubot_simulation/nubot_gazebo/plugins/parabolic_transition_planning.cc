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

/* Desc: Simple trajectory planning with parabolic transition of displacement.
 * Author: Weijia Yao
 * Date: Jun 2015
 */

#include <math.h>
#include <cmath>
#include <stdio.h>
#include <ros/ros.h>

#include "parabolic_transition_planning.hh"

using namespace nubot;

ParaTrajPlanning::ParaTrajPlanning(double _accel, double _max_vel, double _distance)
  : acceleration_(_accel), distance_(_distance), max_vel_(_max_vel)
{
    this->Reset();
    distance_ = _distance>0 ? _distance : -_distance;
    if(max_vel_ * max_vel_ < acceleration_ * distance_)
    {
        is_straight_line_ = true;
        t_b_ = max_vel_ / acceleration_;
        t_f_ = (acceleration_*distance_ + max_vel_*max_vel_)/(acceleration_*max_vel_);
    }
    else
    {
        is_straight_line_ = false;
        t_b_ = sqrt(distance_ / acceleration_);
        t_f_ = 2.0 * t_b_;
    }
}

ParaTrajPlanning::~ParaTrajPlanning()
{}

void ParaTrajPlanning::Init(double _accel, double _max_vel, double _distance)
{
    this->acceleration_ = _accel;
    this->distance_ = _distance>0 ? _distance : -_distance;
    this->max_vel_ = _max_vel;

    this->Reset();
    if(max_vel_ * max_vel_ < acceleration_ * distance_)
    {
        is_straight_line_ = true;
        t_b_ = max_vel_ / acceleration_;
        t_f_ = (acceleration_*distance_ + max_vel_*max_vel_)/(acceleration_*max_vel_);
    }
    else
    {
        is_straight_line_ = false;
        t_b_ = sqrt(distance_ / acceleration_);
        t_f_ = 2.0 * t_b_;
    }
    ROS_FATAL("Traj Initialization!! max_vel^2=%f a(%f)*d(%f)=%f is_straight_line=%d tf:%f",
              max_vel_ * max_vel_ ,
              acceleration_,distance_,
              acceleration_ * distance_,
              is_straight_line_,t_f_);
}

void ParaTrajPlanning::SetAccel(double _accel)
{
    this->acceleration_ = _accel;
}


void ParaTrajPlanning::SetDistance(double _distance)
{
    this->distance_ = _distance>0 ? _distance : -_distance;
}

void ParaTrajPlanning::SetMaxVel(double _vel)
{
    this->max_vel_ = _vel;
}

void ParaTrajPlanning::Reset()
{
    this->time_ = 0.0;
    this->output_vel_ = 0.0;
    this->output_disp_ = 0.0;
}

double ParaTrajPlanning::VelUpdate(double _dt)
{
    if(time_ < 0.0)
    {
        ROS_FATAL("ERROR TIME!");
        Reset();
        return -1;
    }

    if(is_straight_line_)
    {
        if(time_ < t_b_)                // 0 < time_ < t_b_
        {
            output_vel_ = acceleration_ * time_;
        }
        else if(time_ < t_f_ - t_b_)   // t_b_ <= time_ < t_f_ - t_b_
        {
            output_vel_ = max_vel_;
        }
        else if(time_ < t_f_)          // t_f_ - t_b_ < time_ < t_f_
        {
            output_vel_ = acceleration_ * (t_f_ - time_);
        }
        else                            // time_ > t_f_
        {
            output_vel_ = 0.0;
        }
    }
    else
    {
        if(time_ < t_b_)        // 0 < time_ < t_b_
        {
            output_vel_ = acceleration_ * time_;
        }
        else if(time_ < t_f_)   // t_b_ <= time_ < t_f_
        {
            output_vel_ = acceleration_ * (t_f_ - time_);
        }
        else                    // time_ > t_f_
        {
            output_vel_ = 0.0;
        }
    }

    time_ = time_ + _dt;
    return this->output_vel_;
}


double ParaTrajPlanning::DispUpdate(double _dt)
{}


double ParaTrajPlanning::GetAccel() const
{
    return this->acceleration_;
}


double ParaTrajPlanning::GetDistance() const
{
    return this->distance_;
}


double ParaTrajPlanning::GetMaxVel() const
{
    return this->max_vel_;
}
