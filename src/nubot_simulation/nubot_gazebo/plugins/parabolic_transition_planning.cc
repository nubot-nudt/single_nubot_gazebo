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
  : a_(_accel), dis_(_distance), vm_(_max_vel)
{
    this->Reset();
    dis_ = _distance>0 ? _distance : -_distance;
    if(vm_ * vm_ < a_ * dis_)
    {
        is_straight_line_ = true;
        tb_ = vm_ / a_;
        tf_ = (a_*dis_ + vm_*vm_)/(a_*vm_);
    }
    else
    {
        is_straight_line_ = false;
        tb_ = sqrt(dis_ / a_);
        tf_ = 2.0 * tb_;
    }
}

ParaTrajPlanning::~ParaTrajPlanning()
{}

void ParaTrajPlanning::Init(double _accel, double _max_vel, double _distance)
{
    this->a_ = _accel;
    this->dis_ = _distance>0 ? _distance : -_distance;
    this->vm_ = _max_vel;

    this->Reset();
    if(vm_ * vm_ < a_ * dis_)
    {
        is_straight_line_ = true;
        tb_ = vm_ / a_;
        tf_ = (a_*dis_ + vm_*vm_)/(a_*vm_);
    }
    else
    {
        is_straight_line_ = false;
        tb_ = sqrt(dis_ / a_);
        tf_ = 2.0 * tb_;
    }
    ROS_INFO("Traj Initialization!! max_vel^2=%f a(%f)*d(%f)=%f is_straight_line=%d tf:%f",
              vm_ * vm_ ,
              a_,dis_,
              a_ * dis_,
              is_straight_line_,tf_);
}

void ParaTrajPlanning::SetAccel(double _accel)
{
    this->a_ = _accel;
}


void ParaTrajPlanning::SetDistance(double _distance)
{
    this->dis_ = _distance>0 ? _distance : -_distance;
}

void ParaTrajPlanning::SetMaxVel(double _vel)
{
    this->vm_ = _vel;
}

void ParaTrajPlanning::Reset()
{
    this->t_ = 0.0;
    this->out_vel_ = 0.0;
    this->out_disp_ = 0.0;
}

double ParaTrajPlanning::VelUpdate(double _dt)
{
    if(t_ < 0.0)
    {
        ROS_FATAL("ERROR TIME!");
        Reset();
        return -1;
    }

    if(is_straight_line_)
    {
        if(t_ < tb_)                // 0 < time_ < t_b_
        {
            out_vel_ = a_ * t_;
        }
        else if(t_ < tf_ - tb_)   // t_b_ <= time_ < t_f_ - t_b_
        {
            out_vel_ = vm_;
        }
        else if(t_ < tf_)          // t_f_ - t_b_ < time_ < t_f_
        {
            out_vel_ = a_ * (tf_ - t_);
        }
        else                            // time_ > t_f_
        {
            out_vel_ = 0.0;
        }
    }
    else
    {
        if(t_ < tb_)        // 0 < time_ < t_b_
        {
            out_vel_ = a_ * t_;
        }
        else if(t_ < tf_)   // t_b_ <= time_ < t_f_
        {
            out_vel_ = a_ * (tf_ - t_);
        }
        else                    // time_ > t_f_
        {
            out_vel_ = 0.0;
        }
    }

    t_ = t_ + _dt;
    return this->out_vel_;
}


double ParaTrajPlanning::DispUpdate(double _dt)
{
    if(t_ < 0.0)
    {
        ROS_FATAL("ERROR TIME!");
        Reset();
        return -1;
    }

    if(is_straight_line_)
    {
        if(t_ < tb_)                // 0 < time_ < t_b_
        {
            out_disp_= 0.5*a_*t_*t_;
        }
        else if(t_ < tf_ - tb_)   // t_b_ <= time_ < t_f_ - t_b_
        {
            out_disp_= 0.5*a_*tb_*tb_ + vm_*(t_-tb_);
        }
        else if(t_ < tf_)          // t_f_ - t_b_ < time_ < t_f_
        {
            out_disp_= 0.5*a_*tb_*tb_ + vm_*(tf_-2*tb_)+a_*tf_*(t_-tf_+tb_)-0.5*a_*(t_+tf_-tb_)*(t_-tf_+tb_);
        }
        else                            // time_ > t_f_
        {
            out_disp_= a_*tb_*(tf_-tb_);
        }
    }
    else
    {
        if(t_ < tb_)        // 0 < time_ < t_b_
        {
            out_disp_= 0.5*a_*t_*t_;
        }
        else if(t_ < tf_)   // t_b_ <= time_ < t_f_
        {
            out_disp_= 0.5*a_*tb_*tb_ + a_*tf_*(t_-tb_) - 0.5*a_*(t_+tb_)*(t_-tb_);
        }
        else                    // time_ > t_f_
        {
            out_disp_= a_*(tb_*tb_+0.5*tf_*tf_-tf_*tb_);
        }
    }

    t_ = t_ + _dt;
    return this->out_disp_;
}


double ParaTrajPlanning::GetAccel() const
{
    return this->a_;
}


double ParaTrajPlanning::GetDistance() const
{
    return this->dis_;
}


double ParaTrajPlanning::GetMaxVel() const
{
    return this->vm_;
}
