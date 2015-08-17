/*
 * Copyright (C) 2014-2015 NUDT
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

#ifndef PARABOLIC_TRANSITION_PLANNING_HH
#define PARABOLIC_TRANSITION_PLANNING_HH

#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

namespace nubot
{
    /// \class ParaTrajPlanning
    /// \brief trajectory planning for parabolic curve transition.
    /// The trajectory consists of 3 part: two parabolic curve in the beginning and
    /// in the end; a straight line in the middle. This trajectory avoids the infinite
    /// acceleration at the beginning and at the end if just sepcify straight line trajectory
    class ParaTrajPlanning
    {
      /// \brief Constructor. Initialize para_traj_planning params
      /// \param[in] _accel     The acceleration.
      /// \param[in] _distance  The total distance to travel.
      /// \param[in] _max_vel   The maximum velocity.
      public: ParaTrajPlanning(double _accel = 0.0, double _max_vel = 0.0, double _distance = 0.0);

      /// \brief Destructor
      public: virtual ~ParaTrajPlanning();

      /// \brief Initialize para_traj_planning params
      /// \param[in] _accel     The acceleration.
      /// \param[in] _distance  The total distance to travel. If _distance is negative, it will
      ///                       changed to positive in this function.
      /// \param[in] _max_vel   The maximum velocity.
      public: void Init(double _accel = 0.0,  double _max_vel = 0.0, double _distance = 0.0);

      /// \brief Set the constant acceleration for the parabolic curve.
      /// \param[in] _accel the acceleration.
      public: void SetAccel(double _accel);

      /// \brief Set distance about to travel.
      /// \param[in] _distance the distance
      public: void SetDistance(double _distance);

      /// \brief Set the maximum velocity.
      /// \param[in] _vel the velocity
      public: void SetMaxVel(double _vel);

      /// \brief Get the constant acceleration.
      /// \return The constant acceleration
      public: double GetAccel() const;

      /// \brief Get distance about to travel.
      /// \return The distance
      public: double GetDistance() const;

      /// \brief Get the maximum velocity or the velocity of the straight line part.
      /// \return The maximum velocity
      public: double GetMaxVel() const;

      /// \brief Get velocity of the trajectory.
      /// \param[_in] _dt Change in time since last update call.
      /// \return the velocity
      public: double VelUpdate(double _dt);

      /// \brief Get displacement of the trajectory.
      /// \param[in] _dt Change in time since last update call.
      /// \return the displacemet
      public: double DispUpdate(double _dt);

      /// \brief Assignment operator
      /// \param[in] _p a reference to a para_traj_plannining to assign values from
      /// \return reference to this instance
      public: ParaTrajPlanning &operator=(const ParaTrajPlanning &_p)
              {
                if (this == &_p)
                  return *this;

                this->acceleration_ = _p.acceleration_;
                this->distance_ = _p.distance_;
                this->max_vel_ = _p.max_vel_;
                this->time_ = _p.time_;
                this->output_vel_ = _p.output_vel_;
                this->output_disp_ = _p.output_disp_;
                this->t_b_ = _p.t_b_;
                this->t_f_ = _p.t_f_;

                this->Reset();
                return *this;
              }

      /// \brief Reset all the params, output_vel_ and output_disp_;
      public: void Reset();

      /// \brief constant acceleration of the parabolic curve.
      private: double acceleration_;

      /// \brief total distance to travel
      private: double distance_;

      /// \brief maximum velocity; the const velocity of the stragight line trajectory
      private: double max_vel_;

      /// \brief total time starting from 0
      private: double time_;

      /// \brief output velocity of the trajectory
      private: double output_vel_;

      /// \brief output displacement of the trajectory
      private: double output_disp_;

      /// \brief end time of the first parabolic curve
      private: double t_b_;

      /// \brief end time of the trajectory
      private: double t_f_;

      /// \brief flag to indicate if there is a straight line part
      private: bool is_straight_line_;
    };
}
#endif
