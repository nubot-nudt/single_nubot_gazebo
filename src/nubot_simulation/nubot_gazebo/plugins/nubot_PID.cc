/*
 * Copyright (C) 2015 NuBot Team of National University of Defense Technology
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

/* Desc: PID class
 * Author: Weijia Yao
 * Date: Jun 2015
 */


#include <math.h>
#include <cmath>
#include <stdio.h>
#include <ros/ros.h>

#include "gazebo/math/Helpers.hh"
#include "nubot_PID.hh"

using namespace gazebo;
using namespace nubot;

/////////////////////////////////////////////////
PID::PID(double _p, double _i, double _d, double _imax, double _imin,
         double _cmdMax, double _cmdMin)
  : pGain(_p), iGain(_i), dGain(_d), iMax(_imax), iMin(_imin),
    cmdMax(_cmdMax), cmdMin(_cmdMin)
{
  this->Reset();
}

/////////////////////////////////////////////////
PID::~PID()
{
}

/////////////////////////////////////////////////
void PID::Init(double _p, double _i, double _d, double _imax, double _imin,
         double _cmdMax, double _cmdMin)
{
  this->pGain = _p;
  this->iGain = _i;
  this->dGain = _d;
  this->iMax = _imax;
  this->iMin = _imin;
  this->cmdMax = _cmdMax;
  this->cmdMin = _cmdMin;

  this->Reset();
}

/////////////////////////////////////////////////
void PID::SetPGain(double _p)
{
  this->pGain = _p;
}

/////////////////////////////////////////////////
void PID::SetIGain(double _i)
{
  this->iGain = _i;
}

/////////////////////////////////////////////////
void PID::SetDGain(double _d)
{
  this->dGain = _d;
}

/////////////////////////////////////////////////
void PID::SetIMax(double _i)
{
  this->iMax = _i;
}

/////////////////////////////////////////////////
void PID::SetIMin(double _i)
{
  this->iMin = _i;
}

/////////////////////////////////////////////////
void PID::SetCmdMax(double _c)
{
  this->cmdMax = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdMin(double _c)
{
  this->cmdMin = _c;
}

/////////////////////////////////////////////////
void PID::Reset()
{
  this->pErrLast = 0.0;
  this->pErr = 0.0;
  this->iErr = 0.0;
  this->dErr = 0.0;
  this->cmd = 0.0;
}

/////////////////////////////////////////////////
double PID::Update(double _error, double _dt)
{
  this->pErr = _error;

  if (_dt == 0 || math::isnan(_error) || std::isinf(_error))
  {
    ROS_ERROR("ERROR! return 0");
    return 0.0;
  }

  pTerm = this->pGain * this->pErr;

  this->iErr = this->iErr + _dt * this->pErr;
  iTerm = this->iGain * this->iErr;
  if (iTerm > this->iMax)
  {
    ROS_ERROR("%f exceed max I term.", iTerm);
    iTerm = this->iMax;
    this->iErr = iTerm / this->iGain;
  }
  else if (iTerm < this->iMin)
  {
    ROS_ERROR("%f lower min I term.", iTerm);
    iTerm = this->iMin;
    this->iErr = iTerm / this->iGain;
  }

  if (_dt != 0)
  {
    this->dErr = (this->pErr - this->pErrLast) / _dt;
    this->pErrLast = this->pErr;
  }
  dTerm = this->dGain * this->dErr;

  this->cmd = pTerm + iTerm + dTerm;
  if (!math::equal(this->cmdMax, 0.0) && this->cmd > this->cmdMax)
    this->cmd = this->cmdMax;
  if (!math::equal(this->cmdMin, 0.0) && this->cmd < this->cmdMin)
    this->cmd = this->cmdMin;
  ROS_FATAL("p_err:%f i_err:%f d_err:%f i_term:%f, cmd:%f",this->pErr,this->iErr,
                                                   this->dErr,iTerm,cmd);

  return this->cmd;
}

/////////////////////////////////////////////////
void PID::SetCmd(double _cmd)
{
  this->cmd = _cmd;
}

/////////////////////////////////////////////////
double PID::GetCmd()
{
  return this->cmd;
}

/////////////////////////////////////////////////
void PID::GetErrors(double &_pe, double &_ie, double &_de)
{
  _pe = this->pErr;
  _ie = this->iErr;
  _de = this->dErr;
}

/////////////////////////////////////////////////
void PID::GetTerms(double &_pt, double &_it, double &_dt)
{
  _pt = this->pTerm;
  _it = this->iTerm;
  _dt = this->dTerm;
}

/////////////////////////////////////////////////
double PID::GetPGain() const
{
  return this->pGain;
}

/////////////////////////////////////////////////
double PID::GetIGain() const
{
  return this->iGain;
}

/////////////////////////////////////////////////
double PID::GetDGain() const
{
  return this->dGain;
}

/////////////////////////////////////////////////
double PID::GetIMax() const
{
  return this->iMax;
}

/////////////////////////////////////////////////
double PID::GetIMin() const
{
  return this->iMin;
}

/////////////////////////////////////////////////
double PID::GetCmdMax() const
{
  return this->cmdMax;
}

/////////////////////////////////////////////////
double PID::GetCmdMin() const
{
  return this->cmdMin;
}
