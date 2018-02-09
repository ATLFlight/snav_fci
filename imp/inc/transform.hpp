/****************************************************************************
 * Copyright (c) 2018 John A. Dougherty. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
 ****************************************************************************/

#ifndef SNAV_FCI_TRANSFORM_HPP_
#define SNAV_FCI_TRANSFORM_HPP_

#include <cmath>
#include <mutex>

#include <Eigen/Geometry>

namespace snav_fci
{

class Transform
{
public:
  Transform() : q(1,0,0,0), t(0,0,0) {}

  ~Transform() {}

  inline Transform operator* (const Transform& tf) const
  {
    Transform tf_out;
    tf_out.q = q * tf.q;
    tf_out.t = t + q * tf.t;
    return tf_out;
  }

  inline StateVector operator* (const StateVector& sv) const
  {
    StateVector sv_out;
    sv_out.position = t + q*sv.position;
    sv_out.velocity = q*sv.velocity;
    sv_out.acceleration = q*sv.acceleration;
    sv_out.jerk = q*sv.jerk;

    Eigen::Quaternionf qtemp(
        Eigen::AngleAxisf(sv.yaw,
          Eigen::Vector3f::UnitZ()));
    Eigen::Matrix3f R_temp = (q*qtemp).toRotationMatrix();
    float yaw = 0;
    if (fabs(R_temp(0, 0)) < 1e-6 && fabs(R_temp(1, 0)) < 1e-6)
      yaw = 0;
    else
      yaw = atan2(R_temp(1, 0), R_temp(0, 0));

    sv_out.yaw = yaw;
    sv_out.yaw_rate = sv.yaw_rate;

    return sv_out;
  }

  inline Transform inverse() const
  {
    Transform tf_out;
    tf_out.q = this->q.inverse();
    tf_out.t = tf_out.q * this->t * -1;
    return tf_out;
  }

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
};

class TransformThreadSafe
{
public:
  TransformThreadSafe() {}

  ~TransformThreadSafe() {}

  inline void set(const Eigen::Quaternionf& q)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tf_.q = q;
  }

  inline void set(const Eigen::Vector3f& t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tf_.t = t;
  }

  inline void set(const Eigen::Quaternionf& q, const Eigen::Vector3f t)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tf_.q = q;
    tf_.t = t;
  }

  inline Eigen::Quaternionf get_rotation()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return tf_.q;
  }

  inline Eigen::Vector3f get_translation()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return tf_.t;
  }

  inline Transform get_transform()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return tf_;
  }

private:
  Transform tf_;
  std::mutex mutex_;
};

} // namespace snav_fci

#endif // SNAV_FCI_TRANSFORM_HPP_

