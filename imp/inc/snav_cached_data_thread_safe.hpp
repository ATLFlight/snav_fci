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

#ifndef SNAV_FCI_SNAV_CACHED_DATA_THREAD_SAFE_HPP_
#define SNAV_FCI_SNAV_CACHED_DATA_THREAD_SAFE_HPP_

#include <cstring>
#include <mutex>

#include "snav/snapdragon_navigator.h"

namespace snav_fci
{

class SnavCachedDataThreadSafe
{
public:
  SnavCachedDataThreadSafe()
  {
    snav_data_ptr_ = nullptr;
    initialized_ = false;
  }

  ~SnavCachedDataThreadSafe() {}

  inline int initialize()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    int ret_code = -1;
    if (!initialized_)
    {
      if (sn_get_flight_data_ptr(sizeof(SnavCachedData), &snav_data_ptr_) == 0)
      {
        if (sn_update_data() == 0)
        {
          initialized_ = true;
          ret_code = 0;
        }
      }
    }
    else
    {
      ret_code = 0;
    }

    return ret_code;
  }

  inline int update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return -1;

    return sn_update_data();
  }

  inline SnavCachedData get()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    SnavCachedData output;
    std::memset(&output, 0, sizeof(SnavCachedData));
    if (!initialized_) return output;
    if (!snav_data_ptr_) return output;
    std::memcpy(&output, snav_data_ptr_, sizeof(SnavCachedData));
    return output;
  }

private:
  SnavCachedData* snav_data_ptr_;
  bool initialized_;
  std::mutex mutex_;
};

} // namespace snav_fci

#endif // SNAV_FCI_SNAV_CACHED_DATA_THREAD_SAFE_HPP_

