/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"

namespace microstrain {

// Samples this much later than the estimate allows count as late, rather than
// as normal arrival jitter
static constexpr double LATE_THRESHOLD = 0.05;

// How long, in arrival time, samples must keep arriving late before it is
// treated as a clock change rather than a backlog
static constexpr double LATE_PERSISTENCE = 1.0;

ClockBiasMonitor::ClockBiasMonitor(const double weight,
                                   const double max_bias_estimate,
                                   const size_t delta_time_average_size,
                                   const double max_drift_rate)
    : weight_(weight), max_bias_estimate_(max_bias_estimate),
      max_drift_rate_(max_drift_rate),
      delta_time_average_size_(delta_time_average_size) {}

void ClockBiasMonitor::addTime(const double source_time,
                               const double target_time) {
  // Arrival latency only ever makes target_time later, so delta_time is largest
  // for the least latent samples
  const double delta_time = source_time - target_time;

  // Check if initialization is required
  if (!have_bias_estimate_) {
    delta_time_average_vector_.push_back(delta_time);
    if (delta_time_average_vector_.size() >= delta_time_average_size_) {
      // Use the least latent sample, not the average, so latency in the first
      // samples does not skew the estimate
      bias_estimate_ = *std::max_element(delta_time_average_vector_.begin(),
                                         delta_time_average_vector_.end());
      have_bias_estimate_ = true;
      delta_time_average_vector_.clear();
      late_ = false;
    }
    last_source_time_ = source_time;
    return;
  }

  // Much earlier than the estimate allows. Latency cannot do that, so one of
  // the clocks jumped
  if (delta_time - bias_estimate_ > max_bias_estimate_) {
    restart(delta_time);
    last_source_time_ = source_time;
    return;
  }

  // Clearly later than the estimate allows. Usually a backlog after the reading
  // thread was blocked, which is read in a short burst, so these samples are
  // not allowed to pull the estimate down. If samples keep arriving this late
  // for longer than a burst could last, one of the clocks changed, so start a
  // new estimate. Persistence is measured in arrival time: a backlog covers a
  // long span of source time but arrives in a short span of target time
  if (bias_estimate_ - delta_time > LATE_THRESHOLD) {
    if (!late_) {
      late_ = true;
      late_start_target_time_ = target_time;
    } else if (target_time - late_start_target_time_ > LATE_PERSISTENCE) {
      restart(delta_time);
    }
    last_source_time_ = source_time;
    return;
  }
  late_ = false;

  if (delta_time > bias_estimate_) {
    // Less latent than anything seen so far, so a better estimate
    bias_estimate_ = delta_time;
  } else {
    // More latent. Low pass filter towards it to follow real drift between the
    // clocks, but never faster than real drift could be, so a burst of late
    // samples cannot move the estimate much
    const double filtered_bias_estimate =
        bias_estimate_ * weight_ + delta_time * (1 - weight_);
    const double elapsed_source_time =
        std::max(0.0, source_time - last_source_time_);
    bias_estimate_ =
        std::max(filtered_bias_estimate,
                 bias_estimate_ - max_drift_rate_ * elapsed_source_time);
  }
  last_source_time_ = source_time;
}

bool ClockBiasMonitor::hasBiasEstimate() const { return have_bias_estimate_; }

double ClockBiasMonitor::getBiasEstimate() const { return bias_estimate_; }

void ClockBiasMonitor::reset() {
  have_bias_estimate_ = false;
  late_ = false;
  delta_time_average_vector_.clear();
}

void ClockBiasMonitor::restart(const double delta_time) {
  reset();
  delta_time_average_vector_.push_back(delta_time);
}

} // namespace microstrain
