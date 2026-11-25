/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H

#include <vector>

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

/**
 * Estimates the offset between a source clock (device time) and a target clock (system time at which data arrived).
 *
 * Data can arrive late but never early, so the sample with the least arrival latency is the best estimate. The
 * estimate moves up to a less latent sample immediately, and moves down only slowly, at most at max_drift_rate.
 * This means a burst of late data (for example after the reading thread was blocked) cannot drag the estimate down
 * and cause timestamps to be computed in the future.
 */
class ClockBiasMonitor
{
 public:
  /**
   * \brief Constructor
   * \param weight How much to weight the old bias estimate vs the new delta time. Closer to 1 means more weight on the old bias estimate
   * \param max_bias_estimate A sample this much earlier than the estimate allows is treated as a clock jump and resets
   *                          the estimate immediately. Samples that keep arriving late for over a second also reset it
   * \param source_time_average_size Number of measurements to collect before the first estimate is made
   * \param max_drift_rate Fastest the estimate may decrease, in seconds per second of source time
  */
  explicit ClockBiasMonitor(const double weight = 0.5, const double max_bias_estimate = 1, const size_t source_time_average_size = 10,
                            const double max_drift_rate = 1e-3);

  /**
   * \brief Adds a new time. The system time should be from as close as possible to the device time
   * \param source_time The source time you want to convert from
   * \param target_time The target time we will calculate the bias for
  */
  void addTime(const double source_time, const double target_time);

  /**
   * \brief Checks if the monitor has a bias estimate, this will be set to true, once addTime has been called once
   * \return Whether or not the monitor has a bias estimate
  */
  bool hasBiasEstimate() const;

  /**
   * \brief Gets the current bias estimate.
   * \return The current bias estimate
  */
  double getBiasEstimate() const;

  /**
   * \brief Resets the bias estimate
   */
  void reset();

 private:
  /**
   * \brief Resets the estimate and uses the given sample as the first one towards a new estimate
   */
  void restart(const double delta_time);

  double weight_;  /// How much to weight the old bias estimate vs the new delta time. Closer to 1 means more weight on the old bias estimate
  double max_bias_estimate_;  /// Largest change from the estimate treated as latency rather than a clock jump
  double max_drift_rate_;  /// Fastest the estimate may decrease, in seconds per second of source time
  double last_source_time_ = 0.0;  /// Source time of the previous sample
  bool late_ = false;  /// Whether recent samples have all been clearly later than the estimate allows
  double late_start_target_time_ = 0.0;  /// Target time of the first of those late samples

  bool have_bias_estimate_ = false;  /// Will be set to true after getting the bias estimate for the first time
  double bias_estimate_ = 0.0;  /// Saved bias estimate. Can be used to convert a device time to a system time

  size_t delta_time_average_size_;  /// Number of source times we need to compute an average
  std::vector<double> delta_time_average_vector_;  /// Delta times collected before the first estimate is made
};

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_CLOCK_BIAS_MONITOR_H
