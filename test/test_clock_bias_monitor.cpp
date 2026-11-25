// Tests for ClockBiasMonitor. The stall scenarios reproduce the shape of the
// GpsDataInterrupted events seen while recording a raw binary file: the driver
// thread is blocked for ~1-1.6 s, then the backlog of packets is read in a ~100
// ms burst. Arrival (target) times during the burst are late by up to the stall
// length.

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "microstrain_inertial_driver_common/utils/clock_bias_monitor.h"

using microstrain::ClockBiasMonitor;

static constexpr double SAMPLE_PERIOD = 0.01; // Sensor descriptor set at 100 Hz
static constexpr double TRUE_BIAS =
    1234.5; // source (device GPS) time - true ROS time
static constexpr double MIN_LATENCY =
    0.002; // Shortest time from measurement to arrival
static constexpr double JITTER =
    0.018; // Additional arrival latency, uniformly distributed
static constexpr double STAMP_TOLERANCE =
    0.005; // How far a stamp may sit ahead of its arrival time

struct Sample {
  double source_time; // Device time of the measurement
  double target_time; // ROS time when the packet was read
};

// Deterministic pseudo-random latency so the tests are repeatable
static double jitter(uint32_t *state) {
  *state = *state * 1664525u + 1013904223u;
  return JITTER * static_cast<double>(*state >> 8) /
         static_cast<double>(1u << 24);
}

// Generates samples at 100 Hz from t=0 to t=duration. Measurements taken in
// [stall_start, stall_start + stall_length) are not read until the stall ends,
// and are then read in a burst spread over burst_length.
static std::vector<Sample> makeSamples(const double duration,
                                       const double stall_start = -1,
                                       const double stall_length = 0,
                                       const double burst_length = 0.1) {
  std::vector<Sample> samples;
  uint32_t state = 42;
  const double stall_end = stall_start + stall_length;
  const size_t stalled_count =
      static_cast<size_t>(stall_length / SAMPLE_PERIOD);
  size_t stalled_index = 0;
  for (double t = 0; t < duration; t += SAMPLE_PERIOD) {
    double arrival = t + MIN_LATENCY + jitter(&state);
    if (stall_length > 0 && t >= stall_start && t < stall_end)
      arrival = stall_end + burst_length *
                                static_cast<double>(stalled_index++) /
                                static_cast<double>(stalled_count);
    samples.push_back({t + TRUE_BIAS, arrival});
  }
  return samples;
}

struct Result {
  double max_stamp_lead = -1e9; // Largest (stamp - arrival) seen. Positive
                                // means stamped in the future
  double final_bias = 0;
};

// Feeds samples through the monitor and computes the stamp the driver would
// give each one
static Result run(ClockBiasMonitor *monitor,
                  const std::vector<Sample> &samples) {
  Result result;
  for (const Sample &sample : samples) {
    monitor->addTime(sample.source_time, sample.target_time);
    if (monitor->hasBiasEstimate()) {
      const double stamp = sample.source_time - monitor->getBiasEstimate();
      result.max_stamp_lead =
          std::max(result.max_stamp_lead, stamp - sample.target_time);
    }
  }
  result.final_bias = monitor->getBiasEstimate();
  return result;
}

TEST(ClockBiasMonitor, SteadyStateTracksLeastLatentSample) {
  ClockBiasMonitor monitor(0.99, 1.0);
  const Result result = run(&monitor, makeSamples(30));
  ASSERT_TRUE(monitor.hasBiasEstimate());
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  // The estimate should land within the arrival jitter of the true bias
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, StallLongerThanResetThresholdNeverStampsInFuture) {
  // Nov 24 2026 Kohanaiki events at ...718 and ...1085: 1.6 s stall, backlog
  // read in ~100 ms
  ClockBiasMonitor monitor(0.99, 1.0);
  const Result result = run(&monitor, makeSamples(30, 10.0, 1.6));
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, StallShorterThanResetThresholdNeverStampsInFuture) {
  // Nov 24 2026 Kohanaiki event at ...905: 0.99 s stall
  ClockBiasMonitor monitor(0.99, 1.0);
  const Result result = run(&monitor, makeSamples(30, 10.0, 0.99));
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, RepeatedStallsDoNotDriftEstimate) {
  ClockBiasMonitor monitor(0.99, 1.0);
  std::vector<Sample> samples;
  for (int i = 0; i < 5; i++) {
    std::vector<Sample> chunk = makeSamples(10, 5.0, 0.6);
    for (Sample &sample : chunk) {
      sample.source_time += 10.0 * i;
      sample.target_time += 10.0 * i;
    }
    samples.insert(samples.end(), chunk.begin(), chunk.end());
  }
  const Result result = run(&monitor, samples);
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, FollowsTargetClockStepBackward) {
  // ROS time steps back 2 s: samples arrive "earlier" than any real latency
  // allows, so this must be a clock change
  ClockBiasMonitor monitor(0.99, 1.0);
  std::vector<Sample> samples = makeSamples(30);
  for (Sample &sample : samples)
    if (sample.source_time - TRUE_BIAS >= 15.0)
      sample.target_time -= 2.0;
  const Result result = run(&monitor, samples);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS + 2.0 - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, FollowsTargetClockStepForward) {
  // ROS time steps forward 2 s and stays there. This must not be mistaken for a
  // backlog forever
  ClockBiasMonitor monitor(0.99, 1.0);
  std::vector<Sample> samples = makeSamples(30);
  for (Sample &sample : samples)
    if (sample.source_time - TRUE_BIAS >= 15.0)
      sample.target_time += 2.0;
  const Result result = run(&monitor, samples);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - 2.0 - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, ResetClearsEstimate) {
  ClockBiasMonitor monitor(0.99, 1.0);
  run(&monitor, makeSamples(1));
  ASSERT_TRUE(monitor.hasBiasEstimate());
  monitor.reset();
  EXPECT_FALSE(monitor.hasBiasEstimate());
}

// Applies a lasting change of `shift` seconds to the arrival times of all samples measured from `at` onward
static std::vector<Sample> shiftTargetTime(std::vector<Sample> samples, const double at, const double shift)
{
  for (Sample& sample : samples)
    if (sample.source_time - TRUE_BIAS >= at)
      sample.target_time += shift;
  return samples;
}

TEST(ClockBiasMonitor, FollowsSubSecondTargetClockStepForwardQuickly)
{
  // ROS time steps forward 0.5 s (e.g. chrony makestep). The estimate must follow within a couple of seconds, not
  // drift there slowly, or every hybrid stamp would be 0.5 s stale for minutes
  ClockBiasMonitor monitor(0.99, 1.0);
  const Result result = run(&monitor, shiftTargetTime(makeSamples(17.5), 15.0, 0.5));
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - 0.5 - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, FollowsSubSecondSourceClockCorrectionQuickly)
{
  // Device time corrects backward 0.3 s, which looks the same to the monitor as ROS time stepping forward
  ClockBiasMonitor monitor(0.99, 1.0);
  std::vector<Sample> samples = makeSamples(17.5);
  for (Sample& sample : samples)
    if (sample.target_time >= 15.0)
      sample.source_time -= 0.3;
  const Result result = run(&monitor, samples);
  EXPECT_LE(result.max_stamp_lead, STAMP_TOLERANCE);
  EXPECT_NEAR(result.final_bias, TRUE_BIAS - 0.3 - MIN_LATENCY, JITTER);
}

TEST(ClockBiasMonitor, FollowsSubSecondTargetClockStepBackwardImmediately)
{
  ClockBiasMonitor monitor(0.99, 1.0);
  const Result result = run(&monitor, shiftTargetTime(makeSamples(15.5), 15.0, -0.5));
  EXPECT_NEAR(result.final_bias, TRUE_BIAS + 0.5 - MIN_LATENCY, JITTER);
}
