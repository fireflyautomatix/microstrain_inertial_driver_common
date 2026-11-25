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

#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H

#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "microstrain/connections/connection.hpp"
#include "mip/mip_interface.hpp"

#include "microstrain_inertial_driver_common/utils/ros_compat.h"
#include "microstrain_inertial_driver_common/utils/async_file_writer.h"

namespace microstrain
{

// Predeclare here so we can pass as a parameter to the later configure step
class RosMipDevice;

/**
 * ROS implementation of the MIP connection class
 */
class RosConnection : public microstrain::Connection
{
 public:
  /**
   * \brief Constructs the ROS connection given a reference to the node that initialized it.
   * \param node Reference to the node that is initializing the connection
   */
  explicit RosConnection(RosNodeType* node);

  /**
   * \brief Tests if the connection is connected
   * \return true if the connection is connected
  */
  bool isConnected() const final;

  /**
   * \brief Connects this connection to the device
   * \return true if the connection succeeds
  */
  bool connect() final;

  /**
   * \brief Disconnects the connection from the device
   * \return true if the connection is disconnected
  */
  bool disconnect() final;

  /**
   * \brief Connects to the MIP device. When this function is finished, if it returns true, the device will be open, but not necesarilly ready to communicate
   * \param config_node Reference to a ROS node object that contains configuration information
   * \param port The serial port to connect to
   * \param baudrate The baudrate to open the serial port at
   * \return true if the connection was successful and false otherwise
   */
  bool connect(RosNodeType* config_node, const std::string& port, const int32_t baudrate);

  /**
   * \brief Configures the RosConnection object. This should be called after connect
   * \param config_node Reference to a ROS node object that contains configuration information
   * \param device The device to use to read information from the device
   * \return true if the configuration was successful and false otherwise
   */
  bool configure(RosNodeType* config_node, RosMipDevice* device);

  /**
   * \brief Gets whether or not this connection is parsing NMEA
   * \return Whether or not this connection is parsing NMEA
  */
  bool shouldParseNmea() const;

  /**
   * \brief Configures the connection object to attempt to parse or not attempt to parse NMEA sentences
   * \param enable Whether or not to enable NMEA parsing
  */
  void shouldParseNmea(bool enable);

  /**
   * \brief Returns the parse timeout for this connection object
   * \return the parse timeout to use with this connection object
   */
  mip::Timeout parseTimeout() const;

  /**
   * \brief Returns the base reply timeout for this connection object
   * \return the base reply timeout to use with this connection object
   */
  mip::Timeout baseReplyTimeout() const;

  /**
   * \brief Returns the NMEA messages collected by the connection, and clears the list of messages on the connection object
   * \return List of NMEA messages collected by the connection
   */
  std::vector<NMEASentenceMsg> nmeaMsgs();

  /**
   * \brief Gets whether we are recording to a raw file
   * \return Whether the raw file is enabled
   */
  bool rawFileEnable();

  /**
   * \brief Gets the raw file path where data is being recorded
   * \return Empty string if data is not being recorded
   */
  std::string rawFilePath();

  /**
   * \brief Updates the recording state, so will either start, or stop recording or switch files if needed
   * \param should_record Whether or not we should be recording data
   * \param record_file_path The full path to where the data should be recorded
   * \return Whether or not the state was able to be updated
   */
  bool updateRecordingState(const bool should_record, const std::string& record_file_path);

  /**
   * \brief Gets the number of bytes returned by the most recent read from the device
   * \return Bytes read by the last call to recvFromDevice, or 0 if it failed
   */
  size_t lastRecvLength() const;

  // Implemented in order to satisfy the requirements for the MIP connection
  bool sendToDevice(const uint8_t* data, size_t length) final;
  bool recvFromDevice(uint8_t* buffer, size_t max_length, unsigned int wait_time_ms, size_t* length_out, microstrain::EmbeddedTimestamp* timestamp_out) final;
  const char* interfaceName() const final;
  uint32_t parameter() const final;

 private:
  /**
   * \brief Extracts NMEA data from a byte array
   * \param data  Raw bytes that may contain a NMEA sentence
   * \param data_len  Length of the data array in bytes
   */
  void extractNmea(const uint8_t* data, size_t data_len);

  RosNodeType* node_;  /// Reference to the ROS node that created this connection

  std::unique_ptr<microstrain::Connection> connection_;  /// Connection object used to actually interact with the device
  mip::Timeout parse_timeout_ = 0;  /// Parse timeout given the type of connection configured
  mip::Timeout base_reply_timeout_ = 0;  /// Base reply timeout given the type of connection configured

  bool should_record_ = false;  /// Whether or not we should record binary data on this connection
  std::string record_file_path_;  /// The path to where data will be recorded
  AsyncFileWriter record_file_;  /// Writes the binary data on its own thread, so a slow disk can never stall reading the device
  uint64_t reported_dropped_bytes_ = 0;  /// Dropped recording bytes that have already been warned about
  size_t last_recv_length_ = 0;  /// Bytes returned by the most recent read from the device

  bool should_parse_nmea_ = false;  /// Whether or not we should attempt to parse and extract NMEA sentences on this connection
  std::string nmea_string_ = "";  /// Cached data read from the port, used to extraxt NMEA messages
  std::vector<NMEASentenceMsg> nmea_msgs_;  /// List of NMEA messages received by this connection
};

/**
 * \brief Keeps reading and parsing data that is already waiting on the connection, so a backlog is cleared in one call
 *        instead of one read per timer tick. Stops when a read returns no data, a read fails, or the time budget runs out
 * \param device The MIP interface to update
 * \param connection The connection the interface reads from
 * \param budget Longest time to spend
 * \return Number of bytes read
 */
size_t drainDevice(mip::Interface* device, const RosConnection& connection, std::chrono::milliseconds budget);

}  // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_MIP_ROS_CONNECTION_H
