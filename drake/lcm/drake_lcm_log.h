#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

// TODO(siyuan):

/**
 * A Lcm interface for logging Lcm messages to a file or playing back from a
 * existing log. Note the user is responsible for synchronizing the clock used
 * to generate the log and the clock used for playback.
 */
class DrakeLcmLog : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmLog);

  /**
   * Constructs a DrakeLcmLog.
   * @param file_name Log's file name for reading or writing.
   * @param is_write If false, this instance reads from the Lcm log
   * identified by @p file_name. If true, this instance writes to the Lcm log
   * whose name is given by @p file_name.
   *
   * @throws std::runtime_error if unable to open file.
   */
  explicit DrakeLcmLog(const std::string& file_name, bool is_write);

  /**
   * Writes an entry occurred at @p timestamp with content @p data to the log
   * file. The current implementation blocks until writing is done.
   * @param channel Channel name.
   * @param data Pointer to raw bytes.
   * @param data_size Number of bytes in @p data.
   * @param second Time in seconds when the message is published. Since
   * messages are save to the log file in the order of Publish calls, this
   * function should only be called with non-decreasing @p second.
   *
   * @throws std::logic_error if this instance is not constructed in write-only
   * mode.
   */
  void Publish(const std::string& channel, const void* data, int data_size,
               double second) override;

  /**
   * Subscribes @p handler to @p channel. Multiple handlers can subscribe to the
   * same channel.
   *
   * @throws std::logic_error if this instance is not constructed in read-only
   * mode.
   */
  void Subscribe(const std::string& channel,
                 DrakeLcmMessageHandlerInterface* handler) override;

  /**
   * Returns the time in seconds for the next logged message's occurrence time
   * or infinity if there are no more messages in the current log.
   *
   * @throws std::logic_error if this instance is not constructed in read-only
   * mode.
   */
  double GetNextMessageTime() const override;

  /**
   * Let `MSG` be the next message event in the log, if @p current_time matches
   * `MSG`'s timestamp, for every DrakeLcmMessageHandlerInterface `sub` that's
   * subscribed to `MSG`'s channel, invoke `sub`'s HandleMessage method. Then,
   * this function advances the log by exactly one message. This function does
   * nothing if `MSG` is null (end of log) or @p current_time does not match
   * `MSG`'s timestamp.
   *
   * @throws std::logic_error if this instance is not constructed in read-only
   * mode.
   */
  void DispatchMessageAndAdvanceLog(double current_time) override;

  /**
   * Returns true if this instance is constructed in write-only mode.
   */
  bool is_write_only() const { return is_write_; }

  /**
   * Converts @p timestamp (in microseconds) to time (in seconds) relative to
   * the starting time passed to the constructor.
   */
  double timestamp_to_second(uint64_t timestamp) const {
    return static_cast<double>(timestamp) / 1e6;
  }

  /**
   * Converts time (in seconds) relative to the starting time passed to the
   * constructor to a timestamp in microseconds.
   */
  uint64_t second_to_timestamp(double sec) const {
    return static_cast<uint64_t>(sec * 1e6);
  }

  void StartReceiveThread() override {}
  void StopReceiveThread() override {}

 private:
  const bool is_write_;

  std::map<std::string, std::vector<DrakeLcmMessageHandlerInterface*>>
      subscriptions_;

  mutable std::mutex mutex_;
  std::unique_ptr<::lcm::LogFile> log_;
  const ::lcm::LogEvent* next_event_{nullptr};
};

}  // namespace lcm
}  // namespace drake
