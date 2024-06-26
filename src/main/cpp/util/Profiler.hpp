#pragma once

#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace util {

/**
 * @brief Profiler.
 */
class Profiler {
  using ProfilerLabel = std::pair<std::string, int>;  // label name, parameter

  struct ProfilerData {
    int frequency;
    std::chrono::system_clock::duration accumulated_time;
    std::chrono::system_clock::duration best_time;
  };

  std::map<ProfilerLabel, std::chrono::system_clock::time_point> active_timer_;
  std::map<ProfilerLabel, ProfilerData> data_;
  std::map<ProfilerLabel, long long> counter_;
  std::chrono::system_clock::time_point last_report_;
  int report_interval_sec_;

 public:
  Profiler(int report_interval_sec = 0) : report_interval_sec_(report_interval_sec) {
    (void)report_interval_sec_;  // prevent not-used warnings

#if PROFILE_ON
    static_assert(PROFILE_ON);

    last_report_ = std::chrono::system_clock::now();
#endif
  }

  /**
   * @brief Updates report timings.
   *
   * @param interval_sec interval in seconds; set 0 for no interval reports
   */
  void set_report_interval(int interval_sec = 0) {
#if PROFILE_ON
    static_assert(PROFILE_ON);

    report_interval_sec_ = interval_sec;
#endif
  }

  /**
   * @brief Starts the timer.
   *
   * @param label label for timer
   * @param param optional parameter for the timer
   */
  void start(std::string const& label, int param = 0) {
#if PROFILE_ON
    static_assert(PROFILE_ON);

    auto lab = std::make_pair(label, param);
    if (active_timer_.find(lab) == active_timer_.end()) {
      // if the timer has already started, do nothing
      active_timer_[lab] = std::chrono::system_clock::now();
    }
#endif
  }

  /**
   * @brief Stops the timer.
   *
   * @param label label for timer
   * @param param optional parameter for the timer
   */
  void stop(std::string const& label, int param = 0) {
#if PROFILE_ON
    static_assert(PROFILE_ON);

    auto stopped = std::chrono::system_clock::now();
    auto lab = std::make_pair(label, param);
    auto it = active_timer_.find(lab);
    if (it != active_timer_.end()) {
      auto elapsed = stopped - it->second;
      active_timer_.erase(lab);
      data_[lab] = ProfilerData({data_[lab].frequency + 1, data_[lab].accumulated_time + elapsed,
                                 data_[lab].frequency == 0 ? elapsed : std::min(data_[lab].best_time, elapsed)});
    }

    // periodic report
    if (report_interval_sec_ > 0) {
      auto elapsed_from_last_report = std::chrono::duration_cast<std::chrono::seconds>(stopped - last_report_).count();
      if (elapsed_from_last_report >= report_interval_sec_) {
        print();
        last_report_ = stopped;
      }
    }
#endif
  }

  /**
   * @brief Counts the number of occurrences.
   *
   * @param label label for counter
   * @param param optional parameter for the counter
   */
  void count(std::string const& label, int param = 0) {
#if PROFILE_ON
    static_assert(PROFILE_ON);

    if constexpr (!PROFILE_ON) return;
    ++counter_[std::make_pair(label, param)];
#endif
  }

  /**
   * @brief Prints the profiler results.
   */
  void print() const {
#if PROFILE_ON
    static_assert(PROFILE_ON);

    fprintf(stderr, "[Timer]\n");
    if (data_.empty()) {
      fprintf(stderr, "None.\n");
    } else {
      // sort by elapsed time
      typedef std::pair<ProfilerLabel, ProfilerData> T;
      std::vector<T> d(data_.begin(), data_.end());
      std::sort(d.begin(), d.end(),
                [](T const& a, T const& b) { return a.second.accumulated_time > b.second.accumulated_time; });
      for (auto& kv : d) {
        auto count = kv.second.frequency;
        double ms = std::chrono::duration_cast<std::chrono::nanoseconds>(kv.second.accumulated_time).count();
        double bs = std::chrono::duration_cast<std::chrono::nanoseconds>(kv.second.best_time).count();

        fprintf(stderr, "%-30s (%5d): %10.3f sec %9d calls ", kv.first.first.c_str(), kv.first.second, ms * 1e-9, count);
        if (ms / count < 1e4) {
          fprintf(stderr, "[%10.0f ns /call; (min)%10.6f s]\n", ms / count, bs * 1e-9);
        } else {
          fprintf(stderr, "[%10.6f sec/call; (min)%10.6f s]\n", ms / count * 1e-9, bs * 1e-9);
        }
      }
    }

    fprintf(stderr, "\n[Counter]\n");
    if (counter_.empty()) {
      fprintf(stderr, "None.\n");
    } else {
      for (auto& kv : counter_) {
        //
        fprintf(stderr, "%-30s (%5d): %10lld\n", kv.first.first.c_str(), kv.first.second, kv.second);
      }
    }
    fprintf(stderr, "\n");
#endif
  }
};

/** Global profiler. */
extern Profiler prof;
}  // namespace util
