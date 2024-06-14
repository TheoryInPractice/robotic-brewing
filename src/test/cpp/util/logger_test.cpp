#include <gtest/gtest.h>
#include <iostream>
#include <regex>
#include <sstream>
#include <vector>

#include "util/logger.hpp"

using namespace std;

//
// RandomTest
//
TEST(LoggerTest, SetLogLevel) {
  // Capture stderr.
  testing::internal::CaptureStderr();

  // Testing.
  std::vector<util::logging::LogLevel> levels = {
      util::logging::LogLevel::TRACE,     //
      util::logging::LogLevel::DEBUG,     //
      util::logging::LogLevel::INFO,      //
      util::logging::LogLevel::WARNING,   //
      util::logging::LogLevel::ERROR,     //
      util::logging::LogLevel::CRITICAL,  //
      util::logging::LogLevel::SUCCESS,   //
      util::logging::LogLevel::NONE,      //
  };

  int counter = 0;
  for (auto level : levels) {
    util::set_log_level(level);

    log_trace_("t");
    log_trace("t%d", ++counter);
    log_debug_("d");
    log_debug("d%d", ++counter);
    log_info_("i");
    log_info("i%d", ++counter);
    log_warning_("w");
    log_warning("w%d", ++counter);
    log_error_("e");
    log_error("e%d", ++counter);
    log_critical_("c");
    log_critical("c%d", ++counter);
    log_success_("s");
    log_success("s%d", ++counter);
  }

  // Copy the captured string.
  std::string actual = testing::internal::GetCapturedStderr();

  // Mask date and time representations.
  std::regex datetime_re("\\d\\d\\d\\d-\\d\\d-\\d\\d \\d\\d:\\d\\d:\\d\\d,\\d\\d\\d");
  std::string converted = std::regex_replace(actual, datetime_re, "0000-00-00 00:00:00,000");

  std::string expect =
      // trace
      "\x1B[36m0000-00-00 00:00:00,000 [TRACE   ] t\x1B[0m\n"
      "\x1B[36m0000-00-00 00:00:00,000 [TRACE   ] t1\x1B[0m\n"
      "\x1B[2;37m0000-00-00 00:00:00,000 [DEBUG   ] d\x1B[0m\n"
      "\x1B[2;37m0000-00-00 00:00:00,000 [DEBUG   ] d2\x1B[0m\n"
      "0000-00-00 00:00:00,000 [INFO    ] i\n"
      "0000-00-00 00:00:00,000 [INFO    ] i3\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w\x1B[0m\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w4\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e5\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c6\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s7\x1B[0m\n"
      // debug
      "\x1B[2;37m0000-00-00 00:00:00,000 [DEBUG   ] d\x1B[0m\n"
      "\x1B[2;37m0000-00-00 00:00:00,000 [DEBUG   ] d8\x1B[0m\n"
      "0000-00-00 00:00:00,000 [INFO    ] i\n"
      "0000-00-00 00:00:00,000 [INFO    ] i9\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w\x1B[0m\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w10\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e11\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c12\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s13\x1B[0m\n"
      // info
      "0000-00-00 00:00:00,000 [INFO    ] i\n"
      "0000-00-00 00:00:00,000 [INFO    ] i14\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w\x1B[0m\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w15\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e16\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c17\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s18\x1B[0m\n"
      // warning
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w\x1B[0m\n"
      "\x1B[33m0000-00-00 00:00:00,000 [WARNING ] w19\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e20\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c21\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s22\x1B[0m\n"
      // error
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e\x1B[0m\n"
      "\x1B[31m0000-00-00 00:00:00,000 [ERROR   ] e23\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c24\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s25\x1B[0m\n"
      // critical
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c\x1B[0m\n"
      "\x1B[1;31m0000-00-00 00:00:00,000 [CRITICAL] c26\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s27\x1B[0m\n"
      // success
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s\x1B[0m\n"
      "\x1B[1;32m0000-00-00 00:00:00,000 [SUCCESS ] s28\x1B[0m\n"
      // none
      ;

  EXPECT_EQ(converted, expect);
}
