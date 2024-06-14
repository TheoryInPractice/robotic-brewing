#include <gtest/gtest.h>

#include "algebra/MultilinearDetector.hpp"

using namespace std;
using namespace algebra;

typedef ArithmeticCircuit<ds::set::SortedVectorSet> Circuit;
typedef std::vector<int> VI;
typedef std::vector<VI> VVI;

template <typename T>
void expect_in(T &actual, std::vector<T> const &expect) {
  bool ok = false;
  for (auto &x : expect) {
    // log_warning("actual=%d, x=%d, result=%d", actual, x, actual == x);
    if (actual == x) {
      ok = true;
      break;
    }
  }
  EXPECT_TRUE(ok);
}

//
// MultilinearDetectorTest
//
TEST(MultilinearDetectorTest, Run) {
  util::set_log_level(util::logging::LogLevel::INFO);
  {
    Circuit C(5);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 2}), false), 5);
    EXPECT_EQ(C.add_addition_gate(VI({2, 3, 4}), false), 6);
    EXPECT_EQ(C.add_addition_gate(VI({2, 3, 4}), false), 7);
    EXPECT_EQ(C.add_addition_gate(VI({3, 4}), false), 8);
    EXPECT_EQ(C.add_multiplication_gate(VI({5, 6, 7, 8}), true), 9);

    util::Random rand(12341);

    for (int t = 0; t < 3; ++t) {
      MultilinearDetector detector1(C, 3);
      EXPECT_FALSE(detector1.run(rand, 10));

      MultilinearDetector detector2(C, 4);
      EXPECT_TRUE(detector2.run(rand, 10));

      auto cert = detector2.find_certificate(rand);
      auto actual = cert.evaluate(VI({2, 3, 5, 7, 11}));
      log_trace("case 1 (%d): actual=%s", t, cstr(actual));
      expect_in(actual, VVI({VI({2 * 5 * 7 * 11}), VI({3 * 5 * 7 * 11})}));
    }
  }
  //----------------------------------------------------------------------------
  {
    Circuit C(6);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 4}), false), 6);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 4}), false), 7);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 4}), false), 8);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 4}), false), 9);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1, 4}), false), 10);
    EXPECT_EQ(C.add_multiplication_gate(VI({2, 10}), false), 11);
    EXPECT_EQ(C.add_multiplication_gate(VI({3, 9}), false), 12);
    EXPECT_EQ(C.add_multiplication_gate(VI({4, 8}), false), 13);
    EXPECT_EQ(C.add_multiplication_gate(VI({5, 7}), false), 14);
    EXPECT_EQ(C.add_addition_gate(VI({12, 5}), false), 15);
    EXPECT_EQ(C.add_addition_gate(VI({13, 14}), false), 16);
    EXPECT_EQ(C.add_addition_gate(VI({11, 5}), false), 17);
    EXPECT_EQ(C.add_addition_gate(VI({13, 14}), false), 18);
    EXPECT_EQ(C.add_multiplication_gate(VI({17, 18}), false), 19);
    EXPECT_EQ(C.add_multiplication_gate(VI({15, 16, 6}), false), 20);
    EXPECT_EQ(C.add_addition_gate(VI({19, 20}), true), 21);

    util::Random rand(12345);

    for (int t = 0; t < 3; ++t) {
      MultilinearDetector detector1(C, 2);
      EXPECT_FALSE(detector1.run(rand, 10));

      MultilinearDetector detector2(C, 3);
      EXPECT_TRUE(detector2.run(rand, 10));

      auto cert = detector2.find_certificate(rand);
      auto actual = cert.evaluate(VI({2, 3, 5, 7, 11, 13}));
      log_trace("case 2 (%d): actual=%s", t, cstr(actual));
      expect_in(actual, VVI({VI({2 * 11 * 13}), VI({3 * 11 * 13})}));
    }
  }
  //----------------------------------------------------------------------------
  {
    // x_0^4 + x_0 x_2 + x_1 x_2
    Circuit C(3);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1}), false), 3);
    EXPECT_EQ(C.add_multiplication_gate(VI({3, 2}), false), 4);
    EXPECT_EQ(C.add_addition_gate(VI({0}), false), 5);
    EXPECT_EQ(C.add_multiplication_gate(VI({5, 0}), false), 6);
    EXPECT_EQ(C.add_multiplication_gate(VI({6, 0}), false), 7);
    EXPECT_EQ(C.add_multiplication_gate(VI({7, 0}), false), 8);
    EXPECT_EQ(C.add_addition_gate(VI({4, 8}), true), 9);

    util::Random rand(12341);

    for (int t = 0; t < 3; ++t) {
      MultilinearDetector detector1(C, 1);
      EXPECT_FALSE(detector1.run(rand, 10));

      MultilinearDetector detector2(C, 2);
      EXPECT_TRUE(detector2.run(rand, 10));

      auto cert = detector2.find_certificate(rand);
      auto actual = cert.evaluate(VI({2, 3, 5}));
      log_trace("case 3 (%d): actual=%s", t, cstr(actual));
      expect_in(actual, VVI({VI({2 * 5}), VI({3 * 5})}));

      MultilinearDetector detector3(C, 3);
      EXPECT_TRUE(detector3.run(rand, 10));

      MultilinearDetector detector4(C, 4);
      EXPECT_TRUE(detector4.run(rand, 10));
    }
  }
  //----------------------------------------------------------------------------
  {
    Circuit C(5);
    EXPECT_EQ(C.add_addition_gate(VI({1, 2, 3}), false), 5);
    EXPECT_EQ(C.add_addition_gate(VI({3, 4}), false), 6);
    EXPECT_EQ(C.add_addition_gate(VI({3, 4}), false), 7);
    EXPECT_EQ(C.add_addition_gate(VI({1, 2, 3}), false), 8);
    EXPECT_EQ(C.add_addition_gate(VI({1, 2, 3}), false), 9);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1}), false), 10);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1}), false), 11);
    EXPECT_EQ(C.add_multiplication_gate(VI({10, 11}), false), 12);
    EXPECT_EQ(C.add_multiplication_gate(VI({0, 9}), false), 13);
    EXPECT_EQ(C.add_multiplication_gate(VI({8, 4}), false), 14);
    EXPECT_EQ(C.add_multiplication_gate(VI({6, 7}), false), 15);
    EXPECT_EQ(C.add_addition_gate(VI({12, 13}), false), 16);
    EXPECT_EQ(C.add_addition_gate(VI({14, 15}), false), 17);
    EXPECT_EQ(C.add_multiplication_gate(VI({5, 16, 17}), true), 18);

    util::Random rand(12345);

    for (int t = 0; t < 3; ++t) {
      MultilinearDetector detector1(C, 4);
      EXPECT_FALSE(detector1.run(rand, 10));

      MultilinearDetector detector2(C, 5);
      EXPECT_TRUE(detector2.run(rand, 10));

      auto cert = detector2.find_certificate(rand);
      auto actual = cert.evaluate(VI({2, 3, 5, 7, 11}));
      log_trace("case 4 (%d): actual=%s", t, cstr(actual));
      EXPECT_EQ(actual, VI({2 * 3 * 5 * 7 * 11}));
    }
  }
  //----------------------------------------------------------------------------
  {
    Circuit C(2);
    EXPECT_EQ(C.add_multiplication_gate(VI({0, 1}), false), 2);
    EXPECT_EQ(C.add_addition_gate(VI({2}), false), 3);
    EXPECT_EQ(C.add_addition_gate(VI({0, 1}), true), 4);

    util::Random rand(12345);

    for (int t = 0; t < 3; ++t) {
      MultilinearDetector detector1(C, 1);
      EXPECT_TRUE(detector1.run(rand, 10));

      auto cert = detector1.find_certificate(rand);
      auto actual = cert.evaluate(VI({2, 3}));
      log_trace("case 5 (%d): actual=%s", t, cstr(actual));
      expect_in(actual, VVI({VI({2}), VI({3})}));
    }
  }
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(MultilinearDetectorTest, RunMultithread) {
  Circuit C(7);
  EXPECT_EQ(C.add_multiplication_gate(VI({0, 1}), false), 7);
  EXPECT_EQ(C.add_multiplication_gate(VI({1, 2}), false), 8);
  EXPECT_EQ(C.add_multiplication_gate(VI({2, 3}), false), 9);
  EXPECT_EQ(C.add_multiplication_gate(VI({3, 4}), false), 10);
  EXPECT_EQ(C.add_multiplication_gate(VI({4, 5}), false), 11);
  EXPECT_EQ(C.add_multiplication_gate(VI({5, 6}), false), 12);
  EXPECT_EQ(C.add_addition_gate(VI({7, 8, 10}), false), 13);
  EXPECT_EQ(C.add_addition_gate(VI({8, 9, 10}), false), 14);
  EXPECT_EQ(C.add_addition_gate(VI({2, 9, 11, 12}), false), 15);
  EXPECT_EQ(C.add_addition_gate(VI({10, 11, 12}), false), 16);
  EXPECT_EQ(C.add_multiplication_gate(VI({13, 1}), false), 17);
  EXPECT_EQ(C.add_multiplication_gate(VI({14, 3}), false), 18);
  EXPECT_EQ(C.add_multiplication_gate(VI({16, 6}), false), 19);
  EXPECT_EQ(C.add_addition_gate(VI({17, 18, 19}), false), 20);
  EXPECT_EQ(C.add_multiplication_gate(VI({13, 14, 15, 20}), false), 21);
  EXPECT_EQ(C.add_multiplication_gate(VI({20, 15, 16}), false), 22);
  EXPECT_EQ(C.add_addition_gate(VI({21, 22}), true), 23);

  util::Random rand(12344);

  util::set_log_level(util::logging::LogLevel::NONE);
  for (auto k : VI({5, 6, 7})) {
    for (auto t : VI({1, 2, 3})) {
      MultilinearDetector detector(C, k, t);
      EXPECT_EQ(detector.run(rand, 10), k >= 6);
    }
  }
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(MultilinearDetectorTest, RunMinimal) {
  Circuit C(2);
  EXPECT_EQ(C.add_multiplication_gate(VI({0, 1}), true), 2);

  util::Random rand(12345);

  util::set_log_level(util::logging::LogLevel::NONE);
  for (auto j = 1; j <= 3; ++j) {
    for (int t = 0; t < 50; ++t) {
      MultilinearDetector detector(C, 1, j);
      EXPECT_FALSE(detector.run(rand, 50));
    }
  }
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(MultilinearDetectorTest, Fix20240225) {
  // 007_tiny.gr with k=3
  Circuit C(3);
  EXPECT_EQ(C.add_addition_gate(VI(), true), 3);  // root

  // layer 1
  EXPECT_EQ(C.add_addition_gate(VI({1}), false), 4);
  EXPECT_EQ(C.add_addition_gate(VI({2}), false), 5);
  EXPECT_EQ(C.add_addition_gate(VI({0}), false), 6);
  EXPECT_EQ(C.add_addition_gate(VI({2}), false), 7);

  // layer 2
  EXPECT_EQ(C.add_multiplication_gate(VI({6, 1}), false), 8);
  EXPECT_EQ(C.add_addition_gate(VI({8}), false), 9);
  EXPECT_EQ(C.add_multiplication_gate(VI({7, 1}), false), 10);
  C.add_edge(10, 9);

  EXPECT_EQ(C.add_multiplication_gate(VI({6, 2}), false), 11);
  EXPECT_EQ(C.add_addition_gate(VI({11}), false), 12);

  EXPECT_EQ(C.add_multiplication_gate(VI({4, 0}), false), 13);
  EXPECT_EQ(C.add_addition_gate(VI({13}), false), 14);
  EXPECT_EQ(C.add_multiplication_gate(VI({7, 0}), false), 15);
  C.add_edge(15, 14);

  EXPECT_EQ(C.add_multiplication_gate(VI({4, 2}), false), 16);
  EXPECT_EQ(C.add_addition_gate(VI({16}), false), 17);
  EXPECT_EQ(C.add_multiplication_gate(VI({6, 2}), false), 18);
  C.add_edge(18, 17);

  // layer 3
  EXPECT_EQ(C.add_multiplication_gate(VI({17, 1}), false), 19);
  C.add_edge(19, 3);

  EXPECT_EQ(C.add_multiplication_gate(VI({17, 0}), false), 20);
  C.add_edge(20, 3);

  EXPECT_EQ(C.add_multiplication_gate(VI({9, 2}), false), 21);
  EXPECT_EQ(C.add_multiplication_gate(VI({14, 2}), false), 22);

  util::Random rand(12345);
  util::set_log_level(util::logging::LogLevel::NONE);
  for (int t = 0; t < 3; ++t) {
    MultilinearDetector detector(C, 3);

    auto cert = detector.find_certificate(rand);
    auto actual = cert.evaluate(VI({2, 3, 5}));
    EXPECT_EQ(actual, VI({30}));
  }
  util::set_log_level(util::logging::LogLevel::TRACE);
}
