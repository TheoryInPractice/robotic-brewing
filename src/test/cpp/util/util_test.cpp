#include <gtest/gtest.h>

#include "util/util.hpp"

using namespace std;

//
// UtilTest
//
TEST(UtilTest, ToString) {
  EXPECT_EQ(util::to_string(12345), "12345");
  EXPECT_EQ(util::to_string("abc"), "abc");
  EXPECT_EQ(util::to_string(std::pair<int, int>(1, 3)), "(1, 3)");
  EXPECT_EQ(util::to_string(std::vector<int>({1, 3, 5})), "[1, 3, 5]");
  EXPECT_EQ(util::to_string(std::list<int>({1, 3, 5})), "[1, 3, 5]");
  EXPECT_EQ(util::to_string(std::set<int>({1, 3, 5})), "{1, 3, 5}");
  EXPECT_EQ(util::to_string(std::unordered_set<int>({1})), "{1}");
  EXPECT_EQ(util::to_string(std::map<int, int>({{1, 2}, {3, 4}, {5, 6}})), "{1: 2, 3: 4, 5: 6}");
  EXPECT_EQ(util::to_string(std::unordered_map<int, int>({{1, 2}})), "{1: 2}");
}

TEST(UtilTest, Contains) {
  std::vector<int> xs = {1, 3, 5};
  std::set<int> ys = {1, 3, 5};
  std::map<int, int> zs = {{1, 2}, {3, 4}, {5, 6}};

  EXPECT_TRUE(util::contains(xs, 5));
  EXPECT_TRUE(util::contains(ys, 5));
  EXPECT_TRUE(util::contains(zs, 5));

  EXPECT_FALSE(util::contains(xs, 6));
  EXPECT_FALSE(util::contains(ys, 6));
  EXPECT_FALSE(util::contains(zs, 6));

  EXPECT_TRUE(util::contains(std::string("abc"), "abc"));
  EXPECT_TRUE(util::contains("abc", "abc"));
  EXPECT_TRUE(util::contains("abc", std::string("abc")));
  EXPECT_TRUE(util::contains("abc", "ab"));
  EXPECT_FALSE(util::contains("abc", "ac"));
  EXPECT_TRUE(util::contains("abc", ""));
  EXPECT_FALSE(util::contains("ab", "abc"));
}

TEST(UtilTest, Extend) {
  std::vector<int> xs = {1, 3, 5};
  std::vector<int> ys = {2, 4, 6};
  util::extend(xs, ys);
  EXPECT_EQ(xs, std::vector<int>({1, 3, 5, 2, 4, 6}));
}

TEST(UtilTest, Sorted) {
  std::vector<int> xs = {1, 3, 5, 2, 4, 6};
  EXPECT_EQ(util::sorted(xs), std::vector<int>({1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(xs, std::vector<int>({1, 3, 5, 2, 4, 6}));  // original remains the same
}

TEST(UtilTest, Split) {
  EXPECT_EQ(util::split("  123 4 5 678 "), std::vector<std::string>({"", "", "123", "4", "5", "678", ""}));
  EXPECT_EQ(util::split("  123 4 5 678 ", "4 5"), std::vector<std::string>({"  123 ", " 678 "}));
  EXPECT_EQ(util::split("  123 4 5 678 ", "8"), std::vector<std::string>({"  123 4 5 67", " "}));
  EXPECT_EQ(util::split("  123 4 5 678 ", "8 "), std::vector<std::string>({"  123 4 5 67", ""}));
  EXPECT_EQ(util::split("  123 4 5 678 ", "9"), std::vector<std::string>({"  123 4 5 678 "}));
}

TEST(UtilTest, Format) { EXPECT_EQ(util::format("a%d%s", 123, "bcd"), "a123bcd"); }
