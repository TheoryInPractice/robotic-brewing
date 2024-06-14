#include <gtest/gtest.h>

#include "ds/queue/AdaptivePriorityQueue.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace ds::queue;

typedef std::vector<int> VI;

TEST(AdaptivePriorityQueueTest, BasicOperations) {
  AdaptivePriorityQueue<int, int> q;
  EXPECT_EQ(q.size(), 0);
  EXPECT_TRUE(q.empty());

  q.push(1, 5);
  q.push(3, -10);
  q.push(4, 7);
  q.push(5, 0);

  EXPECT_EQ(q.size(), 4);
  EXPECT_FALSE(q.empty());

  EXPECT_EQ(q.top(), make_pair(3, -10));
  q.pop();
  EXPECT_EQ(q.top(), make_pair(5, 0));

  q.push(5, 6);
  EXPECT_EQ(q.size(), 3);
  EXPECT_EQ(q.top(), make_pair(1, 5));

  q.clear();
  q.push(1, 5);
  q.push(1, 5);
  q.push(1, 7);
  q.push(1, 5);
  q.push(1, 5);
  EXPECT_EQ(q.size(), 1);
  q.pop();
  EXPECT_EQ(q.size(), 0);
}

TEST(AdaptivePriorityQueueTest, RandomInput) {
  util::Random rand(12345);

  for (int t = 0; t < 10; ++t) {
    AdaptivePriorityQueue<int, pair<int, double>> q;
    vector<pair<pair<int, double>, int>> v;

    for (int i = 0; i < 100; ++i) {
      if (rand.random() < 0.2) {
        if (!q.empty()) q.pop();
        if (!v.empty()) v.pop_back();
      } else {
        int j = rand.randint(0, 9);
        int x = rand.randint(0, 1000000000);
        double y = rand.random();
        q.push(j, make_pair(x, y));
        int found = -1;
        for (size_t k = 0; k < v.size(); ++k) {
          if (v[k].second == j) {
            found = k;
            break;
          }
        }

        if (found >= 0) {
          v[found] = make_pair(make_pair(x, y), j);
        } else {
          v.push_back(make_pair(make_pair(x, y), j));
        }
        sort(v.rbegin(), v.rend());
      }

      if (q.empty()) {
        EXPECT_TRUE(v.empty());
      } else {
        EXPECT_EQ(q.top(), make_pair(v.back().second, v.back().first));
      }
    }
  }
}
