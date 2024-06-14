#include <gtest/gtest.h>

#include "algorithm/graph/eulerian.hpp"
#include "util/Random.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::graph;

typedef std::vector<int> VI;
typedef std::vector<std::pair<int, int>> VII;

std::pair<double, std::set<int>> verify_eulerian_tour(ds::graph::Graph const& graph, std::vector<int> const& tour) {
  EXPECT_EQ(tour.front(), tour.back());

  double weight = 0.0;
  std::set<int> visited;

  for (std::size_t i = 1; i < tour.size(); ++i) {
    int u = tour[i - 1], v = tour[i];

    weight += graph.get_weight(u, v);
    visited.insert(u);
  }
  return {weight, visited};
}

//
// EeulerianTest
//
TEST(EulerianTest, EulerianPath) {
  EXPECT_EQ(eulerian_path(VII({{3, 1}, {1, 3}}), 3), VI({3, 1, 3}));
  EXPECT_EQ(eulerian_path(VII({{3, 1}, {3, 1}}), 1), VI({1, 3, 1}));
  EXPECT_EQ(eulerian_path(VII({{0, 1}, {1, 2}}), 0), VI({0, 1, 2}));
  EXPECT_EQ(eulerian_path(VII({{0, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 1}, {1, 0}}), 0), VI({0, 1, 1, 2, 2, 1, 0}));

  VII edges1 = {{6, 1}, {1, 0}, {0, 2}, {2, 4}, {4, 1}, {1, 6}, {1, 3}, {3, 1},
                {3, 5}, {3, 5}, {3, 5}, {5, 3}, {5, 3}, {4, 5}, {0, 4}, {0, 3}};
  EXPECT_EQ(eulerian_path(edges1, 6), VI({6, 1, 0, 2, 4, 0, 3, 1, 3, 5, 3, 5, 3, 5, 4, 1, 6}));
}

TEST(EulerianTest, EulerianPathRandom) {
  util::Random rand(12345);

  VI ns = {10, 20};
  VI ms = {0, 10, 35};
  for (auto n : ns) {
    for (auto m : ms) {
      for (int t = 0; t < 10; ++t) {
        VII edges;
        VI deg(n);

        // construct a tree
        for (int i = 1; i < n; ++i) {
          int p = rand.randint(0, i - 1);
          edges.push_back({p, i});
          ++deg[p];
          ++deg[i];
        }

        // add random edges
        for (int i = 0; i < m; ++i) {
          int u = rand.randint(0, n - 1);
          int v = rand.randint(0, n - 1);
          ++deg[u];
          ++deg[v];
          edges.push_back({u, v});
        }

        // level odd-degree vertices
        VI odd_vs;
        for (int i = 0; i < n; ++i) {
          if (deg[i] % 2 == 1) odd_vs.push_back(i);
        }
        EXPECT_TRUE(odd_vs.size() % 2 == 0);

        rand.shuffle(odd_vs);
        for (std::size_t i = 0; i < odd_vs.size(); i += 2) edges.push_back({odd_vs[i], odd_vs[i + 1]});

        // run function
        int s = rand.randint(0, n - 1);
        auto ret = eulerian_path(edges, s);

        EXPECT_EQ(ret.size(), edges.size() + 1);

        // reconstruct edges
        VII used_edges;
        for (std::size_t i = 0; i < ret.size() - 1; ++i) {
          used_edges.push_back({std::min(ret[i], ret[i + 1]), std::max(ret[i], ret[i + 1])});
        }
        std::sort(used_edges.begin(), used_edges.end());

        VII sorted_edges;
        for (auto& e : edges) sorted_edges.push_back({std::min(e.first, e.second), std::max(e.first, e.second)});
        std::sort(sorted_edges.begin(), sorted_edges.end());

        EXPECT_EQ(used_edges, sorted_edges);
      }
    }
  }
}
