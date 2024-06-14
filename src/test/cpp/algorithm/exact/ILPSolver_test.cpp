#if GUROBI_ON
#include <gtest/gtest.h>

#include "algorithm/exact/ILPSolver.hpp"
#include "readwrite/pace.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::exact;
using namespace readwrite;

typedef std::vector<int> VI;
typedef std::vector<std::vector<int>> VVI;

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
// ILPSolverTest
//
TEST(ILPSolverTest, Solve) {
  int num_threads = 2;
  int seed = 10;
  util::set_log_level(util::logging::LogLevel::NONE);

  for (int version = 1; version <= 2; ++version) {
    {
      auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads);

      //----------------------------------------------------------------------------

      EXPECT_TRUE(solver.solve(4, 0, 0));
      EXPECT_TRUE(solver.is_feasible());

      EXPECT_EQ(solver.get_solution_weight(), 16);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      auto actual = solver.get_solution();
      expect_in(actual, VVI({
                            VI({0, 1, 0, 3, 4, 3, 0}),
                            VI({0, 3, 4, 3, 0, 1, 0}),
                        }));
      //----------------------------------------------------------------------------
      EXPECT_TRUE(solver.solve(2, 1, 1));  // k=2, s=t=1
      EXPECT_EQ(solver.get_solution_weight(), 0);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2}));
      EXPECT_EQ(solver.get_solution(), VI({1}));

      EXPECT_TRUE(solver.solve(3, 1, 1));
      EXPECT_EQ(solver.get_solution_weight(), 0);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2}));
      EXPECT_EQ(solver.get_solution(), VI({1}));

      EXPECT_TRUE(solver.solve(4, 1, 1));
      EXPECT_EQ(solver.get_solution_weight(), 16);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      EXPECT_EQ(solver.get_solution(), VI({1, 0, 3, 4, 3, 0, 1}));

      EXPECT_TRUE(solver.solve(4, 1, 2));
      EXPECT_EQ(solver.get_solution_weight(), 9);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      EXPECT_EQ(solver.get_solution(), VI({1, 0, 2}));

      EXPECT_TRUE(solver.solve(5, 1, 2));
      EXPECT_FALSE(solver.is_feasible());

      //----------------------------------------------------------------------------
      EXPECT_TRUE(solver.solve(3, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 6);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2}));
      EXPECT_EQ(solver.get_solution(), VI({0, 1, 0}));

      EXPECT_TRUE(solver.solve(0, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 0);
      EXPECT_EQ(solver.get_solution_colors(), VI());
      EXPECT_EQ(solver.get_solution(), VI({0}));

      EXPECT_TRUE(solver.solve(4, 2, 0));
      EXPECT_EQ(solver.get_solution_weight(), 12);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      EXPECT_EQ(solver.get_solution(), VI({2, 0, 1, 0}));

      EXPECT_TRUE(solver.solve(3, 2, 0));
      EXPECT_EQ(solver.get_solution_weight(), 12);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      EXPECT_EQ(solver.get_solution(), VI({2, 0, 1, 0}));

      EXPECT_TRUE(solver.solve(3, 3, 4));
      EXPECT_EQ(solver.get_solution_weight(), 13);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      EXPECT_EQ(solver.get_solution(), VI({3, 0, 1, 0, 3, 4}));

      EXPECT_TRUE(solver.solve(2, 4, 4));
      EXPECT_EQ(solver.get_solution_weight(), 6);
      EXPECT_EQ(solver.get_solution_colors(), VI({1, 3}));
      EXPECT_EQ(solver.get_solution(), VI({4, 3, 4}));

      EXPECT_TRUE(solver.solve(2, 4, 2));
      EXPECT_EQ(solver.get_solution_weight(), 11);
      EXPECT_EQ(solver.get_solution_colors(), VI({1, 3}));
      EXPECT_EQ(solver.get_solution(), VI({4, 3, 0, 2}));
    }

    {
      auto g = load_pace("src/test/resources/instances/002_tiny.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads);

      EXPECT_TRUE(solver.solve(4, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 15);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1, 2, 3}));
      auto actual = solver.get_solution();
      expect_in(actual, VVI({
                            VI({0, 1, 2, 0}),
                            VI({0, 2, 1, 0}),
                        }));
    }

    {
      auto g = load_pace("src/test/resources/instances/006_non_metric.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads);

      EXPECT_TRUE(solver.solve(2, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 12);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1}));
      EXPECT_EQ(solver.get_solution(), VI({0, 1, 2, 3, 2, 1, 0}));
    }
  }

  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(ILPSolverTest, SolveCorner) {
  int num_threads = 2;
  int seed = 10;
  util::set_log_level(util::logging::LogLevel::NONE);

  for (int version = 1; version <= 2; ++version) {
    // corner cases: 101-103
    for (int i = 101; i <= 103; ++i) {
      auto path = util::format("src/test/resources/instances/%d_corner.gr", i);
      auto g = load_pace(path.c_str()).graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(3, 6, 6));
      EXPECT_FALSE(solver.is_feasible());
    }

    // corner cases: 104-106
    for (int i = 104; i <= 106; ++i) {
      auto w = i - 104;
      auto path = util::format("src/test/resources/instances/%d_corner.gr", i);
      auto g = load_pace(path.c_str()).graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(3, 6, 6));
      EXPECT_EQ(solver.get_solution_weight(), w * 2);
      EXPECT_EQ(solver.get_solution_colors(), VI({1, 3, 5}));
      EXPECT_EQ(solver.get_solution(), VI({6, 4, 6}));
    }

    // corner case: 107
    {
      auto g = load_pace("src/test/resources/instances/107_corner.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(2, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 10);
      EXPECT_EQ(solver.get_solution_colors(), VI({0, 1}));
      EXPECT_EQ(solver.get_solution(), VI({0, 1, 0}));
    }
  }

  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(ILPSolverTest, SolveAdditional) {
  int num_threads = 2;
  int seed = 10;
  util::set_log_level(util::logging::LogLevel::NONE);

  for (int version = 1; version <= 2; ++version) {
    {
      auto g = load_pace("src/test/resources/instances/010_subdivided.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(5, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 30);

      EXPECT_TRUE(solver.solve(1, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 2);

      EXPECT_TRUE(solver.solve(2, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 6);
    }
    {
      auto g = load_pace("src/test/resources/instances/501_grid_3x3.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(8, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 10);
    }
    {
      auto g = load_pace("src/test/resources/instances/502_grid_4x4.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(15, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 16);
    }
    {
      auto g = load_pace("src/test/resources/instances/503_cycles.gr").graph;
      auto solver = ILPSolver(g, seed, num_threads, false, false, false, version);

      EXPECT_TRUE(solver.solve(11, 0, 0));
      EXPECT_EQ(solver.get_solution_weight(), 15);
    }
  }

  util::set_log_level(util::logging::LogLevel::TRACE);
}
#endif
