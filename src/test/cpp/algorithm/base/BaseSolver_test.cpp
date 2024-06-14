#include <gtest/gtest.h>

#include "algorithm/base/BaseSolver.hpp"
#include "readwrite/pace.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::base;
using namespace readwrite;

typedef std::vector<int> VI;
typedef Graph::Vertex Vertex;
typedef Graph::Edge Edge;
typedef Graph::Color Color;
typedef Graph::Weight Weight;
typedef Graph::ColoredVertexList CVL;
typedef Graph::WeightedEdgeList WEL;

// dummy class
struct DummySolver : public BaseSolver {
  DummySolver(Graph const& g) : BaseSolver("DummySolver", g) {}

  bool solve(int k, Vertex s, Vertex t, int time_limit = 0) override { return true; }
};

// fixture
class BaseSolverTest : public ::testing::Test {
 protected:
  virtual void SetUp() { util::set_log_level(util::logging::LogLevel::NONE); }
  virtual void TearDown() { util::set_log_level(util::logging::LogLevel::TRACE); }
};

//
// BaseSolverTest
//
TEST_F(BaseSolverTest, Properties) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  EXPECT_EQ(solver.get_solver_name(), "DummySolver");
}

TEST_F(BaseSolverTest, SplitColors) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  solver.split_colors();

  CVL expect_vs = {{0, {}}, {1, {0}}, {2, {1}}, {3, {1}}, {4, {3}}, {5, {1}}, {6, {2}}, {7, {3}}};
  WEL expect_es = {{{0, 1}, 3}, {{0, 2}, 6}, {{0, 3}, 2}, {{1, 5}, 0}, {{1, 6}, 0}, {{2, 7}, 0}, {{3, 4}, 3}};
  auto actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);
}

TEST_F(BaseSolverTest, RemoveUnreachableVertices) {
  auto g = load_pace("src/test/resources/instances/108_unreachable.gr").graph;
  auto solver = DummySolver(g);
  solver.remove_unreachable_vertices(0);

  CVL expect_vs = {{0, {}}, {3, {1}}, {4, {3}}};
  WEL expect_es = {{{0, 3}, 3}, {{0, 4}, 2}, {{3, 4}, 3}};
  auto actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);
}

TEST_F(BaseSolverTest, RemoveColorlessVertices) {
  auto g = load_pace("src/test/resources/instances/109_colorless.gr").graph;
  auto solver = DummySolver(g);
  solver.remove_colorless_vertices({0});

  CVL expect_vs = {{0, {}}, {1, {0, 1, 2}}, {3, {1}}};
  WEL expect_es = {{{0, 1}, 3}, {{0, 3}, 2}};
  auto actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);
}

TEST_F(BaseSolverTest, SmoothColorlessVertices) {
  Graph g1 = Graph(
      {
          {0, {}},
          {1, {10, 20}},
          {2, {30, 40}},
      },
      {
          {{0, 1}, 1},
          {{0, 2}, 2},
      });

  auto solver1 = DummySolver(g1);
  EXPECT_EQ(solver1.smooth_colorless_vertices(), 1);

  CVL expect_vs1 = {{1, {10, 20}}, {2, {30, 40}}};
  WEL expect_es1 = {{{1, 2}, 3}};
  auto actual1 = solver1.get_graph();
  EXPECT_EQ(actual1.colored_vertices(true), expect_vs1);
  EXPECT_EQ(actual1.weighted_edges(true), expect_es1);

  Graph g2 = Graph(
      {
          {0, {}},
          {1, {10, 20}},
          {2, {30, 40}},
      },
      {
          {{0, 1}, 1},
          {{0, 2}, 2},
          {{1, 2}, 2},
      });

  auto solver2 = DummySolver(g2);
  EXPECT_EQ(solver2.smooth_colorless_vertices(), 1);

  CVL expect_vs2 = {{1, {10, 20}}, {2, {30, 40}}};
  WEL expect_es2 = {{{1, 2}, 2}};
  auto actual2 = solver2.get_graph();
  EXPECT_EQ(actual2.colored_vertices(true), expect_vs2);
  EXPECT_EQ(actual2.weighted_edges(true), expect_es2);

  Graph g3 = Graph(
      {
          {0, {}},
          {1, {10, 20}},
          {2, {30, 40}},
      },
      {
          {{0, 1}, 1},
          {{0, 2}, 2},
          {{1, 2}, 4},
      });

  auto solver3 = DummySolver(g3);
  EXPECT_EQ(solver3.smooth_colorless_vertices(), 1);

  CVL expect_vs3 = {{1, {10, 20}}, {2, {30, 40}}};
  WEL expect_es3 = {{{1, 2}, 3}};  // weight should be updated
  auto actual3 = solver3.get_graph();
  EXPECT_EQ(actual3.colored_vertices(true), expect_vs3);
  EXPECT_EQ(actual3.weighted_edges(true), expect_es3);

  Graph g4 = Graph(
      {
          {0, {}},
          {1, {10, 20}},
          {2, {30, 40}},
      },
      {
          {{0, 1}, 1},
          {{0, 2}, 2},
          {{1, 2}, 4},
      });

  auto solver4 = DummySolver(g4);
  EXPECT_EQ(solver4.smooth_colorless_vertices(VI({0})), 0);

  CVL expect_vs4 = {{0, {}}, {1, {10, 20}}, {2, {30, 40}}};
  WEL expect_es4 = {{{0, 1}, 1}, {{0, 2}, 2}, {{1, 2}, 4}};
  auto actual4 = solver4.get_graph();
  EXPECT_EQ(actual4.colored_vertices(true), expect_vs4);
  EXPECT_EQ(actual4.weighted_edges(true), expect_es4);

  // more involved cases
  Graph g5 = Graph(
      {
          {0, {10}},
          {1, {}},
          {2, {}},
          {3, {}},
          {4, {}},
          {5, {}},
          {6, {}},
          {7, {}},
          {8, {}},
      },
      {
          {{4, 1}, 2},
          {{1, 5}, 3},
          {{1, 3}, 4},
          {{3, 6}, 2},
          {{3, 2}, 3},
          {{2, 8}, 4},
          {{2, 0}, 5},
      });

  auto solver5 = DummySolver(g5);
  EXPECT_EQ(solver5.smooth_colorless_vertices(VI({4})), 7);

  CVL expect_vs5 = {{0, {10}}, {4, {}}};
  WEL expect_es5 = {{{0, 4}, 14}};
  auto actual5 = solver5.get_graph();
  EXPECT_EQ(actual5.colored_vertices(true), expect_vs5);
  EXPECT_EQ(actual5.weighted_edges(true), expect_es5);

  Graph g6 = Graph(
      {
          {0, {}},
          {1, {}},
          {2, {}},
          {3, {}},
          {4, {}},
          {5, {}},
          {6, {}},
          {7, {}},
          {8, {}},
          {9, {}},
      },
      {
          {{3, 7}, 1},
          {{7, 4}, 1},
          {{7, 6}, 1},
          {{4, 0}, 1},
          {{4, 9}, 1},
          {{0, 9}, 1},
          {{4, 8}, 1},
          {{6, 8}, 1},
          {{2, 8}, 1},
          {{1, 8}, 1},
          {{5, 8}, 1},
          {{2, 8}, 1},
          {{1, 5}, 1},
          {{2, 5}, 1},
      });

  auto solver6 = DummySolver(g6);
  EXPECT_EQ(solver6.smooth_colorless_vertices(VI({3, 7})), 8);

  CVL expect_vs6 = {{3, {}}, {7, {}}};
  WEL expect_es6 = {{{3, 7}, 1}};
  auto actual6 = solver6.get_graph();
  EXPECT_EQ(actual6.colored_vertices(true), expect_vs6);
  EXPECT_EQ(actual6.weighted_edges(true), expect_es6);

  // no smoothing
  Graph g7 = Graph(
      {
          {0, {10}},
          {1, {}},
          {2, {}},
          {3, {}},
          {4, {}},
          {5, {20}},
      },
      {
          {{0, 1}, 1},
          {{1, 2}, 1},
          {{1, 3}, 1},
          {{2, 3}, 1},
          {{2, 4}, 1},
          {{3, 4}, 1},
          {{4, 5}, 1},
      });

  auto solver7 = DummySolver(g7);
  EXPECT_EQ(solver7.smooth_colorless_vertices(), 0);

  CVL expect_vs7 = {{0, {10}}, {1, {}}, {2, {}}, {3, {}}, {4, {}}, {5, {20}}};
  WEL expect_es7 = {{{0, 1}, 1}, {{1, 2}, 1}, {{1, 3}, 1}, {{2, 3}, 1}, {{2, 4}, 1}, {{3, 4}, 1}, {{4, 5}, 1}};
  auto actual7 = solver7.get_graph();
  EXPECT_EQ(actual7.colored_vertices(true), expect_vs7);
  EXPECT_EQ(actual7.weighted_edges(true), expect_es7);
}

TEST_F(BaseSolverTest, CreateTransitiveClosure) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  solver.create_transitive_closure();

  CVL expect_vs = {{0, {}}, {1, {0, 1, 2}}, {2, {1, 3}}, {3, {1}}, {4, {3}}};
  WEL expect_es = {{{0, 1}, 3}, {{0, 2}, 6}, {{0, 3}, 2}, {{0, 4}, 5},  {{1, 2}, 9},
                   {{1, 3}, 5}, {{1, 4}, 8}, {{2, 3}, 8}, {{2, 4}, 11}, {{3, 4}, 3}};

  auto actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);

  solver.create_transitive_closure();  // operation should be idempotent
  actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);

  g = load_pace("src/test/resources/instances/006_non_metric.gr").graph;
  solver = DummySolver(g);
  solver.create_transitive_closure(false);

  expect_vs = {{0, {}}, {1, {1}}, {2, {1}}, {3, {0}}};
  expect_es = {{{0, 1}, 3}, {{0, 2}, 4}, {{0, 3}, 200}, {{1, 2}, 1}, {{1, 3}, 100}, {{2, 3}, 2}};
  actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);

  g = load_pace("src/test/resources/instances/006_non_metric.gr").graph;
  solver = DummySolver(g);
  solver.create_transitive_closure(true);

  expect_vs = {{0, {}}, {1, {1}}, {2, {1}}, {3, {0}}};
  expect_es = {{{0, 1}, 3}, {{0, 2}, 4}, {{0, 3}, 6}, {{1, 2}, 1}, {{1, 3}, 3}, {{2, 3}, 2}};
  actual = solver.get_graph();
  EXPECT_EQ(actual.colored_vertices(true), expect_vs);
  EXPECT_EQ(actual.weighted_edges(true), expect_es);

  // disconnected graph
  g = load_pace("src/test/resources/instances/111_minimal.gr").graph;
  solver = DummySolver(g);
  solver.create_transitive_closure();
  EXPECT_EQ(solver.get_graph().number_of_edges(), 0);
}

TEST_F(BaseSolverTest, SetCriticalWalk) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  solver.split_colors();
  solver.create_transitive_closure();

  solver.set_critical_walk(VI({0, 5, 1, 4, 6, 7, 2, 0}));
  EXPECT_EQ(solver.get_solution(), VI({0, 1, 0, 3, 4, 3, 0, 1, 0, 2, 0}));

  // erorr case
  EXPECT_THROW(solver.set_critical_walk(VI()), std::invalid_argument);
}

TEST_F(BaseSolverTest, SetSolution) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  solver.set_critical_walk(VI({0, 1, 0, 3, 4, 3, 0}));

  EXPECT_EQ(solver.get_solution(), VI({0, 1, 0, 3, 4, 3, 0}));
  EXPECT_EQ(solver.get_solution_weight(), 16);
  EXPECT_EQ(solver.get_solution_colors().to_vector(), VI({0, 1, 2, 3}));
}

TEST_F(BaseSolverTest, FindMSTSolution) {
  auto g = load_pace("src/test/resources/instances/115_mst.gr").graph;
  auto solver = DummySolver(g);

  VI expect1 = {0,  3,  20, 3, 0, 2,  0, 1, 8, 19, 8,  18, 8,  1,  7,  1,  6,  1,  4,
                10, 12, 10, 4, 9, 11, 9, 4, 1, 5,  13, 15, 13, 14, 17, 14, 16, 14, 13};

  EXPECT_EQ(solver.find_mst_solution(0, 13), expect1);

  VI expect2 = {0,  3,  20, 3,  0,  2,  0,  1, 8, 19, 8,  18, 8,  1, 7, 1,  6, 1, 5, 13, 15,
                13, 14, 17, 14, 16, 14, 13, 5, 1, 4,  10, 12, 10, 4, 9, 11, 9, 4, 1, 0};
  EXPECT_EQ(solver.find_mst_solution(0, 0), expect2);

  VI expect3 = {5, 13, 15, 13, 14, 17, 14, 16, 14, 13, 5, 1, 8, 19, 8, 18, 8, 1, 7, 1, 6,
                1, 4,  10, 12, 10, 4,  9,  11, 9,  4,  1, 0, 3, 20, 3, 0,  2, 0, 1, 5};
  EXPECT_EQ(solver.find_mst_solution(5, 5), expect3);

  VI expect4 = {17, 14, 16, 14, 13, 15, 13, 5, 1, 8, 19, 8, 18, 8, 1, 7,  1,
                6,  1,  0,  3,  20, 3,  0,  2, 0, 1, 4,  9, 11, 9, 4, 10, 12};
  EXPECT_EQ(solver.find_mst_solution(17, 12), expect4);
}

TEST_F(BaseSolverTest, FindNearestTerminals) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  EXPECT_EQ(solver.find_nearest_terminals(4, 0, 0), VI({0, 3, 1, 4}));
  EXPECT_EQ(solver.find_nearest_terminals(3, 0, 0), VI({0, 3, 1}));
  EXPECT_EQ(solver.find_nearest_terminals(2, 0, 0), VI({0, 3, 1}));
  EXPECT_EQ(solver.find_nearest_terminals(1, 0, 0), VI({0, 3}));
  EXPECT_EQ(solver.find_nearest_terminals(2, 0, 4), VI({0, 4, 3}));
  EXPECT_EQ(solver.find_nearest_terminals(4, 2, 1), VI({2, 1}));
}

TEST_F(BaseSolverTest, FindSteinerTreeSolution) {
  auto g = load_pace("src/test/resources/instances/001_tiny.gr").graph;
  auto solver = DummySolver(g);
  solver.find_steiner_tree_solution(3, 2, 0);

  EXPECT_EQ(solver.get_solution(), VI({2, 0, 1, 0}));
  EXPECT_EQ(solver.get_solution_weight(), 12);
  EXPECT_EQ(solver.get_solution_colors().to_vector(), VI({0, 1, 2, 3}));

  solver.clean();
  solver.find_steiner_tree_solution(4, 0, 0);

  EXPECT_EQ(solver.get_solution(), VI({0, 1, 0, 3, 4, 3, 0}));
  EXPECT_EQ(solver.get_solution_weight(), 16);
  EXPECT_EQ(solver.get_solution_colors().to_vector(), VI({0, 1, 2, 3}));
}
