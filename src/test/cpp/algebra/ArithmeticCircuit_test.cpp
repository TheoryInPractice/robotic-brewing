#include <gtest/gtest.h>

#include "algebra/ArithmeticCircuit.hpp"
#include "ds/set/SortedVectorSet.hpp"

using namespace std;
typedef std::vector<int> VI;
typedef std::pair<int, int> II;
typedef std::vector<II> VII;
typedef std::map<II, int> M;

//
// ArithmeticCircuitTest
//
TEST(ArithmeticCircuitTest, Evaluate) {
  auto c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(3);

  EXPECT_EQ(c.number_of_variables(), 3);
  EXPECT_EQ(c.number_of_nodes(), 3);

  EXPECT_EQ(c.add_addition_gate({0, 1, 2}), 3);
  EXPECT_EQ(c.get_output_nodes(), VI({3}));
  EXPECT_EQ(c.evaluate(VI({1, 10, 100})), VI({111}));
  EXPECT_EQ(c.evaluate(VI({1, 10, 100}), M({{{0, 3}, 2}, {{1, 3}, 4}, {{2, 3}, 8}})), VI({842}));

  c.reset_output_node(3);
  EXPECT_EQ(c.add_multiplication_gate({3, 2, 1}), 4);
  EXPECT_EQ(c.get_output_nodes(), VI({4}));
  EXPECT_EQ(c.evaluate(VI({1, 10, 100})), VI({111000}));
  EXPECT_EQ(c.evaluate(VI({1, 10, 100}), M({{{0, 3}, 2}, {{1, 3}, 4}, {{2, 3}, 8}})), VI({842000}));

  EXPECT_EQ(c.add_addition_gate({0, 2}, false), 5);
  EXPECT_EQ(c.get_output_nodes(), VI({4}));
  c.add_edge(5, 4);
  EXPECT_EQ(c.evaluate(VI({1, 10, 100})), VI({11211000}));

  auto fp = M({{{0, 3}, 2}, {{1, 3}, 4}, {{2, 3}, 8}, {{0, 5}, 3}, {{2, 5}, 9}});
  EXPECT_EQ(c.fingerprint_edges(), VII({{0, 3}, {1, 3}, {2, 3}, {0, 5}, {2, 5}}));
  EXPECT_EQ(c.evaluate(VI({1, 10, 100}), fp), VI({760326000}));

  // check ordering
  auto ordering = c.topological_ordering(false);
  auto ordering_rev = c.topological_ordering(true);
  EXPECT_EQ(ordering, VI({0, 1, 2, 3, 5, 4}));
  EXPECT_EQ(ordering_rev, VI({4, 3, 5, 1, 0, 2}));
}

TEST(ArithmeticCircuitTest, RemoveNode) {
  auto c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(3);
  EXPECT_EQ(c.add_addition_gate({0, 1}, false), 3);
  EXPECT_EQ(c.add_addition_gate({1, 2}, false), 4);
  EXPECT_EQ(c.add_multiplication_gate({2, 3, 4}, true), 5);

  EXPECT_EQ(c.number_of_nodes(), 6);
  EXPECT_EQ(c.number_of_edges(), 7);
  EXPECT_TRUE(c.has_edge(1, 3));
  c.remove_edge(1, 3);

  EXPECT_EQ(c.number_of_nodes(), 6);
  EXPECT_EQ(c.number_of_edges(), 6);
  EXPECT_FALSE(c.has_edge(1, 3));

  EXPECT_TRUE(c.has_node(4));
  c.remove_node(4);

  EXPECT_EQ(c.number_of_nodes(), 5);
  EXPECT_EQ(c.number_of_edges(), 3);
  EXPECT_FALSE(c.has_node(4));
  EXPECT_EQ(c.get_output_nodes(), VI({5}));

  EXPECT_EQ(c.evaluate(VI({2, 3, 5})), VI({10}));
  EXPECT_EQ(c.topological_ordering(false), VI({0, 1, 2, 3, 5}));
  EXPECT_EQ(c.topological_ordering(true), VI({5, 2, 3, 0}));

  // reuse removed node
  EXPECT_EQ(c.add_addition_gate({1, 5}, true), 4);
  c.reset_output_node(5);
  EXPECT_EQ(c.get_output_nodes(), VI({4}));
  EXPECT_EQ(c.number_of_nodes(), 6);
  EXPECT_EQ(c.number_of_edges(), 5);
  EXPECT_TRUE(c.has_node(4));
  EXPECT_EQ(c.evaluate(VI({2, 3, 5})), VI({13}));
  EXPECT_EQ(c.topological_ordering(false), VI({0, 1, 2, 3, 5, 4}));
  EXPECT_EQ(c.topological_ordering(true), VI({4, 1, 5, 2, 3, 0}));

  // Case where node id > number of nodes
  auto c2 = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(2);
  EXPECT_EQ(c2.add_addition_gate({0, 1}), 2);
  EXPECT_EQ(c2.add_multiplication_gate({2, 1}), 3);
  EXPECT_EQ(c2.add_addition_gate({3, 1}), 4);
  EXPECT_EQ(c2.add_multiplication_gate({4, 1}), 5);
  EXPECT_EQ(c2.add_addition_gate({5, 1}), 6);
  c2.remove_node(5);
  c2.remove_node(2);
  c2.remove_node(4);
  c2.remove_node(3);
  EXPECT_EQ(c2.number_of_nodes(), 3);
  EXPECT_EQ(c2.number_of_edges(), 1);
  EXPECT_EQ(c2.evaluate(VI({2, 3})), VI({3}));
}

TEST(ArithmeticCircuitTest, RemoveAndCleanNode) {
  auto c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(2);
  c.add_addition_gate(VI({0}), false);
  c.add_addition_gate(VI({1}), true);
  c.add_addition_gate(VI({1}), false);
  c.add_multiplication_gate(VI({2, 3, 4}), false);
  c.add_addition_gate(VI({5}), false);
  c.add_addition_gate(VI({4}), false);
  c.add_addition_gate(VI({5}), false);
  c.add_edge(8, 6);
  c.add_edge(8, 7);
  c.add_multiplication_gate(VI({6, 7}), false);
  c.add_multiplication_gate(VI({6}), false);
  c.add_multiplication_gate(VI({10}), true);
  c.add_multiplication_gate(VI({10, 9}), true);

  EXPECT_EQ(c.number_of_nodes(), 13);
  EXPECT_EQ(c.number_of_edges(), 17);

  c.remove_and_clean_node(5);
  EXPECT_TRUE(c.has_node(0));
  EXPECT_TRUE(c.has_node(1));
  EXPECT_FALSE(c.has_node(2));
  EXPECT_TRUE(c.has_node(3));
  EXPECT_TRUE(c.has_node(4));
  EXPECT_FALSE(c.has_node(5));
  EXPECT_FALSE(c.has_node(6));
  EXPECT_TRUE(c.has_node(7));
  EXPECT_FALSE(c.has_node(8));
  EXPECT_TRUE(c.has_node(9));
  EXPECT_FALSE(c.has_node(10));
  EXPECT_TRUE(c.has_node(11));
  EXPECT_TRUE(c.has_node(12));
  EXPECT_EQ(c.number_of_nodes(), 8);
  EXPECT_EQ(c.number_of_edges(), 5);
}

TEST(ArithmeticCircuitTest, Degree) {
  auto c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(4);
  c.add_addition_gate(VI({0, 1, 2}));
  c.add_addition_gate(VI({1, 2, 3}));
  c.add_addition_gate(VI({4, 5}));
  c.add_addition_gate(VI({4, 5, 6}));
  EXPECT_EQ(c.degree(), 1);

  c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(4);
  c.add_addition_gate(VI({0, 1}));
  c.add_addition_gate(VI({2, 3}));
  c.add_multiplication_gate(VI({4, 5}));
  c.add_multiplication_gate(VI({4, 5}));
  c.add_multiplication_gate(VI({6, 7}));
  EXPECT_EQ(c.degree(), 4);

  c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(3);
  c.add_multiplication_gate(VI({0, 1, 2}));
  c.add_multiplication_gate(VI({0, 1, 2}));
  c.add_multiplication_gate(VI({0, 1, 2}));
  c.add_multiplication_gate(VI({3, 4, 5}));

  EXPECT_EQ(c.get_output_nodes(), VI({3, 4, 5, 6}));
  EXPECT_EQ(c.degree(), 9);
}

TEST(ArithmeticCircuitTest, IsOutputReachable) {
  auto c = algebra::ArithmeticCircuit<ds::set::SortedVectorSet>(3);
  EXPECT_EQ(c.add_addition_gate(VI({0, 1}), false), 3);
  EXPECT_EQ(c.add_addition_gate(VI({1, 2}), false), 4);
  EXPECT_EQ(c.add_multiplication_gate(VI({3}), true), 5);
  EXPECT_EQ(c.add_multiplication_gate(VI({3, 4}), true), 6);
  EXPECT_EQ(c.add_addition_gate(VI({5, 6}), true), 7);
  c.remove_edge(0, 3);
  c.remove_edge(3, 5);

  EXPECT_EQ(c.get_output_nodes(), VI({5, 6, 7}));
  EXPECT_FALSE(c.is_output_reachable());
  c.reset_output_node(5);
  EXPECT_EQ(c.get_output_nodes(), VI({6, 7}));
  EXPECT_TRUE(c.is_output_reachable());
  c.reset_output_node(7);
  EXPECT_TRUE(c.is_output_reachable());

  EXPECT_EQ(c.add_addition_gate(VI({0, 5}), true), 8);
  EXPECT_TRUE(c.is_output_reachable());
  c.remove_edge(0, 8);
  EXPECT_FALSE(c.is_output_reachable());
}
