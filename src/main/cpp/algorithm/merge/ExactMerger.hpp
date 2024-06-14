#pragma once

#include "algorithm/ilp/ILPRunner.hpp"
#include "algorithm/merge/BaseMerger.hpp"
#include "util/logger.hpp"

namespace algorithm {
namespace merge {
class ExactMerger : public BaseMerger {
 private:
  int num_threads_;
  uint32_t seed_;

 public:
  /**
   * @brief Constructs a new ExactMerger instance.
   *
   * @param num_threads number of threads
   * @param seed random seed
   */
  ExactMerger(int num_threads, uint32_t seed) : BaseMerger(), num_threads_(num_threads), seed_(seed) {}

  /**
   * @brief Merges and simplifies all walks using ILP.
   *
   * @param graph graph
   * @param walks input walks
   * @return std::vector<ds::graph::Graph::Vertex> merged walk
   */
  std::vector<ds::graph::Graph::Vertex> merge(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const override {
    validate_walks(graph, walks);

    Walk ret;

#if GUROBI_ON
    // create a corresponding multigraph
    auto g = create_multigraph(graph, walks);
    int k = g.graph.number_of_vertices() - 1;
    auto one_way_edges = g.get_one_way_edges();
    log_debug("ExactMerger: Constructed ILP instance: n=%lu, m=%lu, k=%d, #one-way-edges=%lu",
              g.graph.number_of_vertices(), g.graph.number_of_edges(), k, one_way_edges.size());

    // run ILP
    int time_limit = 0;
    int formulation_version = 2;
    auto runner = algorithm::ilp::ILPRunner(g.graph, k, g.source, g.source, false, num_threads_, seed_, time_limit,
                                            false, formulation_version, one_way_edges);

    runner.run();
    ret = runner.get_solution();
#else
    throw std::invalid_argument("ExactMerger: Gurobi is disabled.");
#endif
    return ret;
  }
};
}  // namespace merge
}  // namespace algorithm
