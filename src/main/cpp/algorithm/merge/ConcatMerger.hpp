#pragma once

#include "algorithm/merge/BaseMerger.hpp"

namespace algorithm {
namespace merge {
class ConcatMerger : public BaseMerger {
 public:
  /**
   * @brief Concatenates all walks in the input order.
   *
   * @param graph graph
   * @param walks input walks
   * @return std::vector<ds::graph::Graph::Vertex> merged walk
   */
  std::vector<ds::graph::Graph::Vertex> merge(ds::graph::Graph const& graph, std::vector<Walk> const& walks) const override {
    validate_walks(graph, walks);

    Walk ret = walks[0];
    for (std::size_t i = 1; i < walks.size(); ++i) {
      for (std::size_t j = 1; j < walks[i].size(); ++j) ret.push_back(walks[i][j]);
    }
    return ret;
  }
};
}  // namespace merge
}  // namespace algorithm
