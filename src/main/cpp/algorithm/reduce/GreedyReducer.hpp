#pragma once

#include "algorithm/clustering/clustering.hpp"
#include "algorithm/reduce/BaseReducer.hpp"

namespace algorithm {
namespace reduce {
class GreedyReducer : public BaseReducer {
 private:
  bool random_start_;

 public:
  /**
   * @brief Constructs a new GreedyReducer instance.
   *
   * @param poi_position map from all POIs to their coordinates in R3.
   * @param graph given graph
   * @param source_vertex starting vertex
   * @param partition_algorithm partitioning algorithm
   * @param partition_timing partitioning timing
   * @param random_start start from a random POI if true;
   *                     otherwise, use vertex 0's POIs for the starting set
   */
  GreedyReducer(std::unordered_map<int, ::geometry::Point> const& poi_position, ds::graph::Graph const& graph,
                ds::graph::Graph::Vertex source_vertex, PartitionAlgorithm partition_algorithm,
                PartitionTiming partition_timing, bool random_start)
      : BaseReducer("GreedyReducer", poi_position, graph, source_vertex, partition_algorithm, partition_timing),
        random_start_(random_start) {}

  /**
   * @brief Filters the given POIs to k POIs.
   *
   * @param pois target POIs
   * @param k number of POIs to choose
   * @param rand util::Random instance
   * @return std::vector<POI> list of chosen POIs
   */
  std::vector<POI> filter_pois(std::vector<int> const& pois, int k, util::Random& rand) const override {
    return find_k_center(pois, k, rand, random_start_);
  }
};
}  // namespace reduce
}  // namespace algorithm
