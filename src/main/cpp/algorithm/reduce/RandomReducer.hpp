#pragma once

#include "algorithm/reduce/BaseReducer.hpp"

namespace algorithm {
namespace reduce {
class RandomReducer : public BaseReducer {
 public:
  /**
   * @brief Constructs a new RandomReducer instance.
   *
   * @param poi_position map from all POIs to their coordinates in R3.
   * @param graph given graph
   * @param source_vertex starting vertex
   * @param partition_algorithm partitioning algorithm
   * @param partition_timing partitioning timing
   */
  RandomReducer(std::unordered_map<int, ::geometry::Point> const& poi_position, ds::graph::Graph const& graph,
                ds::graph::Graph::Vertex source_vertex, PartitionAlgorithm partition_algorithm, PartitionTiming partition_timing)
      : BaseReducer("RandomReducer", poi_position, graph, source_vertex, partition_algorithm, partition_timing) {}

  /**
   * @brief Filters the given POIs to k POIs with uniform random sampling.
   *
   * @param pois target POIs
   * @param k number of POIs to choose
   * @param rand util::Random instance
   * @return std::vector<POI> list of chosen POIs
   */
  std::vector<POI> filter_pois(std::vector<int> const& pois, int k, util::Random& rand) const override {
    auto ret = rand.sample(pois, k);
    return ret;
  }
};
}  // namespace reduce
}  // namespace algorithm
