#pragma once

#include <climits>
#include <unordered_map>

#include "algorithm/clustering/clustering.hpp"
#include "ds/graph/Graph.hpp"
#include <algorithm/reduce/BaseReducer.hpp>
#include <geometry/Point.hpp>
#include <util/Random.hpp>
#include <util/Timer.hpp>
#include <util/logger.hpp>
#include <util/util.hpp>

// kcenter reducer
// 1. Select the first center as the densest area (mean of all pois)

// 2. Select k-1 new centers based on the maximum minimum distance from the set of centers S

// 3. Repeat until convergence of centers:

// 3.1. For each poi, assign it to the closest center

// 3.1. Recalculate the center of each cluster

// 4. Clustre the centers into splits

// 5. Return the centers ordered by split first, density second

namespace algorithm {
namespace reduce {
class KCenterReducer : public BaseReducer {
 private:
  bool random_start_;

 public:
  /**
   * @brief Constructs a new KCenterReducer instance.
   *
   * @param poi_position map from all POIs to their coordinates in R3.
   * @param graph given graph
   * @param source_vertex starting vertex
   * @param partition_algorithm partitioning algorithm
   * @param partition_timing partitioning timing
   * @param random_start start from a random POI if true;
   *                     otherwise, use vertex 0's POIs for the starting set
   */
  KCenterReducer(std::unordered_map<int, ::geometry::Point> const& poi_position, ds::graph::Graph const& graph,
                 ds::graph::Graph::Vertex source_vertex, PartitionAlgorithm partition_algorithm,
                 PartitionTiming partition_timing, bool random_start)
      : BaseReducer("KCenterReducer", poi_position, graph, source_vertex, partition_algorithm, partition_timing),
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
    // Run k-Center.
    auto centers = find_k_center(pois, k, rand, random_start_);

    // Run k-Means.
    auto ret = clustering::k_center_recalculate_centers(centers, rand, pois, get_location());
    return ret;
  }
};
}  // namespace reduce
}  // namespace algorithm
