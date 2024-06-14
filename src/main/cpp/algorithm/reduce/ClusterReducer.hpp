#pragma once

#include "ds/graph/Graph.hpp"
#include <algorithm/reduce/BaseReducer.hpp>
#include <climits>
#include <ds/queue/AdaptivePriorityQueue.hpp>
#include <ds/set/FastSet.hpp>
#include <geometry/Point.hpp>
#include <unordered_map>
#include <util/Random.hpp>
#include <util/Timer.hpp>
#include <util/logger.hpp>
#include <util/util.hpp>

namespace algorithm {
namespace reduce {
class ClusterReducer : public BaseReducer {
 private:
  bool random_start_;
  int scale_;

 public:
  /**
   * @brief Constructs a new ClusterReducer instance.
   *
   * @param poi_position map from all POIs to their coordinates in R3.
   * @param graph given graph
   * @param source_vertex starting vertex
   * @param partition_algorithm partitioning algorithm
   * @param partition_timing partitioning timing
   * @param random_start start from a random POI if true;
   *                     otherwise, use vertex 0's POIs for the starting set
   * @param scale scale
   */
  ClusterReducer(std::unordered_map<int, ::geometry::Point> const& poi_position, ds::graph::Graph const& graph,
                 ds::graph::Graph::Vertex source_vertex, PartitionAlgorithm partition_algorithm,
                 PartitionTiming partition_timing, bool random_start, int scale)
      : BaseReducer("ClusterReducer", poi_position, graph, source_vertex, partition_algorithm, partition_timing),
        random_start_(random_start),
        scale_(scale) {}

  /**
   * @brief Filters the given POIs to k POIs.
   *
   * @param pois target POIs
   * @param k number of POIs to choose
   * @param rand util::Random instance
   * @return std::vector<POI> list of chosen POIs
   */
  std::vector<POI> filter_pois(std::vector<int> const& pois, int k, util::Random& rand) const override {
    // Step 1: Run k-Center to find (scale * k) centers.
    auto centers = find_k_center(pois, scale_ * k, rand, random_start_);

    // Step 2: Assign POIs to the closest center based on distance.
    auto clusters = clustering::partition_into_points(pois, centers, get_location(), 0, rand);

    // Step 3, Return the k best centers with max density (random tie-breaking).
    std::vector<std::tuple<std::size_t, double, POI>> density;
    for (std::size_t i = 0; i < centers.size(); ++i) {
      density.push_back({clusters[i].size(), rand.random() + rand.random() + rand.random(), centers[i]});
    }
    std::sort(density.rbegin(), density.rend());  // largest density to smallest

    std::vector<POI> final_centers;
    for (int i = 0; i < k; ++i) { final_centers.push_back(std::get<2>(density[i])); }
    return final_centers;
  }
};
}  // namespace reduce
}  // namespace algorithm
