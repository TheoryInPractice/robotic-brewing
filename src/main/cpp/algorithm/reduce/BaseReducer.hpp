#pragma once

#include "algorithm/clustering/clustering.hpp"
#include "ds/graph/Graph.hpp"
#include "geometry/Point.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace algorithm {
namespace reduce {

/** Partitioning algorithm. */
enum PartitionAlgorithm { Ordered, Geometric };

/** Partitioning timing. */
enum PartitionTiming { BeforeReduction, AfterReduction };

class BaseReducer {
 protected:
  using POI = int;

 private:
  char const* reducer_name_;
  std::unordered_map<POI, geometry::Point> const& poi_position_;
  ds::graph::Graph const& graph_;
  ds::graph::Graph::Vertex source_vertex_;
  PartitionAlgorithm partition_algorithm_;
  PartitionTiming partition_timing_;
  ds::graph::Graph::ColorSet poi_all_;
  ds::graph::Graph::ColorSet poi_source_;
  ds::graph::Graph::ColorSet poi_target_;

 public:
  /**
   * @brief Constructs a new BaseReducer instance.
   */
  BaseReducer(char const* reducer_name, std::unordered_map<int, geometry::Point> const& poi_position,
              ds::graph::Graph const& graph, ds::graph::Graph::Vertex source_vertex,
              PartitionAlgorithm partition_algorithm, PartitionTiming partition_timing)
      : reducer_name_(reducer_name),
        poi_position_(poi_position),
        graph_(graph),
        source_vertex_(source_vertex),
        partition_algorithm_(partition_algorithm),
        partition_timing_(partition_timing) {
    // Compute POI sets.
    poi_all_ = graph_.get_colors();                   // all POIs present in the graph
    poi_source_ = graph_.get_colors(source_vertex_);  // source vertex's POIs

    // Exclude source POIs from target
    poi_target_ = poi_all_ - poi_source_;
  }

  virtual ~BaseReducer() = default;

  std::unordered_map<POI, geometry::Point> const& get_location() const { return poi_position_; }

  /**
   * @brief Finds a reduced set of POIs.
   *
   * @param k number of reduced POIs for each cluster
   * @param splits number of clusters
   * @param rand util::Random instance
   * @return std::vector<int> reduced PIOs
   */
  std::vector<std::vector<POI>> reduce(int k, int splits, util::Random& rand) const {
    if (splits <= 0) throw std::invalid_argument("splits must be positive");
    k = std::min(k, static_cast<int>(poi_target_.size()));  // k cannot exceed the number of targets

    util::Timer timer;
    log_info("%s: started: K=%d, #splits=%d", reducer_name_, k, splits);

    std::vector<std::vector<POI>> ret;

    switch (partition_timing_) {
      case PartitionTiming::BeforeReduction: {
        // Cluster target POIs into `splits` groups of size at least k.
        log_info("%s: pre-partitioning into %d parts: min_size=%d", reducer_name_, splits, k);
        auto clusters = partition_pois(poi_target_.to_vector(), splits, k, rand);

        // For each cluster, choose k POIs.
        for (auto& cluster : clusters) {
          log_debug("%s: filtering POIs: cluster_size=%lu, reduced_size=%d", reducer_name_, cluster.size(), k);
          ret.push_back(filter_pois(cluster, k, rand));
        }
        break;
      }
      case PartitionTiming::AfterReduction: {
        // Choose k * `splits` POIs first.
        log_debug("%s: filtering POIs: original_size=%lu, reduced_size=%d", reducer_name_, poi_target_.size(), k * splits);
        auto filtered = filter_pois(poi_target_.to_vector(), k * splits, rand);

        // Cluster filtered POIs into `splits` groups of size k.
        log_info("%s: post-partitioning into %d parts: min_size=%d", reducer_name_, splits, k);
        ret = partition_pois(filtered, splits, k, rand);
        break;
      }
      default: throw std::runtime_error("never happens");
    }

    log_info("%s: finished: K=%d, #splits=%d, elapsed=%.3fs", reducer_name_, k, splits, timer.stop());
    return ret;
  }

  /**
   * @brief Filters the given POIs to k POIs.
   *
   * @param pois target POIs
   * @param k number of POIs to choose
   * @param rand util::Random instance
   * @return std::vector<POI> list of chosen POIs
   */
  virtual std::vector<POI> filter_pois(std::vector<int> const& pois, int k, util::Random& rand) const = 0;

  /**
   * @brief Finds a subset of POIs using greedy k-center.
   *
   * @param pois target POIs
   * @param k number of POIs to choose
   * @param rand util::Random instance
   * @param random_start start from a random POI if true;
   *                     otherwise, use vertex 0's POIs for the starting set
   * @return std::vector<POI> list of chosen POIs
   */
  std::vector<POI> find_k_center(std::vector<int> const& pois, int k, util::Random& rand, bool random_start) const {
    // set initial centers
    std::vector<POI> initial_centers = random_start ? std::vector<POI>() : poi_source_.to_vector();

    // Run k-Center.
    std::vector<POI> ret = clustering::k_center_greedy(pois, get_location(), k, rand, initial_centers);
    return ret;
  }

  //============================================================================
  //    Partitioning Algoritihms
  //============================================================================
  /**
   * @brief Partitions given POIs into parts.
   *
   * @param pois input POIs
   * @param splits number of parts
   * @param min_size minimum size of each partition
   * @param rand util::Random instance
   * @return std::vector<std::vector<POI>> partition of POIs
   */
  std::vector<std::vector<POI>> partition_pois(  //
      std::vector<POI> const& pois, int splits, std::size_t min_size, util::Random& rand) const {
    if (pois.size() < splits * min_size) throw std::invalid_argument("not enough number of POIs");
    if (splits <= 1) return {pois};

    switch (partition_algorithm_) {
      case PartitionAlgorithm::Ordered: return partition_pois_ordered(pois, splits);
      case PartitionAlgorithm::Geometric: return partition_pois_geometric(pois, splits, min_size, rand);
      default: throw std::runtime_error("never happens");
    }
  }

 private:
  /**
   * @brief Returns clusters of POIs.
   *
   * @note This implements `geom-part` in the paper.
   *
   * @param pois input POIs
   * @param splits number of clusters
   * @param min_size minimum size of each cluster
   * @param rand util::Random instance
   * @return std::vector<std::vector<POI>> clusters of POIs
   */
  std::vector<std::vector<POI>> partition_pois_geometric(  //
      std::vector<POI> const& pois, int splits, std::size_t min_size, util::Random& rand) const {
    auto centers = clustering::k_center_greedy(pois, get_location(), splits, rand, poi_source_.to_vector());

    // Partition target POIs into `splits` parts. Each part has size at least k.
    auto ret = clustering::partition_into_points(pois, centers, get_location(), min_size, rand);
    return ret;
  };

  /**
   * @brief Partitions POIs into parts based on the given order.
   *
   * @param pois input POIs
   * @param splits number of parts
   * @return std::vector<std::vector<POI>> partition of POIs
   */
  std::vector<std::vector<POI>> partition_pois_ordered(std::vector<POI> const& pois, int splits) const {
    std::size_t n = pois.size();
    std::vector<std::vector<POI>> ret;

    // Sequential partitioning.
    for (std::size_t i = 0; static_cast<int>(ret.size()) < splits;) {
      assert(i <= n);
      int remain = splits - ret.size();
      int k = (n - i + remain - 1) / remain;
      ret.push_back(std::vector<POI>(pois.begin() + i, pois.begin() + i + k));
      i += k;
    }
    assert(static_cast<int>(ret.size()) == splits);
    return ret;
  };
};
}  // namespace reduce
}  // namespace algorithm
