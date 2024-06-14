#include "clustering.hpp"

namespace algorithm {
namespace clustering {

namespace impl {

/**
 * @brief A data structure to keep the maximum value and its corresponding key,
 * with random tie-breaking.
 */
class Maximizer {
 private:
  util::Random& rand_;
  int best_key_;
  double best_value_;
  int num_ties_;

 public:
  Maximizer(util::Random& rand) : rand_(rand) { clear(); }

  void clear() {
    best_key_ = -1;
    best_value_ = -INFINITY;
    num_ties_ = 0;
  }

  void push(int key, double value) {
    if (value > best_value_) {
      best_value_ = value;
      best_key_ = key;
      num_ties_ = 1;
    } else if (value == best_value_) {
      // Randomly choose one of the tied indices with the same probability
      // (replace with probability 1/t, where t is the current number of ties).
      if (rand_.random() < 1.0 / ++num_ties_) best_key_ = key;
    }
  }

  int top() const { return best_key_; }
};
}  // namespace impl

/**
 * @brief Chooses k centers from a set of points in 3D.
 * Implementation is based on Gonzalez (1985). Ties are broken randomly.
 *
 * @param points set of points and their positions
 * @param location position of all points
 * @param k number of centers to choose in addition to initial centers
 * @param rand util::Random instance
 * @param initial_centers initial centers
 * @return std::vector<int> ordered indices of the chosen points
 *
 * Time complexity: O((n_0+k)n), where n=|points| and n_0=|initial centers|.
 */
std::vector<int> k_center_greedy(                                //
    std::vector<int> const& points,                              //
    std::unordered_map<int, ::geometry::Point> const& location,  //
    std::size_t k,                                               //
    util::Random& rand,                                          //
    std::vector<int> const& initial_centers                      //
) {
  /**
   * @brief Keeps track of remaining points.
   * @note std::set<> is faster than std::unordered_set<> here.
   */
  std::set<int> remain(points.begin(), points.end());
  for (auto x : initial_centers) {
    if (util::contains(remain, x)) remain.erase(x);
  }

  std::map<int, double> dist;  // distance from the current set of centers
  std::vector<int> ret;        // stores result in order
  impl::Maximizer maximizer(rand);

  // Step 1: Compute the distance from the initial centers
  for (auto y : remain) {
    double d = INFINITY;
    for (auto x : initial_centers) { d = std::min(d, geometry::Point::distance(location.at(x), location.at(y))); }
    maximizer.push(y, dist[y] = d);
  }

  // Step 2: Iteratively pick the farthest point as a new center.
  while (!remain.empty() && ret.size() < k) {
    auto x = maximizer.top();
    assert(util::contains(remain, x));
    assert(!util::contains(initial_centers, x));  // should not be in the initial centers

    ret.push_back(x);
    remain.erase(x);

    // Update information for remaining points and pick the best one.
    maximizer.clear();
    for (auto y : remain) {
      maximizer.push(y, dist[y] = std::min(dist[y], geometry::Point::distance(location.at(x), location.at(y))));
    }
  }
  return ret;
}

/**
 * @brief Assigns points to the nearest center.
 * Ties are broken randomly.
 *
 * @param points points to partition
 * @param centers centers
 * @param location position of all points
 * @param minimum_part_size minimum size of each part
 * @param rand util::Random instance
 * @return std::vector<std::vector<int>> list of parts of points
 */
std::vector<std::vector<int>> partition_into_points(             //
    std::vector<int> const& points,                              //
    std::vector<int> const& centers,                             //
    std::unordered_map<int, ::geometry::Point> const& location,  //
    std::size_t minimum_part_size,                               //
    util::Random& rand                                           //
) {
  std::size_t k = centers.size();
  if (points.size() < k * minimum_part_size) throw std::invalid_argument("not enough number of points");

  std::vector<std::vector<int>> ret(k);
  std::unordered_set<int> consumed;

  // Step 1: Assign centers to themselves.
  for (std::size_t c_idx = 0; c_idx < k; ++c_idx) {
    auto c = centers[c_idx];
    ret[c_idx].push_back(c);
    consumed.insert(c);
  }

  // Step 2: Compute distances and sort points for each center.
  using PointInfo = std::tuple<double, double, std::size_t, int>;
  std::vector<PointInfo> sorted_points;

  for (std::size_t c_idx = 0; c_idx < k; ++c_idx) {
    auto c = centers[c_idx];
    for (auto p : points) {
      if (util::contains(consumed, p)) continue;
      auto d = geometry::Point::distance(location.at(p), location.at(c));
      auto rand_value = rand.random() + rand.random() + rand.random();  // for random tie-breaking
      sorted_points.push_back(PointInfo({d, rand_value, c_idx, p}));
    }
  }
  std::sort(sorted_points.begin(), sorted_points.end());  // nearest to farthest

  // Step 3: For each cluster, find `minimum_part_size` nearest neighbors.
  for (auto& [d, r, center_idx, point] : sorted_points) {
    if (util::contains(consumed, point)) continue;              // already used
    if (ret[center_idx].size() >= minimum_part_size) continue;  // already fulfilled

    ret[center_idx].push_back(point);
    consumed.insert(point);
  }

  // Step 4: Assign other points.
  for (auto& [d, r, center_idx, point] : sorted_points) {
    if (consumed.size() >= points.size()) break;    // done
    if (util::contains(consumed, point)) continue;  // already used

    ret[center_idx].push_back(point);
    consumed.insert(point);
  }

  return ret;
}

/**
 * @brief Assigns points to the closest center.
 * Ties are broken randomly.
 *
 * @param centers centers
 * @param points points
 * @param poi_position positions of the points
 *
 * @return std::unordered_map<int, std::vector<int>> clusters
 */
// std::unordered_map<int, std::vector<int>> k_center_assign_points(std::vector<int> const& centers, util::Random& rand,
//                                                                  std::vector<int> const& poi_all,
//                                                                  std::unordered_map<int, ::geometry::Point> const& poi_position) {
//   std::unordered_map<int, std::vector<int>> clusters;
//   for (int poi : poi_all) {
//     double min_distance = std::numeric_limits<double>::infinity();
//     // int closest_center = -1;

//     std::vector<int> tied_points;

//     for (int center : centers) {
//       double distance = geometry::Point::distance(poi_position.at(poi), poi_position.at(center));
//       if (distance < min_distance) {
//         min_distance = distance;
//         // closest_center = center;
//         tied_points.clear();
//         tied_points.push_back(center);
//       }
//       if (distance == min_distance) { tied_points.push_back(center); }
//     }

//     int index = 0;

//     if (tied_points.size() > 1) { index = rand.randint(0, static_cast<int>(tied_points.size()) - 1); }

//     int center = tied_points[index];

//     clusters[center].push_back(poi);
//   }

//   return clusters;
// }

/**
 * @brief Recalculates the centers of the clusters.
 *
 * @param clusters clusters
 * @param poi_position positions of the points
 *
 * @return std::vector<int> centers
 */
std::vector<int> k_center_recalculate_centers(std::vector<int> const& centers, util::Random& rand, std::vector<int> const& poi_all,
                                              std::unordered_map<int, ::geometry::Point> const& poi_position) {
  std::vector<int> recalculated_centers = centers;

  while (true) {
    bool updated = false;

    auto clusters = partition_into_points(poi_all, recalculated_centers, poi_position, 0, rand);
    recalculated_centers.clear();

    for (std::size_t i = 0; i < clusters.size(); ++i) {
      auto current_center = recalculated_centers[i];
      auto& pois = clusters[i];

      geometry::Point new_center_position(0.0, 0.0, 0.0);
      for (int poi : pois) { new_center_position += poi_position.at(poi); }
      new_center_position /= static_cast<double>(pois.size());

      double min_distance = std::numeric_limits<double>::infinity();
      int closest_poi = -1;

      std::vector<int> tied_points;

      for (int poi : pois) {
        double distance = geometry::Point::distance(poi_position.at(poi), new_center_position);
        if (distance < min_distance) {
          min_distance = distance;
          closest_poi = poi;
          tied_points.clear();
          tied_points.push_back(poi);
        }
        if (distance == min_distance) { tied_points.push_back(poi); }
      }

      int index = 0;

      if (tied_points.size() > 1) { index = rand.randint(0, static_cast<int>(tied_points.size()) - 1); }

      closest_poi = tied_points[index];

      recalculated_centers.push_back(closest_poi);

      if (closest_poi != current_center) { updated = true; }
    }

    if (!updated) break;
  }
  return recalculated_centers;
}

/**
 * @brief Balance the clusters, so each center has equal number of points assigned to it.
 *
 * @param clusters clusters
 * @param poi_position positions of the points
 *
 * @return std::unordered_map<int, std::vector<int>> clusters
 */
// std::unordered_map<int, std::vector<int>> k_center_balance(std::unordered_map<int, std::vector<int>> const& clusters,
//                                                            util::Random& rand,
//                                                            std::unordered_map<int, ::geometry::Point> const& poi_position) {
//   // for each center, assign the poi closest to it, one at a time and remove it from the list
//   // random break tie

//   std::unordered_map<int, std::vector<int>> new_clusters;
//   std::vector<int> all_pois;

//   // collect all pois
//   for (auto& cluster : clusters) {
//     for (int poi : cluster.second) { all_pois.push_back(poi); }
//   }

//   while (all_pois.size() > 0) {
//     for (auto& cluster : clusters) {
//       int center = cluster.first;

//       std::vector<int> tied_pois;
//       double min_distance = std::numeric_limits<double>::infinity();

//       for (int poi : all_pois) {
//         double distance = geometry::Point::distance(poi_position.at(center), poi_position.at(poi));
//         if (distance < min_distance) {
//           min_distance = distance;
//           tied_pois.clear();
//           tied_pois.push_back(poi);
//         } else if (min_distance == distance) {
//           tied_pois.push_back(poi);
//         }
//       }

//       int index = 0;

//       if (tied_pois.size() > 1) { index = rand.randint(0, static_cast<int>(tied_pois.size()) - 1); }

//       int poi = tied_pois[index];
//       new_clusters[center].push_back(poi);
//       all_pois.erase(std::remove(all_pois.begin(), all_pois.end(), poi), all_pois.end());
//     }
//   }
//   return new_clusters;
// }

/**
 * @brief Recalculates the centers of the clusters.
 *
 * @param clusters clusters
 * @param poi_position positions of the points
 *
 * @return std::vector<geometry::Point> centers
 */
// std::vector<geometry::Point> k_means_recalculate_centers(std::vector<geometry::Point> const& centers,
//                                                          util::Random& rand, std::vector<int> const& points,
//                                                          std::unordered_map<int, ::geometry::Point> const& poi_position) {
//   std::vector<geometry::Point> recalculated_centers = centers;

//   while (true) {
//     bool updated = false;

//     std::unordered_map<geometry::Point, std::vector<int>, geometry::Point::PointHash, geometry::Point::PointEqual> clusters;

//     for (int poi : points) {
//       double min_distance = std::numeric_limits<double>::infinity();
//       geometry::Point closest_center;

//       for (const auto& center : recalculated_centers) {
//         double distance = geometry::Point::distance(center, poi_position.at(poi));
//         if (distance < min_distance) {
//           min_distance = distance;
//           closest_center = center;
//         }
//       }

//       clusters[closest_center].push_back(poi);
//     }

//     recalculated_centers.clear();

//     for (const auto& [current_center, pois] : clusters) {
//       geometry::Point new_center_position(0.0, 0.0, 0.0);
//       for (int poi : pois) { new_center_position += poi_position.at(poi); }
//       new_center_position /= static_cast<double>(pois.size());

//       recalculated_centers.push_back(new_center_position);

//       if (new_center_position != current_center) { updated = true; }
//     }

//     if (!updated) break;
//   }
//   return recalculated_centers;
// }

/**
 * @brief Balance the clusters, so each center has equal number of points assigned to it.
 *
 * @param clusters clusters
 * @param poi_position positions of the points
 *
 * @return std::unordered_map<geometry::Point, std::vector<int>, geometry::Point::PointHash, geometry::Point::PointEqual> clusters
 */
// std::unordered_map<geometry::Point, std::vector<int>, geometry::Point::PointHash, geometry::Point::PointEqual> k_means_balance(
//     std::vector<geometry::Point> const& centers, std::vector<int> const& points, util::Random& rand,
//     std::unordered_map<int, ::geometry::Point> const& poi_position) {
//   // for each center, assign the poi closest to it, one at a time and remove it from the list

//   std::unordered_map<geometry::Point, std::vector<int>, geometry::Point::PointHash, geometry::Point::PointEqual>
//   new_clusters; std::vector<int> all_pois;

//   // collect all pois
//   for (int poi : points) { all_pois.push_back(poi); }

//   while (all_pois.size() > 0) {
//     for (geometry::Point center : centers) {
//       std::vector<int> tied_pois;
//       double min_distance = std::numeric_limits<double>::infinity();

//       for (int poi : all_pois) {
//         double distance = geometry::Point::distance(center, poi_position.at(poi));
//         if (distance < min_distance) {
//           min_distance = distance;
//           tied_pois.clear();
//           tied_pois.push_back(poi);
//         } else if (min_distance == distance) {
//           tied_pois.push_back(poi);
//         }
//       }

//       int index = 0;

//       if (tied_pois.size() > 1) { index = rand.randint(0, static_cast<int>(tied_pois.size()) - 1); }

//       int poi = tied_pois[index];
//       new_clusters[center].push_back(poi);
//       all_pois.erase(std::remove(all_pois.begin(), all_pois.end(), poi), all_pois.end());
//     }
//   }
//   return new_clusters;
// }

}  // namespace clustering
}  // namespace algorithm
