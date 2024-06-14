#pragma once

#include "ds/graph/Graph.hpp"
#include "geometry/Point.hpp"
#include "util/Random.hpp"

namespace algorithm {
namespace clustering {

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
    std::vector<int> const& initial_centers = {}                 //
);

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
);

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
std::unordered_map<int, std::vector<int>> k_center_assign_points(std::vector<int> const& centers, util::Random& rand,
                                                                 std::vector<int> const& points,
                                                                 std::unordered_map<int, ::geometry::Point> const& poi_position);

/**
 * @brief Recalculates the centers of the clusters.
 *
 * @param clusters clusters
 * @param poi_position positions of the points
 *
 * @return std::vector<int> centers
 */
std::vector<int> k_center_recalculate_centers(std::vector<int> const& centers, util::Random& rand, std::vector<int> const& points,
                                              std::unordered_map<int, ::geometry::Point> const& poi_position);

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
//                                                          std::unordered_map<int, ::geometry::Point> const& poi_position);

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
//     std::unordered_map<int, ::geometry::Point> const& poi_position);
}  // namespace clustering
}  // namespace algorithm
