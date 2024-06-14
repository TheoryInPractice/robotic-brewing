#pragma once

#include "ds/graph/Graph.hpp"

namespace ds {
/**
 * @brief Represents one instance for Inspection Planning.
 */
struct ProblemInstance {
  graph::Graph graph;

  /** Source vertrex. (-1: undefined) */
  graph::Graph::Vertex s = -1;

  /** Destination vertrex. (-1: undefined) */
  graph::Graph::Vertex t = -1;

  /** Number of colors to collect. (-1: undefined) */
  int k = -1;
};
}  // namespace ds
