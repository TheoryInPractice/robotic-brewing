#include "readwrite/iris.hpp"

using namespace ds::graph;

namespace readwrite {
ds::ProblemInstance read_iris(std::istream &vertex, std::istream &edge) {
  std::vector<std::pair<Graph::Vertex, std::vector<Graph::Color>>> vertices;
  std::vector<std::pair<Graph::Edge, Graph::Weight>> edges;

  for (std::string line; std::getline(vertex, line);) {
    if (line.empty()) continue;
    auto tokens = util::split(line);

    auto v = std::stoi(tokens[0]);
    std::vector<Graph::Color> colors;
    for (std::size_t i = 3; i < tokens.size(); ++i) {
      if (!tokens[i].empty()) { colors.push_back(std::stoi(tokens[i])); }
    }
    vertices.push_back({v, colors});
  }

  for (std::string line; std::getline(edge, line);) {
    if (line.empty()) continue;
    auto tokens = util::split(line);

    auto u = std::stoi(tokens[0]);
    auto v = std::stoi(tokens[1]);

    // if invlid, skip
    bool valid = std::stoi(tokens[3]) > 0;
    if (!valid) continue;

    auto w = std::stod(tokens[6]);

    edges.push_back({{u, v}, w});
  }
  ds::graph::Graph G{vertices, edges};
  ds::ProblemInstance ret = {G};
  return ret;
}

ds::ProblemInstance load_iris(char const *path) {
  std::string vertexPath = util::format("%s_vertex", path);
  std::ifstream v(vertexPath);
  if (v.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", vertexPath.c_str()));

  std::string edgePath = util::format("%s_edge", path);
  std::ifstream e(edgePath);
  if (e.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", edgePath.c_str()));
  return read_iris(v, e);
}
}  // namespace readwrite
