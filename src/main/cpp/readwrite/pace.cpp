#include "readwrite/pace.hpp"

using namespace ds::graph;

namespace readwrite {
ds::ProblemInstance read_pace(std::istream &is) {
  std::vector<std::pair<Graph::Vertex, std::vector<Graph::Color>>> vertices;
  std::vector<std::pair<Graph::Edge, Graph::Weight>> edges;
  Graph::Vertex s = -1, t = -1;
  std::size_t n = -1, m = -1, c = -1;
  int i = 0, k = -1;

  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;

    while (i < static_cast<int>(line.size()) && !std::isprint(line[i])) ++i;
    if (line[i] == 'c') continue;  // ignore comments

    auto tokens = util::split(line);
    if (tokens[0] == "p") {  // p-line
      if (tokens.size() < 5) throw std::invalid_argument(util::format("unexpected p-line: %s", line.c_str()));
      if (tokens[1] != "ip") throw std::invalid_argument(util::format("unexpected p-line: %s", line.c_str()));

      n = std::stoi(tokens[2]);
      m = std::stoi(tokens[3]);
      c = std::stoi(tokens[4]);
      if (tokens.size() >= 6) s = std::stoi(tokens[5]);
      if (tokens.size() >= 7) t = std::stoi(tokens[6]);
      if (tokens.size() >= 8) k = std::stoi(tokens[7]);
    } else if (tokens[0] == "v") {  // v-line
      if (tokens.size() < 1) throw std::invalid_argument("unexpected v-line");

      auto v = std::stoi(tokens[1]);
      std::vector<Graph::Color> colors;
      for (std::size_t i = 2; i < tokens.size(); ++i) colors.push_back(std::stoi(tokens[i]));
      vertices.push_back({v, colors});
    } else if (tokens[0] == "e") {  // e-line
      if (tokens.size() != 4) throw std::invalid_argument("unexpected e-line");

      auto u = std::stoi(tokens[1]);
      auto v = std::stoi(tokens[2]);
      auto w = std::stod(tokens[3]);
      edges.push_back({{u, v}, w});
    }
  }

  ds::graph::Graph G{vertices, edges};
  // verification
  if (n != G.number_of_vertices()) {
    throw std::invalid_argument(util::format("inconsistent n: expect=%d, actual=%lu", n, G.number_of_vertices()));
  }
  if (m != G.number_of_edges()) {
    throw std::invalid_argument(util::format("inconsistent m: expect=%d, actual=%lu", m, G.number_of_edges()));
  }
  if (c != G.number_of_colors()) {
    throw std::invalid_argument(util::format("inconsistent c: expect=%d, actual=%lu", c, G.number_of_colors()));
  }

  ds::ProblemInstance ret = {G, s, t, k};
  return ret;
}

ds::ProblemInstance load_pace(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_pace(f);
}
}  // namespace readwrite
