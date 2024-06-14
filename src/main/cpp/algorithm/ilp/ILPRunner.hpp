#pragma once

#if GUROBI_ON
#include "ds/graph/Graph.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

#include "gurobi_c++.h"

namespace algorithm {
namespace ilp {

/**
 * @brief Manages a lifecycle of one ILP run.
 */
class ILPRunner {
 public:
  using Vertex = ds::graph::Graph::Vertex;
  using Edge = ds::graph::Graph::Edge;
  using Color = ds::graph::Graph::Color;
  using DirectedEdge = std::pair<Vertex, Vertex>;
  using EdgeEndPair = std::pair<DirectedEdge, Vertex>;

 private:
  std::vector<std::string> const GUROBI_STATUSES = {
      "Undefined",
      "Loaded",          // 1
      "Optimal",         // 2
      "Infeasible",      // 3
      "InfOrUnbd",       // 4
      "Unbounded",       // 5
      "Cutoff",          // 6
      "IterationLimit",  // 7
      "NodeLimit",       // 8
      "TimeLimit",       // 9
      "SolutionLimit",   // 10
      "Interrupted",     // 11
      "Numeric",         // 12
      "Suboptimal",      // 13
      "InProgress",      // 14
      "UserObjLimit",    // 15
      "WorkLimit",       // 16
      "MemLimit",        // 17
  };

  ds::graph::Graph const& graph_;
  int k_;
  int source_;
  int destination_;
  bool relax_;
  int num_threads_;
  int seed_;
  int time_limit_;
  int formulation_version_;

  /** Gurobi environment. */
  GRBEnv env_;
  GRBModel model_;
  int status_;

  /** Variable storage. */
  using VarMap = std::unordered_map<std::string, GRBVar>;
  VarMap vars_;

 public:
  ILPRunner(                                //
      ds::graph::Graph const& graph,        //
      int k,                                //
      Vertex source,                        //
      Vertex destination,                   //
      bool relax,                           // LP relaxation
      int num_threads,                      //
      int seed,                             //
      int time_limit = 0,                   // 0: no limit; >=1 in seconds
      bool output_gurobi_log = false,       //
      int formulation_version = 1,          // version of the ILP formulation
      std::vector<Edge> one_way_edges = {}  // one way edges for additional constraints
      )
      : graph_(graph),
        k_(k),
        source_(source),
        destination_(destination),
        relax_(relax),
        num_threads_(num_threads),
        seed_(seed),
        time_limit_(time_limit),
        formulation_version_(formulation_version),
        env_(create_environment(time_limit, num_threads, seed, output_gurobi_log)),
        model_(env_),
        status_(0) {
    if (formulation_version_ < 1 || 2 < formulation_version_) {
      throw std::invalid_argument("unknown formulation version");
    }

    //--------------------------------------------------------------------------
    // 1. Check preconditions
    //--------------------------------------------------------------------------
    auto n = graph.number_of_vertices();
    auto m = graph.number_of_edges();
    if (formulation_version == 1 && n * (n - 1) != m * 2) {
      // We require a connected transitive closure (with version 1).
      throw std::invalid_argument("graph must be a complete graph");
    }

    if (graph.number_of_colors() < static_cast<std::size_t>(k)) {
      throw std::invalid_argument("instance must be feasible");
    }

    //--------------------------------------------------------------------------
    // 2. Formulate LP
    //--------------------------------------------------------------------------
    vars_ = create_variables(model_, graph_, k_, relax);
    add_constraints(model_, graph_, k_, source_, destination_, formulation_version_, one_way_edges);
    add_objective(model_, graph_);
  }

  bool run(GRBCallback* callback = nullptr);

  double get_objective() const;

  std::vector<Vertex> get_solution() const;

  //--------------------------------------------------------------------------
  // Variable names
  //--------------------------------------------------------------------------
  std::string get_name_x(Vertex i, Vertex j) const {
    assert(graph_.has_vertex(i) && graph_.has_vertex(j));
    assert(i != j);
    return util::format("%d_%d", i, j);
  }

  std::string get_name_y(Vertex i, Vertex j, Vertex endpoint) const {
    assert(graph_.has_vertex(i) && graph_.has_vertex(j));
    assert(i == endpoint || j == endpoint);
    return util::format("%d_%d_%d", std::min(i, j), std::max(i, j), endpoint);
  }

  std::string get_name_z(Color c) const { return util::format("%d", c); }

  //--------------------------------------------------------------------------
  // Variable accessors
  //--------------------------------------------------------------------------
  GRBVar x(Vertex i, Vertex j) const { return vars_.at(get_name_x(i, j)); }
  GRBVar y(Vertex i, Vertex j, Vertex endpoint) const { return vars_.at(get_name_y(i, j, endpoint)); }
  GRBVar z(Color c) const { return vars_.at(get_name_z(c)); }

  /**
   * @brief Creates and starts a Gurobi environment.
   *
   * @param time_limit
   * @param num_threads
   * @param seed
   * @param output_log
   * @return GRBEnv
   */
  GRBEnv create_environment(double time_limit, int num_threads, int seed, bool output_log);

  /**
   * @brief Creates a map of all variables.
   *
   * @param model
   * @param graph
   * @param k
   * @param relax
   * @return VarMap
   */
  VarMap create_variables(GRBModel& model, ds::graph::Graph const& graph, int k, bool relax) const;

  /**
   * @brief Adds constraints to the model.
   *
   * @param model
   * @param graph
   * @param k
   * @param source
   * @param destination
   * @param version
   * @param one_way_edges
   */
  void add_constraints(GRBModel& model, ds::graph::Graph const& graph, int k, int source, int destination, int version,
                       std::vector<Edge> const& one_way_edges) const;

  /**
   * @brief Adds objective to the model.
   *
   * @param model GRBModel instance
   * @param graph input graph
   */
  void add_objective(GRBModel& model, ds::graph::Graph const& graph) const;

  std::vector<Vertex> extract_solution_path() const {
    std::map<DirectedEdge, double> x_values;

    for (Vertex i : graph_.vertices()) {
      for (Vertex j : graph_.neighbors(i)) {
        //
        x_values[{i, j}] = x(i, j).get(GRB_DoubleAttr_X);
      }
    }
    return extract_callback_path(x_values);
  }

  std::vector<Vertex> extract_callback_path(std::map<DirectedEdge, double> const& x_values) const;
};
}  // namespace ilp
}  // namespace algorithm
#endif
