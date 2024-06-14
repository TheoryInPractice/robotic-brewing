#if GUROBI_ON
#include "ILPRunner.hpp"
#include "algorithm/graph/eulerian.hpp"

namespace algorithm {
namespace ilp {
bool ILPRunner::run(GRBCallback* callback) {
  util::Timer timer;
  log_info("ILPRunner: started: k=%d, relax=%s, #threads=%d, seed=%d, time_limit=%d", k_, relax_ ? "true" : "false",
           num_threads_, seed_, time_limit_);

  // set callback
  if (callback) model_.setCallback(callback);

  // optimize
  model_.optimize();

  // check status
  status_ = model_.get(GRB_IntAttr_Status);

  if (status_ == GRB_TIME_LIMIT) {
    log_warning("ILPRunner: timed out: k=%d, relax=%s, #vars=%d, #constrs=%d, obj=%.3f, status=%s, elapsed=%.3fs", k_,
                relax_ ? "true" : "false", model_.get(GRB_IntAttr_NumVars), model_.get(GRB_IntAttr_NumConstrs),
                model_.get(GRB_DoubleAttr_ObjVal), GUROBI_STATUSES[status_].c_str(), timer.stop());
  } else if (status_ == GRB_OPTIMAL) {
    log_info("ILPRunner: finished: k=%d, relax=%s, #vars=%d, #constrs=%d, obj=%.3f, status=%s, elapsed=%.3fs", k_,
             relax_ ? "true" : "false", model_.get(GRB_IntAttr_NumVars), model_.get(GRB_IntAttr_NumConstrs),
             model_.get(GRB_DoubleAttr_ObjVal), GUROBI_STATUSES[status_].c_str(), timer.stop());
  } else {
    log_error("ILPRunner: finished: k=%d, relax=%s, #vars=%d, #constrs=%d, status=%s, elapsed=%.3fs", k_,
              relax_ ? "true" : "false", model_.get(GRB_IntAttr_NumVars), model_.get(GRB_IntAttr_NumConstrs),
              GUROBI_STATUSES[status_].c_str(), timer.stop());
    throw std::runtime_error("unexpected Gurobi status");
  }
  return status_ != GRB_TIME_LIMIT;
}

double ILPRunner::get_objective() const { return status_ == GRB_OPTIMAL ? model_.get(GRB_DoubleAttr_ObjVal) : 0.0; }

std::vector<ILPRunner::Vertex> ILPRunner::get_solution() const {
  return status_ == GRB_OPTIMAL ? extract_solution_path() : std::vector<Vertex>();
}

/**
 * @brief Creates and starts a Gurobi environment.
 *
 * @param time_limit
 * @param num_threads
 * @param seed
 * @param output_log
 * @return GRBEnv
 */
GRBEnv ILPRunner::create_environment(double time_limit, int num_threads, int seed, bool output_log) {
  GRBEnv env = GRBEnv(true);
  env.set(GRB_StringParam_LogFile, "");  // no log file
  env.set(GRB_DoubleParam_TimeLimit, time_limit <= 0 ? GRB_INFINITY : time_limit);
  env.set(GRB_IntParam_Threads, num_threads);
  env.set(GRB_IntParam_OutputFlag, output_log ? 1 : 0);
  env.set(GRB_IntParam_Seed, seed);
  env.start();
  return env;
}

/**
 * @brief Creates a map of all variables.
 *
 * @param model
 * @param graph
 * @param k
 * @param relax
 * @return VarMap
 */
ILPRunner::VarMap ILPRunner::create_variables(GRBModel& model, ds::graph::Graph const& graph, int k, bool relax) const {
  VarMap ret;
  for (Vertex i : graph.vertices()) {
    for (Vertex j : graph.neighbors(i)) {
      auto vtype = relax ? GRB_CONTINUOUS : GRB_BINARY;
      ret.emplace(get_name_x(i, j), model.addVar(0.0, 1.0, 0.0, vtype));
    }
  }
  for (auto edge : graph.edges()) {
    auto i = edge.first;
    auto j = edge.second;
    ret.emplace(get_name_y(i, j, i), model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
    ret.emplace(get_name_y(i, j, j), model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
  }
  if (k < static_cast<int>(graph.number_of_colors())) {
    for (auto c : graph.get_colors()) { ret.emplace(get_name_z(c), model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS)); }
  }
  return ret;
}

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
void ILPRunner::add_constraints(GRBModel& model, ds::graph::Graph const& graph, int k, int source, int destination,
                                int version, std::vector<ds::graph::Graph::Edge> const& one_way_edges) const {
  double n = graph.number_of_vertices();
  std::map<Color, std::vector<Vertex>> colors;
  for (Vertex i : graph.vertices()) {
    for (auto c : graph.get_colors(i)) { colors[c].push_back(i); }
  }
  GRBLinExpr expr;

  // Flow condition.
  for (Vertex v : graph.vertices()) {
    GRBLinExpr in = 0;
    GRBLinExpr out = 0;
    for (Vertex u : graph.neighbors(v)) {
      if (u != v) {
        in += x(u, v);
        out += x(v, u);
      }
    }

    if (v == source && source == destination) {
      // Flow at source/destination.
      if (formulation_version_ == 1) {
        model.addConstr(out == 1);
        model.addConstr(in == 1);
      } else if (formulation_version_ == 2) {
        model.addConstr(out >= 1);  // not required, but helps make faster
        model.addConstr(in == out);
      }
    } else if (v == source) {
      // Flow at source.
      if (formulation_version_ == 1) {
        model.addConstr(out == 1);
      } else if (formulation_version_ == 2) {
        model.addConstr(out >= 1);  // not required, but helps make faster
        model.addConstr(out - in == 1);
      }
    } else if (v == destination) {
      // Flow at destination.
      if (formulation_version_ == 1) {
        model.addConstr(in == 1);
      } else if (formulation_version_ == 2) {
        model.addConstr(in >= 1);  // not required, but helps make faster
        model.addConstr(in - out == 1);
      }
    } else {
      // Otherwise.
      model.addConstr(in == out);
    }
  }

  // Charges from solution edges
  for (auto& edge : graph.edges()) {
    auto i = edge.first;
    auto j = edge.second;
    if (i != source && j != source) {
      GRBLinExpr y_sum = y(i, j, i) + y(i, j, j);
      GRBLinExpr x_sum = x(i, j) + x(j, i);

      model.addConstr(y_sum == 2 * x_sum);
    }
  }

  for (Vertex v : graph.vertices()) {
    if (v != source) {
      GRBLinExpr expr = 0;
      for (Vertex u : graph.neighbors(v)) {
        if (u != source) { expr += y(u, v, v); }
      }

      GRBLinExpr limit = 0;

      switch (version) {
        case 1: {
          limit = 2.0 - 1.0 / n;
          break;
        }
        case 2: {  // Formulation Version 2
          GRBLinExpr multiplicity = 0;
          for (Vertex u : graph.neighbors(v)) multiplicity += x(u, v);
          limit = multiplicity * (2.0 - 1.0 / n);
          break;
        }
        default: {
        }
      }

      model.addConstr(expr <= limit);
    }
  }

  auto unique_colors = graph.get_colors();

  for (auto c : unique_colors) {
    GRBLinExpr expr = 0;
    for (Vertex v : colors[c]) {
      for (Vertex u : graph.neighbors(v)) {
        // Optimization: We may assume that a new color is collected at v.
        expr += x(u, v);
      }
    }

    if (k == static_cast<int>(unique_colors.size())) {
      model.addConstr(expr >= 1);
    } else {
      model.addConstr(expr >= z(c));
    }
  }

  if (k < static_cast<int>(unique_colors.size())) {
    GRBLinExpr expr = 0;
    for (auto c : unique_colors) { expr += z(c); }
    model.addConstr(expr >= k);
  }

  // Additional constraints: one-way edges
  for (auto& e : one_way_edges) {
    //
    model.addConstr(x(e.first, e.second) + x(e.second, e.first) <= 1);
  }
}

/**
 * @brief Adds objective to the model.
 *
 * @param model GRBModel instance
 * @param graph input graph
 */
void ILPRunner::add_objective(GRBModel& model, ds::graph::Graph const& graph) const {
  GRBLinExpr expr = 0;
  for (auto& edge : graph.weighted_edges()) {
    auto ed = edge.first;
    auto i = ed.first;
    auto j = ed.second;
    expr += edge.second * (x(i, j) + x(j, i));
  }
  model.setObjective(expr, GRB_MINIMIZE);
}

static std::vector<ILPRunner::DirectedEdge> extract_solution_edges(std::map<ILPRunner::DirectedEdge, double> const& x_values) {
  std::vector<ILPRunner::DirectedEdge> ret;

  for (auto& e : x_values) {
    // Threshold for inclusion in the path
    if (e.second > 0.9) ret.push_back(e.first);
  }
  return ret;
}

std::vector<ILPRunner::Vertex> ILPRunner::extract_callback_path(std::map<ILPRunner::DirectedEdge, double> const& x_values) const {
  auto solution_edges = extract_solution_edges(x_values);
  return algorithm::graph::eulerian_path(solution_edges, source_);
}
}  // namespace ilp
}  // namespace algorithm
#endif
