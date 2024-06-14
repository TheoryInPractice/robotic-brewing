#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/ilp/ILPRunner.hpp"
#include "util/Random.hpp"
#include <iostream>

namespace algorithm {
namespace exact {
class ILPSolver : public base::BaseSolver {
 private:
  using Vertex = ds::graph::Graph::Vertex;
  using DirectedEdge = std::pair<Vertex, Vertex>;

  int seed_;
  int num_threads_;
  bool output_gurobi_log_;
  int formulation_version_;

  class MyCallback;

 public:
  ILPSolver(ds::graph::Graph const& graph, int seed, int num_threads, bool output_gurobi_log = false,
            bool print_solution = false, bool print_certificate = false, int formulation_version = 1)
      : base::BaseSolver("ILPSolver", graph, print_solution, print_certificate),
        seed_(seed),
        num_threads_(num_threads),
        output_gurobi_log_(output_gurobi_log),
        formulation_version_(formulation_version) {
    log_info("%s: Initialized: num_threads=%d, output_gurobi_log=%s, formulation_version=%d", get_solver_name(),
             num_threads, output_gurobi_log ? "true" : "false", formulation_version);
  }
  friend class MyCallback;

  /**
   * @brief Solves the Inspection Planning problem.
   * Result will be stored as the fields `solution_`, `solution_weight_`, `solution_colors_`,
   * and `is_feasible_`.
   *
   * @param k number of colors to collect
   * @param source starting vertex
   * @param destination ending vertex (can be same as source)
   * @param time_limit time limit in seconds (0: no time limit)
   *
   * @return true finished solving the problem (can be either feasible or infeasible)
   * @return false solver timed out
   */
  bool solve(int k, Vertex source, Vertex destination, int time_limit = 30) override {
#if GUROBI_ON
    //--------------------------------------------------------------------------
    // 1. Preprocessing
    //--------------------------------------------------------------------------
    if (preprocess(k, source, destination)) return true;

    if (formulation_version_ == 1) {
      create_transitive_closure(true);
      remove_colorless_vertices({source, destination});  // keep source and destination
    }

    log_info("%s: Finished preprocessing: n=%lu, m=%lu, |C|=%lu", get_solver_name(), get_graph().number_of_vertices(),
             get_graph().number_of_edges(), get_graph().number_of_colors());

    //--------------------------------------------------------------------------
    // 2. Run ILP
    //--------------------------------------------------------------------------
    auto const& g = get_graph();
    auto runner = ilp::ILPRunner(g, k, source, destination, false, num_threads_, seed_, time_limit, output_gurobi_log_,
                                 formulation_version_);

    MyCallback cb(this, &runner);
    auto ret = runner.run(&cb);

    //--------------------------------------------------------------------------
    // 3. Recover solution
    //--------------------------------------------------------------------------
    auto solution = runner.get_solution();

    if (!solution.empty()) {
      // set optimal solution
      clear_solution();
      set_critical_walk(solution);
    }
    return ret;
#else
    throw std::runtime_error("Gurobi option was disabled.");
#endif
  }

 private:
#if GUROBI_ON
  class MyCallback : public GRBCallback {
   private:
    ILPSolver* solver_;
    ilp::ILPRunner* runner_;

   public:
    MyCallback(ILPSolver* solver, ilp::ILPRunner* runner) : solver_(solver), runner_(runner) {}

   protected:
    void callback() override {
      if (where == GRB_CB_MIPSOL) {
        auto vertices = solver_->get_graph().vertices();
        std::map<DirectedEdge, double> x_values;

        for (Vertex i : vertices) {
          for (Vertex j : solver_->get_graph().neighbors(i)) {
            //
            x_values[{i, j}] = getSolution(runner_->x(i, j));
          }
        }
        std::vector<Vertex> path = runner_->extract_callback_path(x_values);

        solver_->set_critical_walk(path);
      }
    }
  };
#endif
};
}  // namespace exact
}  // namespace algorithm
