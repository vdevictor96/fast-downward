#include "operator_counting_heuristic.h"

#include "../globals.h"
#include "../state.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <iostream>
#include <vector>

OperatorCountingHeuristic::OperatorCountingHeuristic(const Options& opts)
    : Heuristic(opts)
    , linear_program(nullptr)
{
#if !USE_LP
    std::cerr << "Operator counting heuristic requires compilation with LP solver support" << std::endl;
    exit_with(EXIT_CRITICAL_ERROR);
#endif
}

void
OperatorCountingHeuristic::initialize()
{
    std::cout << "Initializing operator counting heuristic ..." << std::endl;

    /* For a linear program usage example, see ../linear_program.h */

    /* Generate LP variables and constraints */
    // std::vector<lp::LPVariable> variables;
    // std::vector<lp::LPConstraint> constraints;

    /* TODO */

    /* Generate linear program with the variables and constraints constructed
       above */
    // linear_program = std::unique_ptr<lp::LinearProgram>(new lp::LinearProgram(
    //             lp::LPObjectiveSense::MINIMIZE,
    //             variables,
    //             constraints));
}

int
OperatorCountingHeuristic::compute_heuristic(const State& /*state*/)
{
    /* Update variable / constraint bounds, e.g., 
         linear_program->change_constraint_lower_bound(constraint_index, lower_bound);
         linear_program->change_variable_upper_bound(lp_var_index, upper_bound);
    */
    /* TODO */

    /* Solve the (updated) linear program */
    // linear_program->solve();

    /* Check if the the LP has a solution, via
         linear_program->has_optimal_solution()
       and if so, use the objective value as heuristic value
         linear_program->get_objective_value();
    */
    /* TODO */

    return DEAD_END;
}

static Heuristic*
_parse(OptionParser& parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        return new OperatorCountingHeuristic(opts);
    }
    return NULL;
}

static Plugin<Heuristic> _plugin("operator_counting", _parse);

