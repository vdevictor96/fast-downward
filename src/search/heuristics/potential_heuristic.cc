#include "potential_heuristic.h"

#include "../linear_program.h"
#include "../state.h"
#include "../operator.h"
#include "../globals.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../utilities.h"

#include <iostream>
#include <vector>

PotentialHeuristic::PotentialHeuristic(const Options& opts)
    : Heuristic(opts)
{
#if !USE_LP
    std::cerr << "Potential heuristic requires compilation with LP solver support" << std::endl;
    exit_with(EXIT_CRITICAL_ERROR);
#endif
}

void
PotentialHeuristic::initialize()
{
    std::cout << "Initializing potential heuristic..." << std::endl;

    /* For a linear program usage example, see ../linear_program.h */

    /* Generate LP variables and constraints */
    // std::vector<lp::LPVariable> variables;
    // std::vector<lp::LPConstraint> constraints;

    /* TODO */

    /* Generate linear program with the variables and constraints generated
       above */
    // lp::LinearProgram linear_program(lp::LPObjectiveSense::MAXIMIZE,
    //                                  variables,
    //                                  constraints);
    
    /* Solve the LP */
    // linear_program.solve();

    /* Get the solution vector */
    // std::vector<double> sol = linear_program.extract_solution();
    
    /* Extract the weights from the solution vector */
    /* TODO */
}

int
PotentialHeuristic::compute_heuristic(const State& /*state*/)
{
    /* TODO */
    return DEAD_END;
}

static Heuristic*
_parse(OptionParser& parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        return new PotentialHeuristic(opts);
    }
    return NULL;
}

static Plugin<Heuristic> _plugin("pot", _parse);

