#include "blind_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <limits>
#include <utility>


using namespace std;

BlindHeuristic::BlindHeuristic(const Options &opts)
    : Heuristic(opts)
{
}

void BlindHeuristic::initialize()
{
    cout << "Initializing blind heuristic..." << endl;
}

int BlindHeuristic::compute_heuristic(const State &state)
{
    if (test_goal(state)) {
        return 0;
    } else {
        return 1; // minimal operator cost
    }
}

static Heuristic *_parse(OptionParser &parser)
{
    parser.document_synopsis("Blind heuristic",
                             "Returns cost of cheapest action for "
                             "non-goal states, "
                             "0 for goal states");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "supported");
    parser.document_language_support("axioms", "supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new BlindHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("blind", _parse);
