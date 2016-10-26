#include "landmark_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

LandmarkHeuristic::LandmarkHeuristic(const Options &opts)
    : Heuristic(opts)
{
}

void LandmarkHeuristic::initialize()
{
    cout << "Initializing landmark heuristic..." << endl;
}

int LandmarkHeuristic::compute_heuristic(const State &/*state*/)
{
    // TODO implementation
    return -1;
}

void LandmarkHeuristic::reach_state(const State &/*parent_state*/,
                                    const Operator &/*op*/,
                                    const State &/*state*/)
{
    // TODO implementation
}

void LandmarkHeuristic::get_helpful_actions(std::vector<const Operator *>
        &/*result*/)
{
    // TODO implementation
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new LandmarkHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("lm", _parse);
