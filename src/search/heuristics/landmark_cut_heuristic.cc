#include "landmark_cut_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

LMCutHeuristic::LMCutHeuristic(const Options &opts)
    : Heuristic(opts)
{
}

void LMCutHeuristic::initialize()
{
    cout << "Initializing LM-cut heuristic..." << endl;
}

int LMCutHeuristic::compute_heuristic(const State &/*state*/)
{
    // TODO implementation
    return -1;
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new LMCutHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("lmcut", _parse);
