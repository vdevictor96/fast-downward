#include "additive_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

AdditiveHeuristic::AdditiveHeuristic(const Options &opts)
    : Heuristic(opts) {
}

void AdditiveHeuristic::initialize() {
    cout << "Initializing additive heuristic..." << endl;
}

int AdditiveHeuristic::compute_heuristic(const State &/*state*/) {
    // TODO implementation
    return -1;
}

static Heuristic *_parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new AdditiveHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hadd", _parse);
