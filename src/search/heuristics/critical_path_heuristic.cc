#include "critical_path_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

CriticalPathHeuristic::CriticalPathHeuristic(const Options &opts)
    : Heuristic(opts) {
}

void CriticalPathHeuristic::initialize() {
    cout << "Initializing h two heuristic..." << endl;
}

int CriticalPathHeuristic::compute_heuristic(const State &/*state*/) {
    // TODO implementation
    return -1;
}

static Heuristic *_parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new CriticalPathHeuristic(opts);
}

static Plugin<Heuristic> _plugin("h_two", _parse);
