#include "dominance.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;


Dominance::Dominance()
{
    cout << "pruning method: dominance" << endl;
}

void Dominance::initialize()
{
    cout << "initializing Dominance Pruning" << endl;
}

void Dominance::remove_pruned_operators(
    const State &/*state*/, vector<const Operator *> &/*ops*/)
{

    // TODO implement this
}

static PruningMethod *_parse(OptionParser &parser)
{
    parser.document_synopsis(
        "dominance pruning",
        "Dominance pruning method using no-op dominance");

    if (parser.dry_run()) {
        return 0;
    }

    return new Dominance();
}

static Plugin<PruningMethod> _plugin("dominance", _parse);
