#include "pattern_database_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

PDBHeuristic::PDBHeuristic(const Options &opts)
    : Heuristic(opts),
      m_test_pattern(opts.contains("test_pattern") ? new std::vector<int>
                     (opts.get_list<int>("test_pattern")) : NULL)
{
}

void PDBHeuristic::initialize()
{
    cout << "Initializing PDB heuristic..." << endl;
}

int PDBHeuristic::compute_heuristic(const State &/*state*/)
{
    // TODO implementation
    return -1;
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<int>("test_pattern", "", "", OptionFlags(false));
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new PDBHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("pdb", _parse);
