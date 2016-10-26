#include "ff_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

FFHeuristic::FFHeuristic(const Options &opts)
    : Heuristic(opts)
{
}

void FFHeuristic::initialize()
{
    cout << "Initializing FF heuristic..." << endl;
}

int FFHeuristic::compute_heuristic(const State &/*state*/)
{
    // TODO implementation
    return -1;
}

void FFHeuristic::get_helpful_actions(std::vector<const Operator *>
                                      &/*result*/)
{
    // TODO implementation
}

bool FFHeuristic::is_relaxed_plan(const State &state,
                                  std::vector<const Operator *> relaxed_plan)
{
    std::vector<std::vector<bool> > reached;
    reached.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        reached[var].resize(g_variable_domain[var], false);
        reached[var][state[var]] = true;
    }
    bool state_changed = true;
    while (state_changed) {
        state_changed = false;
        std::vector<const Operator *>::iterator it = relaxed_plan.begin();
        while (it != relaxed_plan.end()) {
            bool applicable = true;
            for (const Condition &pre : (*it)->get_preconditions()) {
                if (!reached[pre.var][pre.val]) {
                    applicable = false;
                    break;
                }
            }
            if (applicable) {
                for (const Effect &eff : (*it)->get_effects()) {
                    state_changed = state_changed || !reached[eff.var][eff.val];
                    reached[eff.var][eff.val] = true;
                }
                it = relaxed_plan.erase(it);
            } else {
                it++;
            }
        }
    }
    if (!relaxed_plan.empty()) {
        // Not all actions are applicable
        return false;
    }
    // check if all goal facts are reached:
    for (const std::pair<int, int> &g : g_goal) {
        if (!reached[g.first][g.second]) {
            return false;
        }
    }
    return true;
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new FFHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("ff", _parse);
