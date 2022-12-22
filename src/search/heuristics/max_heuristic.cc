#include "max_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

MaxHeuristic::MaxHeuristic(const Options &opts)
    : Heuristic(opts) {
}

void MaxHeuristic::initialize() {
    cout << "Initializing max heuristic..." << endl;
}


int MaxHeuristic::compute_heuristic(const State &state) {
    /* Init vector with all the variable values with infinite */
    std::vector<std::vector<int> > iterative_costs;
    iterative_costs.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        iterative_costs[var].resize(g_variable_domain[var], DEAD_END);
        iterative_costs[var][state[var]] = 0;
    }
    /* Begin iterations */
    bool state_changed = true;
    while (state_changed) {
        state_changed = false;
        // vector<Operator>::iterator it = g_operators.begin();
        // while (it != g_operators.end()) {
        for (size_t i = 0; i < g_operators.size(); i++) {
            const vector<Condition> &preconditions = g_operators[i].get_preconditions();
            bool applicable = true;
            int previous_cost = 0;
            for (size_t p = 0; p < preconditions.size(); p++) {
                if (iterative_costs[preconditions[p].var][preconditions[p].val] == DEAD_END) {
                    applicable = false;
                    break;
                } else {
                    int cost = iterative_costs[preconditions[p].var][preconditions[p].val];
                    if (cost > previous_cost) {
                        previous_cost = cost;
                    }
                }
            }
            if (applicable) {
                const vector<Effect> &effects = g_operators[i].get_effects();
                const int &possible_action_cost = g_operators[i].get_cost() + previous_cost;
                for (size_t e = 0; e < effects.size(); e++) {
                    if (iterative_costs[effects[e].var][effects[e].val] == DEAD_END || possible_action_cost < iterative_costs[effects[e].var][effects[e].val]) {
                        state_changed = true;
                        iterative_costs[effects[e].var][effects[e].val] = possible_action_cost;
                    }
                    
                }
            }
        }
    }
    int max_cost = DEAD_END;
    for (size_t g = 0; g < g_goal.size(); g++) {
        int cost = iterative_costs[g_goal[g].first][g_goal[g].second];
        if (cost == DEAD_END){
            return DEAD_END;
        }
        else if (cost > max_cost) {
             max_cost = cost;
        }
    }
    return max_cost;
}


static Heuristic *_parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new MaxHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hmax", _parse);