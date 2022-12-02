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
    std::vector<std::vector<size_t> > iterative_costs;
    iterative_costs.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        iterative_costs[var].resize(g_variable_domain[var], INT16_MAX);
        iterative_costs[var][state[var]] = 0;
    }

    bool state_changed = true;
    while (state_changed) {
        state_changed = false;
        // vector<Operator>::iterator it = g_operators.begin();
        // while (it != g_operators.end()) {
        for (size_t i = 0; i < g_operators.size(); i++) {
            const vector<Condition> &preconditions = g_operators[i].get_preconditions();
            bool applicable = true;
            size_t previous_cost = 0;
            for (size_t p = 0; p < preconditions.size(); p++) {
            // for (const Condition &pre : (it)->get_preconditions()) {
                // if (iterative_costs[pre.var][pre.val] == INT16_MAX) {
                //     applicable = false;
                //     break;
                // } else {
                //     size_t cost = iterative_costs[pre.var][pre.val];
                //     if (cost > previous_cost) {
                //         previous_cost = cost;
                //     }
                // }
                if (iterative_costs[preconditions[p].var][preconditions[p].val] == INT16_MAX) {
                    applicable = false;
                    break;
                } else {
                    size_t cost = iterative_costs[preconditions[p].var][preconditions[p].val];
                    if (cost > previous_cost) {
                        previous_cost = cost;
                    }
                }
            }
            if (applicable) {
                const vector<Effect> &effects = g_operators[i].get_effects();
                // for (const Effect &eff : (it)->get_effects()) {
                size_t possible_action_cost = g_operators[i].get_cost() + previous_cost;
                for (size_t e = 0; e < effects.size(); e++) {
                    if ( possible_action_cost < iterative_costs[effects[e].var][effects[e].val]) {
                        // iterative_costs[effects[e].var][effects[e].val] == INT16_MAX ||
                        state_changed = true;
                        // iterative_costs[effects[e].var][effects[e].val] = (it)->get_cost() + previous_cost; //TODO elemento en iterative costs que este en las precondiciones ;
                        iterative_costs[effects[e].var][effects[e].val] = possible_action_cost; //TODO elemento en iterative costs que este en las precondiciones ;
                    }
                    
                }
                // it = g_operators.erase(it);
            } else {
                // it++;
            }
        }
    }
    // if (!g_operators.empty()) {
    //     // Not all actions are applicable
    //     return false;
    // }
    // check if all goal facts are reached:
    size_t max_cost = 0;
    bool reached = false;
    for (size_t g = 0; g < g_goal.size(); ++g) {
        size_t cost = iterative_costs[g_goal[g].first][g_goal[g].second];
        if (cost != INT16_MAX) {
            reached = true;
            if (cost > max_cost) {
            max_cost = cost;
            }
        }
    }
    if (reached) {
        return max_cost;
    } else {
        return -1;
    }
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


// return compute_inner_heuristic(state);
    // dump_everything();
    //  // for action in actions
    // for (size_t i = 0; i < g_axioms.size(); ++i) {
    //     const vector<Effect> &effects = g_axioms[i].get_effects();
    //      for (size_t j = 0; j < effects.size(); j++) {
    //             cout << "Effect " << j << endl;
    //             cout << "var -  " << effects[j].var << endl;
    //             cout << "val -  " << effects[j].val << endl;
    //      }
    // }
    // compute_inner_heuristic(state);
    // // for action in actions
    // for (size_t i = 0; i < g_operators.size(); ++i) {
    //     const vector<Effect> &effects = g_operators[i].get_effects();
    //     for (size_t j = 0; j < effects.size(); ++j) {
    //         const vector<Condition> &cond = effects[j].conditions;
    //         if (!cond.empty()) {
    //             return i;
    //         }
    //     }
    // }
    // // ---------------------------------
    // if (test_goal(state)) { return 0;}
    // return compute_inner_heuristic(state, g_goal);

// int cost_state_to_goal(const State &state, pair<int, int> &subgoal)
// {
//     int curr_cost = 0;
//     int min_cost = 6000;
//     // int min_cost = INT16_MAX;
//     /* Check if subgoal is contained in state */
//     if (state[subgoal.first] != subgoal.second) {
//         for (size_t i = 0; i < g_operators.size(); i++) {
//             const vector<Effect> &effects = g_operators[i].get_effects();
//             const vector<Condition> &preconditions = g_operators[i].get_preconditions();
//             for (size_t j = 0; j < effects.size(); j++) {
//                 cout << "Effect " << j << endl;
//                 cout << "var -  " << effects[j].var << endl;
//                 cout << "val -  " << effects[j].val << endl;
//                 if (effects[j].var == subgoal.first && effects[j].val == subgoal.second) {
//                     vector<pair<int,int>> preconditionPairs;
//                     for (size_t k = 0; preconditions.size(); k++) {
//                         cout << "var -  " << preconditions[k].var << endl;
//                         cout << "val -  " << preconditions[k].val << endl;
//                         // preconditionPairs.push_back(make_pair(preconditions[k].var, preconditions[k].val));
//                     }
//                     curr_cost = g_operators[i].get_cost(); // + compute_inner_heuristic(state, preconditionPairs);
//                     break;
//                 }
//             }
//         }
//         if (curr_cost < min_cost) {
//             min_cost = curr_cost;
//         }
        
//         return min_cost;
//     } else {
//         return 0;
//     }
// }

// int compute_inner_heuristic(const State &state, vector<pair<int, int>> &goal)
// {
//     int curr_cost = 0;
//     int max_cost = 0;
//     for(size_t i = 0; i < goal.size(); ++i) {
//         curr_cost = cost_state_to_goal(state, goal[i]);
//         if (curr_cost > max_cost) {
//             curr_cost = max_cost;
//         }
//     }
//     return max_cost;
// }