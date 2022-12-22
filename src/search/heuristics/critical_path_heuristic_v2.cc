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

typedef std::pair<int,int> varVal;
typedef std::pair<varVal,varVal> varVals;
// typedef std::vector<std::tuple<varVal,varVal,int>> facts;
typedef std::map<varVals,int> facts_map;
typedef std::map<varVals,int>::iterator facts_map_it;

vector<int> combination;

void go(int offset, int k, facts_map &iterative_costs, const State &state) {
    if (k == 0) {
        static int count = 0;
        // cout << "combination no " << (++count) << ": [ ";
        // for (int i = 0; i < combination.size(); ++i) { cout << combination[i] << " "; }
        // cout << "] " << endl;
        for(int val = 0; val < g_variable_domain[combination[0]]; val++) {
            for(int val2 = 0; val2 < g_variable_domain[combination[1]]; val2++) {
                varVals comb = make_pair(make_pair(combination[0],val), make_pair(combination[1],val2));
                if (state[combination[0]] == val && state[combination[1]] == val2) {
                    iterative_costs.insert({comb, 0});
                } else {
                    iterative_costs.insert({comb, -1});
                }
            }
        }
        return;
    }
    for (int var = offset; var <= g_variable_domain.size() - k; ++var) {
        combination.push_back(var);
        go(var+1, k-1, iterative_costs, state);
        combination.pop_back();
    }
}
int CriticalPathHeuristic::compute_heuristic(const State &state) {
    // dump_everything();
    /* Init vector with all the variable values with infinite */
    facts_map iterative_costs;
    go(0, 2, iterative_costs, state);
    /* Add size 1 facts as pair of same facts */
    for(size_t var = 0; var < g_variable_domain.size(); var++) {
        for(size_t val = 0; val < g_variable_domain[var]; val++) {
            varVals comb = make_pair(make_pair(var,val), make_pair(var,val));
            if (state[var] == val) {
                iterative_costs.insert({comb, 0});
            } else {
                iterative_costs.insert({comb, -1});
            }
        }
        
    }
    // for(facts_map_it it = iterative_costs.begin(); it != iterative_costs.end(); ++it) {
    //     cout << "var1: " << it-> first.first.first;
    //     cout << " val1: " << it-> first.first.second;
    //     cout << " var2: " << it-> first.second.first;
    //     cout << " val2: " << it-> first.second.second;
    //     cout << " h: " << it-> second << endl;
    // }
    // for (int i = 0; i < iterative_costs.size(); i++) {
    //     cout << "var1: " << std::get<0>(iterative_costs[i]).first;
    //     cout << " val1: " << std::get<0>(iterative_costs[i]).second;
    //     cout << " var2: " << std::get<1>(iterative_costs[i]).first;
    //     cout << " val2: " << std::get<1>(iterative_costs[i]).second;
    //     cout << " h: " << std::get<2>(iterative_costs[i]) << endl;
    // }
    // cout << "comb size: " << iterative_costs.size() << endl;
    // return DEAD_END;
    /* Set initial state cost to 0*/
    /* Begin iterations */
    bool state_changed = true;
    int count = 0;
    while (state_changed) {
        // cout << " while iterations: " << count++ << endl;
        state_changed = false;
        for (size_t i = 0; i < g_operators.size(); i++) {
            const vector<Condition> &preconditions = g_operators[i].get_preconditions();
            bool applicable = true;
            int previous_cost = 0;
            /* Check for applicability of the action. All pair of preconditions must not different from DEAD_END */
            for (size_t p = 0; p < preconditions.size(); p++) {
                if (!applicable) {
                    break;
                }
                for (size_t p2 = 0; p2 < preconditions.size(); p2++) {
                    varVal pre1 = make_pair(preconditions[p].var,preconditions[p].val);
                    varVal pre2 = make_pair(preconditions[p2].var,preconditions[p2].val);
                    varVals key = iterative_costs.count(make_pair(pre1, pre2)) != 0 ? make_pair(pre1, pre2) : make_pair(pre2, pre1);
                    int cost = iterative_costs[key];
                    if (cost == DEAD_END) {
                        // if any is DEAD_END, then the action is not applicable
                        applicable = false;
                        break;
                    } else { 
                        if (cost > previous_cost) {
                            previous_cost = cost;
                        }
                    }
                } 
            }
            if (applicable) {
                const vector<Effect> &effects = g_operators[i].get_effects();
                // int possible_action_cost = g_operators[i].get_cost() + previous_cost;
                const int &possible_action_cost = g_operators[i].get_cost() + previous_cost;
                /* Create pairs of effects + effects */
                for (size_t e = 0; e < effects.size(); e++) {
                    for (size_t e2 = 0; e2 < effects.size(); e2++) {
                        varVal eff1 = make_pair(effects[e].var,effects[e].val);
                        varVal eff2 = make_pair(effects[e2].var,effects[e2].val);
                        // if((eff1.first == 4 && eff1.second == 1) || (eff2.first == 4 && eff2.second == 1)) {
                        //     cout << "goal effect found";
                        // }
                        // if pair does not exists, the pair in the other order should exists
                        varVals key = iterative_costs.count(make_pair(eff1, eff2)) != 0 ? make_pair(eff1, eff2) : make_pair(eff2, eff1);
                        bool exists_k1 = iterative_costs.count(make_pair(eff1, eff2)) != 0;
                        bool exists_k2 = iterative_costs.count(make_pair(eff2, eff1)) != 0;
                        // TODO remove. it should alaways be true
                        if (iterative_costs.count(key) != 0) {
                            int cost = iterative_costs[key];
                             if (cost == DEAD_END ){
                                // || possible_action_cost < cost) 
                            // if (cost == DEAD_END || possible_action_cost < cost ){
                                state_changed = true;
                                iterative_costs[key] = possible_action_cost;
                            }
                        }
                    }
                }
                for(facts_map_it it = iterative_costs.begin(); it != iterative_costs.end(); ++it) {
                    varVal fact1 = it->first.first;
                    varVal fact2 = it->first.second;
                    int cost = it->second;
                    // possible_action_cost = g_operators[i].get_cost() + cost;
                    /* If the pair of facts is DEAD_END, do not use it for creating new facts */
                    if (cost == DEAD_END) { goto exit; }
                    // else { 
                    //     if (cost > previous_cost) {
                    //         previous_cost = cost;
                    //         possible_action_cost = g_operators[i].get_cost() + previous_cost;
                    //     }
                    // }
                    /* If any effect of the action is inconsistent with this pair of facts then break loop */
                    // TODO check for one fact or for both?
                    // for (size_t e = 0; e < effects.size(); e++) {
                    //     if ((effects[e].var == fact1.first && effects[e].val != fact1.second) ||
                    //         (effects[e].var == fact2.first && effects[e].val != fact2.second)) { 
                    //             goto exit; 
                    //     }
                    // }
                    /* Create pairs with pairs of facts and effects of the action */
                    for (size_t e = 0; e < effects.size(); e++) {
                        varVal eff = make_pair(effects[e].var,effects[e].val);
                        /* Effect + fact 1 */
                        varVals key = iterative_costs.count(make_pair(fact1, eff)) != 0 ? make_pair(fact1, eff) : make_pair(eff, fact1);
                        // If a current fact and an effect is incosistent (same var and different val) it wont exits in iterative_costs
                        if (iterative_costs.count(key) != 0) {
                            int cost = iterative_costs[key];
                            if (cost == DEAD_END || possible_action_cost < cost) {
                                state_changed = true;
                                iterative_costs[key] = possible_action_cost;
                            }
                        }
                    }
                    // if (fact1 == fact2) { goto exit; }
                    /* Create pairs with pairs of facts and effects of the action */
                    for (size_t e = 0; e < effects.size(); e++) {
                        varVal eff = make_pair(effects[e].var,effects[e].val);
                         /* Effect + fact 2 */
                        varVals key2 = iterative_costs.count(make_pair(fact2, eff)) != 0 ? make_pair(fact2, eff) : make_pair(eff, fact2);
                        // If a current fact and an effect is incosistent (same var and different val) it wont exits in iterative_costs
                        if (iterative_costs.count(key2) != 0) {
                            int cost = iterative_costs[key2];
                            if (cost == DEAD_END || possible_action_cost < cost) {
                                state_changed = true;
                                iterative_costs[key2] = possible_action_cost;
                            }
                        }
                    }
                    /* Exit clause */
                    exit: ;
                }
            }
        }
    }
    // cout << "goal checking : " << endl;
    // for(facts_map_it it = iterative_costs.begin(); it != iterative_costs.end(); ++it) {
    //     cout << "var1: " << it-> first.first.first;
    //     cout << " val1: " << it-> first.first.second;
    //     cout << " var2: " << it-> first.second.first;
    //     cout << " val2: " << it-> first.second.second;
    //     cout << " h: " << it-> second << endl;
    // }
    int max_cost = DEAD_END;
    for (size_t g = 0; g < g_goal.size(); g++) {
        for (size_t g2 = 0; g2 < g_goal.size(); g2++) {
            varVal goal1 = make_pair(g_goal[g].first,g_goal[g].second);
            varVal goal2 = make_pair(g_goal[g2].first,g_goal[g2].second);
            varVals key = iterative_costs.count(make_pair(goal1, goal2)) != 0 ? make_pair(goal1, goal2) : make_pair(goal2, goal1);
            // TDO remove. it should always be true
            if (iterative_costs.count(key) != 0) {
                int cost = iterative_costs[key];
                if (cost == DEAD_END){
                    return DEAD_END;
                }
                else if (cost > max_cost) {
                    max_cost = cost;
                }
            }
            
        }
    }
    // cout << "first heuristic value: " << max_cost  << endl;
    // return DEAD_END;
    return max_cost;
}

// static Heuristic *_parse(OptionParser &parser) {
//     Heuristic::add_options_to_parser(parser);
//     Options opts = parser.parse();
//     if (parser.dry_run())
//         return 0;
//     else
//         return new CriticalPathHeuristic(opts);
// }

// static Plugin<Heuristic> _plugin("h_two", _parse);
