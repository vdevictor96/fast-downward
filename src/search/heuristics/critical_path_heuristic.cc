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
       cout << "combination no " << (++count) << ": [ ";
       for (int i = 0; i < combination.size(); ++i) { cout << combination[i] << " "; }
       cout << "] " << endl;

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
    for (int var = offset; var <= g_variable_domain.size() - k; var++) {
        combination.push_back(var);
        go(var+1, k-1, iterative_costs, state);
        combination.pop_back();
    }
}
int CriticalPathHeuristic::compute_heuristic(const State &state) {
    dump_everything();
    /* Init vector with all the variable values with infinite */
    facts_map iterative_costs;
    go(0, 2, iterative_costs, state);
    for(facts_map_it it = iterative_costs.begin(); it != iterative_costs.end(); ++it) {
        cout << "var1: " << it-> first.first.first;
        cout << " val1: " << it-> first.first.second;
        cout << " var2: " << it-> first.second.first;
        cout << " val2: " << it-> first.second.second;
        cout << " h: " << it-> second << endl;
    }
    // for (int i = 0; i < iterative_costs.size(); i++) {
    //     cout << "var1: " << std::get<0>(iterative_costs[i]).first;
    //     cout << " val1: " << std::get<0>(iterative_costs[i]).second;
    //     cout << " var2: " << std::get<1>(iterative_costs[i]).first;
    //     cout << " val2: " << std::get<1>(iterative_costs[i]).second;
    //     cout << " h: " << std::get<2>(iterative_costs[i]) << endl;
    // }
    cout << "comb size: " << iterative_costs.size() << endl;
    // return DEAD_END;
    /* Set initial state cost to 0*/

    /* Begin iterations */
    bool state_changed = true;
    while (state_changed) {
        state_changed = false;
        for (size_t i = 0; i < g_operators.size(); i++) {
            const vector<Condition> &preconditions = g_operators[i].get_preconditions();
            bool applicable = false;
            int previous_cost = 0;
            if (preconditions.size() == 1) {
                // special case (just one precondition)
                // TODO create entries in map with duplicate varVal keys that represent states with of size 1, preconditions of size 1 will check upon them
                varVal varValKey = make_pair(preconditions[0].var,preconditions[0].val);
                // facts_map_it it = std::find_if(iterative_costs.begin(), iterative_costs.end(), 
                //     [varValKey](const std::pair<varVals,int> & t) -> bool { 
                //         return t.first.first == varValKey || t.first.second == varValKey;
                //     });
                for(auto const& [key, val] : iterative_costs) {
                    if (key.first == varValKey || key.second == varValKey) {
                            if (val != DEAD_END) {
                                // if any is not DEAD_END, then set applicable to true for that var1 val1 var2 val2
                                applicable = true;
                                // if the precondition is just one we keep the smallest cost
                                // TODO to avoid this loop we can create entries in the map where the two keys are duplicated
                                if (val < previous_cost) {
                                    previous_cost = val;
                                }
                            }
                        }
                }
                // std::for_each(iterative_costs.begin(), iterative_costs.end(),
                //     [&varValKey, &applicable, &previous_cost](const std::pair<varVals,int>& t) -> void { 
                //         if (t.first.first == varValKey || t.first.second == varValKey) {
                //             if (t.second != DEAD_END) {
                //                 // if any is not DEAD_END, then set applicable to true for that var1 val1 var2 val2
                //                 applicable = true;
                //                 if (t.second > previous_cost) {
                //                     previous_cost = t.second;
                //                 }
                //             }
                //         }
                //         // return t.first.first == varValKey || t.first.second == varValKey;
                //     });
                // if (cost != DEAD_END) {
                //     // if any is not DEAD_END, then set applicable to true for that var1 val1 var2 val2
                //     applicable = true;
                //     if (cost > previous_cost) {
                //         previous_cost = cost;
                //     }
                // }
            } else {
                // TODO what to do when precondtions size == 3
                for (size_t p = 0; p < preconditions.size(); p++) {
                    for (size_t p2 = 0; p2 < preconditions.size(); p2++) {
                        if (p != p2) {
                            // check p and p2 in iterative_costs map
                            varVals key = make_pair(make_pair(preconditions[p].var,preconditions[p].val), make_pair(preconditions[p2].var,preconditions[p2].var));
                            // find before getting cost
                            if (iterative_costs.count(key) != 0) {
                                int cost = iterative_costs[key];
                                if (cost != DEAD_END) {
                                    // if any is not DEAD_END, then set applicable to true for that var1 val1 var2 val2
                                    applicable = false;
                                    if (cost > previous_cost) {
                                        previous_cost = cost;
                                    }
                                }
                            }
                        } 
                    } 
                }
            }  
            if (applicable) {
                const vector<Effect> &effects = g_operators[i].get_effects();
                const int &possible_action_cost = g_operators[i].get_cost() + previous_cost;
                if (effects.size() == 1) {
                        // special case (just one effect)
                        varVal varValKey = make_pair(effects[0].var,effects[0].val);
                        facts_map_it it = std::find_if(iterative_costs.begin(), iterative_costs.end(), 
                            [varValKey](const std::pair<varVals,int> & t) -> bool { 
                                return t.first.first == varValKey || t.first.second == varValKey;
                            });
                            // TODO if there is more than one we should update the iterative costs of all (now just doing it to the first)
                        int cost = it->second;
                        if (cost == DEAD_END || possible_action_cost < cost) {
                            state_changed = true;
                            iterative_costs[it->first] = possible_action_cost;
                        }
                } else {
                    for (size_t e = 0; e < effects.size(); e++) {
                        for (size_t e2 = 0; e2 < effects.size(); e2++) {
                            if (e != e2) {
                                // check e and e2 in iterative_costs map
                                varVals key = make_pair(make_pair(effects[e].var,effects[e].val), make_pair(effects[e2].var,effects[e2].var));
                                // find before getting cost
                                if (iterative_costs.count(key) != 0) {
                                    int cost = iterative_costs[key];
                                    if (cost == DEAD_END || possible_action_cost < cost) {
                                        state_changed = true;
                                        iterative_costs[key] = possible_action_cost;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    cout << "goal checking : " << endl;
    for(facts_map_it it = iterative_costs.begin(); it != iterative_costs.end(); ++it) {
        cout << "var1: " << it-> first.first.first;
        cout << " val1: " << it-> first.first.second;
        cout << " var2: " << it-> first.second.first;
        cout << " val2: " << it-> first.second.second;
        cout << " h: " << it-> second << endl;
    }
    int max_cost = DEAD_END;
    // special case (just one goal)
    if (g_goal.size() == 1) {
        varVal varValKey = make_pair(g_goal[0].first,g_goal[0].second);
        facts_map_it it = std::find_if(iterative_costs.begin(), iterative_costs.end(), 
            [varValKey](const std::pair<varVals,int> & t) -> bool { 
                return t.first.first == varValKey || t.first.second == varValKey;
            });
        int cost = it->second;
        if (cost == DEAD_END){
            return DEAD_END;
        }
        else if (cost > max_cost) {
            max_cost = cost;
        }
    } else {
        for (size_t g = 0; g < g_goal.size(); g++) {
            for (size_t g2 = 0; g2 < g_goal.size(); g2++) {
                if (g != g2) {
                    varVals key = make_pair(make_pair(g_goal[g].first, g_goal[g].second), make_pair(g_goal[g2].first, g_goal[g2].second));
                    // find before getting cost
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
        return new CriticalPathHeuristic(opts);
}

static Plugin<Heuristic> _plugin("h_two", _parse);
