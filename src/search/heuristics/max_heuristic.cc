#include "max_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <map>
#include <unordered_set>




using namespace std;

MaxHeuristic::MaxHeuristic(const Options &opts)
    : Heuristic(opts) {
}

struct compare {
    bool operator() (const pair<int, int>& lhs, const pair<int, int>& rhs) const {
        return (lhs.first > rhs.first);
    }
};

void MaxHeuristic::initialize() {
    cout << "Initializing max heuristic..." << endl; 
}



vector<int> counter(g_operators.size());
const State& init_state = g_initial_state();
map <pair<int, int>, int> fac_mem_layer;
vector <int> action_mem_layer(g_operators.size());
queue <pair<int, int>> fact_schedule;
queue <int> action_schedule;
int timestep = 0;
int req_goal = g_goal.size();
vector <int> goal_mem_layer (g_goal.size());


static void goal_fact(pair<int, int>& fact) {
    for (int g_ind = 0; g_ind < g_goal.size(); ++g_ind) {
        if (g_goal[g_ind].first == fact.first && g_goal[g_ind].second == fact.second) {
            goal_mem_layer[g_ind] = timestep;
            req_goal--;
        }
    }
}

static void init_Membership() {
    fill(counter.begin(), counter.end(), 0); /*Init counter for each action with 0*/
  

    for (int i = 0; i < g_variable_domain.size(); ++i) {
        for (int j = 0; j < g_variable_domain[i]; ++j) {
            if (init_state[i] == j) {

                goal_fact(make_pair(i,j));

                fac_mem_layer.insert(make_pair(make_pair(i,j),0));
                for (int k = 0; k < g_operators.size(); ++k) {
                    const vector<Condition>& preconditions = g_operators[k].get_preconditions();
                    for (int m = 0; m < preconditions.size(); ++m) {
                        if (preconditions[m].var == i && preconditions[m].val == j) {
                            counter[k]++;
                            if (counter[k] == preconditions.size() ){                                 
                                action_mem_layer[k] = 0;
                                const vector<Effect>& effects = g_operators[k].get_effects();
                                    for (int e = 0; e < effects.size(); e++) {
                                        fact_schedule.push(make_pair(effects[e].var, effects[e].val));
                                           
                                    }
                            }
                        }
                    }
                }
            }
          
        }
    }

}



static bool doStep() {
    int q_size = fact_schedule.size(); /* pop all scheduled fact*/
    if (q_size == 0||req_goal==0) {
        /*No new facts were scheduled, so we can't achieve new facts OR we have achieved all goal facts*/
        return true; 
    }
    timestep++;
    for (int i = 0; i < q_size; ++i) {
        pair <int, int >fact= fact_schedule.front();

        goal_fact(fact);

        fac_mem_layer.insert(make_pair(fact, timestep));
        fact_schedule.pop();
        if (req_goal == 0) {
            return true; /*All goals are achieved*/
        }

        for (int op = 0; op < g_operators.size(); ++op) {
            const vector<Condition>& preconditions = g_operators[op].get_preconditions();
            if (preconditions.size() < counter[op]) {
                /*Check whether action is already exhausted*/
                for (int m = 0; m < preconditions.size(); ++m) {
                    if (preconditions[m].var == fact.first && preconditions[m].val == fact.second) {
                        counter[op]++;
                        if (counter[op] == preconditions.size()) {
                            action_mem_layer[op] = timestep;
                            const vector<Effect>& effects = g_operators[op].get_effects();
                            for (int e = 0; e < effects.size(); e++) {
                                fact_schedule.push(make_pair(effects[e].var, effects[e].val));
                            }
                        }
                    }
                }
            }

        }
    }
    return false;
    

}


int MaxHeuristic::compute_heuristic(const State &state) {
    // TODO implementation
    init_Membership();
    bool temp = doStep();
    while (temp) {
        temp = doStep();
    }
    if (req_goal > 0) {
        return DEAD_END;
    }
    else {
        int max = -1;
        for (int i = 0; i < g_goal.size(); ++i) {
            int temp=fac_mem_layer[make_pair(g_goal[i].first, g_goal[i].second)];
            if (temp>max) {
                max = temp;

            }
        }
        return max;
    }

}
/*
State MaxHeuristic::regr(const State &state, const Operator &op) {
    const vector<Condition>& preconditions = op.get_preconditions();
    const vector<Effect>& effects = op.get_effects();
}*/

static Heuristic *_parse(OptionParser &parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new MaxHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hmax", _parse);
