#include "max_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <map>
#include <unordered_set>




using namespace std;

vector<int> counter;
map <pair<int, int>, int> fac_mem_layer;
vector <int> action_mem_layer;
queue <pair<int, int>> fact_schedule;
queue <int> action_schedule;
int timestep;
int req_goal; 
vector <int> goal_mem_layer;
vector <bool> operator_mark; /*Check if this is necessary. This should be redundant with the .mark() operation*/

MaxHeuristic::MaxHeuristic(const Options &opts)
    : Heuristic(opts) {
}


void MaxHeuristic::initialize() {
    cout << "Initializing max heuristic..." << endl; 

 
    /*Try to get a better way of initializing the vector*/
}

static void goal_fact(pair<int, int> fact) {
    /*Check if an (var,val) is an goal fact. Quite inefficient right now*/
    for (int g_ind = 0; g_ind < g_goal.size(); ++g_ind) {
        if (goal_mem_layer[g_ind] == -1) {
            if (g_goal[g_ind].first == fact.first && g_goal[g_ind].second == fact.second) {
                goal_mem_layer[g_ind] = timestep;
                /*cout << fact.first << " " << fact.second << endl;*/
                req_goal--;
            }

        }

    }
}

static void init_layers_and_counter(const State& state) {
    /*Init counter with zero and action layer and goal layer with infinity represented as -1*/
    action_mem_layer.assign(g_operators.size(), -1);
    counter.assign(g_operators.size(), 0);
    goal_mem_layer.assign(g_goal.size(), -1);
    operator_mark.assign(g_operators.size(), false);
    
    assert(action_mem_layer[g_operators.size()-1] == -1);
    assert(goal_mem_layer[g_goal.size()-1] == -1);
    assert(counter[g_operators.size() - 1] == 0);
  
    /*Init fact member layer for given state*/
    for (int i = 0; i < g_variable_domain.size(); ++i) {
        for (int j = 0; j < g_variable_domain[i]; ++j) {
            if (state[i] == j) {
                /* fact from search state. Check if it already contains an goal fact 
                and init fact layer with 0 and schedule fact, as it is already achieved */
                goal_fact(make_pair(i, j));
                fac_mem_layer.insert(make_pair(make_pair(i, j), 0));
                fact_schedule.push(make_pair(i, j));
            }
            else {
                /*fact is not in search state and thus layer is initialized with infinity*/
                fac_mem_layer.insert(make_pair(make_pair(i, j), -1));
            }
        }
    }

}



static void update_action_layer_and_push_facts(int pos) {
    action_mem_layer[pos]= timestep;
    const vector<Effect>& effects = g_operators[pos].get_effects();
    for (int e = 0; e < effects.size(); e++) {
        /*Update goal layer*/
        goal_fact(make_pair(effects[e].var, effects[e].val));
        /*Schedule fact*/
        fact_schedule.push(make_pair(effects[e].var, effects[e].val));
        /*Update fact member layer*/
        fac_mem_layer.insert(make_pair(make_pair(effects[e].var, effects[e].val), timestep));
    }
}


static void layer_iteration() {
    int q_size = fact_schedule.size();
    for (int i = 0; i < q_size; ++i) {
        pair <int, int >fact = fact_schedule.front();

        /* Iterate over all actions preconditions*/
        for (int op = 0; op < g_operators.size(); ++op) {
            const vector<Condition>& preconditions = g_operators[op].get_preconditions();
            if (!operator_mark[op]) {
                assert(operator_mark[op] == false);
                if (counter[op] < preconditions.size()) {
                    /*Check whether action is already exhausted*/
                    /*This should actually be improved, as the for-loop continues even if the fact is already found*/
                    for (int m = 0; m < preconditions.size(); ++m) {
                        if (preconditions[m].var == fact.first && preconditions[m].val == fact.second) {
                            counter[op]++;
                        }
                    }
                }
                else if (counter[op] == preconditions.size()) {
                    operator_mark[op] = true;
                    assert(preconditions.size() == counter[op]);
                    update_action_layer_and_push_facts(op);
                }
            }



        }
        fact_schedule.pop();
    }

}







static bool doStep() {
    timestep++;
    layer_iteration();
   
    int q_size = fact_schedule.size(); /*Number of new scheduled facts for given iteration*/
    //cout << "timestep " << timestep << " Goals to achieve after It " << req_goal << " Scheduled fact for given it " <<q_size<< endl;
    if (req_goal == 0||q_size==0) {
        return false; /*No new facts were scheduled, so we can't achieve new facts OR we have achieved all goal facts*/
    }
    return true; /*Facts were scheduled and not all goal facts are achieved*/
 
}


int MaxHeuristic::compute_heuristic(const State &state) {
    // TODO implementation
    timestep = 0;
    req_goal = g_goal.size();
    cout << "Goals to achieve" << req_goal << endl;
    
    init_layers_and_counter(state);

    cout << "Goals to achieve that are not already achieved" << req_goal << endl;
    if (test_goal(state)) {
        return 0;
   }
   
    bool progress = true;
    
    while (progress) {
        //cout << "timestep " << timestep << " Goals to achieve before it " << req_goal << " Fact that are updated " << fact_schedule.size() << endl;
        progress = doStep();

    }
    cout << "Goals that cannot be achieved" << req_goal << endl;
   
    
    if (req_goal > 0) {
        return DEAD_END; /* Not all goal facts could be achieved*/
    }else {
        assert(req_goal == 0);
        int max = *max_element(goal_mem_layer.begin(),goal_mem_layer.end());
        assert(max != -1);
        assert(max != 0);
        cout << "h-value " << max << endl;
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
