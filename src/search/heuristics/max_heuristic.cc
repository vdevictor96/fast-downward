#include "max_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <map>
#include <unordered_set>
#include <queue>




using namespace std;

MaxHeuristic::MaxHeuristic(const Options& opts)
    : Heuristic(opts) {
    req_goal = g_goal.size();
    timestep = 0;
}

void MaxHeuristic::initialize() {
    cout << "Initializing max heuristic..." << endl;
    /*Init goal set to quickly find out whether an achieved fact is a goal fact*/

    goal_set.resize(g_variable_domain.size());
    fill(goal_set.begin(), goal_set.end(), -1);

    for (int i = 0; i < g_goal.size(); ++i) {
        goal_set[g_goal[i].first] = g_goal[i].second;
    }
    int sum = 0;
    for (auto& g : g_variable_domain) {
        hash_arr.emplace_back(sum);
        sum += g;
    }
    fact_set.resize(hash_arr.back() + g_variable_domain.back());


    for (int i = 0; i < hash_arr.back() + g_variable_domain.back(); i++) {
        unordered_set <int> app_ops;
        fact_set.emplace_back(make_pair(false, app_ops));
    }
    //For each operator add its identifier to the fact that is added by the operator 
    for (int i = 0; i < g_operators.size(); i++) {
        const vector<Condition>& preconditions = g_operators[i].get_preconditions();
        for (auto& pre : preconditions) {
            pair <int, int> pre_fact = make_pair(pre.var, pre.val);
            fact_set[compute_hash(pre_fact)].second.insert(i);
        }
    }

    counter.resize(g_operators.size());


}




void MaxHeuristic::init_layers_and_counter(const State& state) {
    /*Init counter with zero and action layer and goal layer with infinity represented as -1*/
    req_goal = g_goal.size();

    fill(counter.begin(), counter.end() - 1, 0);

    for (int i = 0; i < g_variable_domain.size(); i++) {
        pair <int, int> state_fact = make_pair(i, state[i]);
        fact_schedule.push(state_fact);
        fact_set[compute_hash(state_fact)].first = true;

        if (goal_set[i] == state[i]) {
            req_goal--;
        }


    }

}
int MaxHeuristic::compute_hash(pair<int, int>& x)
{
    return hash_arr[x.first] + x.second;
}

void MaxHeuristic::achieve_facts(int& pos) {

    const vector<Effect>& effects = g_operators[pos].get_effects();
    for (auto& eff : effects) {
        pair<int, int> eff_fact = make_pair(eff.var, eff.val);
        if (!fact_set[compute_hash(eff_fact)].first) {
            /* Fact is achieved for the first time. Schedule Fact and add to seen facts*/

            fact_set[compute_hash(eff_fact)].first = true;
            fact_schedule.push(eff_fact);

            if (goal_set[eff.var] == eff.val) {
                req_goal--;
            }
        }
    }
}


void MaxHeuristic::small_iteration() {
    int schedule_size = fact_schedule.size();
    pair<int, int> fact;
    while (schedule_size != 0) {
        schedule_size--;
        fact = fact_schedule.front();
        fact_schedule.pop();


        for (auto pre_op : fact_set[compute_hash(fact)].second) {
            int pre_size = g_operators[pre_op].get_preconditions().size();
            if (counter[pre_op] < pre_size) {
                counter[pre_op]++;
                if (counter[pre_op] == pre_size) {
                    achieve_facts(pre_op);
                }
            }
        }
    }

}



bool MaxHeuristic::doStep() {
    timestep++;
    small_iteration();
    int q_size = fact_schedule.size(); /*Number of new scheduled facts for given iteration*/
    if (req_goal == 0 || q_size == 0) {
        return false; /*No new facts were scheduled, so we can't achieve new facts OR we have achieved all goal facts*/
    }
    return true; /*Facts were scheduled and not all goal facts are achieved*/

}
void MaxHeuristic::queue_clear(queue<pair<int, int>>& q) {
    while (!q.empty()) {
        q.pop();
    }
}
void MaxHeuristic::op_queue_clear(queue< int>& q)
{
    while (!q.empty()) {
        q.pop();
    }
}



int MaxHeuristic::compute_heuristic(const State& state) {


    //Clear and reinit relevant data structures
    queue_clear(fact_schedule);
    op_queue_clear(operator_queue);
    //fill(fact_set.begin(), fact_set.end() - 1, false);

    //Set fact set to zero but keep op identifiers
    for (auto& fact : fact_set) {
        fact.first = false;
    }
    timestep = 0;
    req_goal = g_goal.size();
    init_layers_and_counter(state);



    bool progress = true;
    while (progress) {
        progress = doStep();
    }
    if (req_goal > 0) {
        return DEAD_END; /* Not all goal facts could be achieved*/
    }
    else {

        return timestep;
    }

}


static Heuristic* _parse(OptionParser& parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new MaxHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hmax", _parse);