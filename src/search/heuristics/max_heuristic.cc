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
}

void MaxHeuristic::initialize() {
    cout << "Initializing max heuristic..." << endl;
    /*Init goal set to quickly find out whether an achieved fact is a goal fact*/
    for (int i = 0; i < g_goal.size(); ++i) {
        goal_set.insert(g_goal[i]);
    }
    int sum = 0;
    for (auto g : g_variable_domain) {
        sum += g;
        hash_arr.push_back(sum); 
    }
    fact_set.assign(hash_arr.back() + g_variable_domain.back(), false);

}




 void MaxHeuristic::init_layers_and_counter(const State& state) {
    /*Init counter with zero and action layer and goal layer with infinity represented as -1*/
    req_goal = g_goal.size();

    counter.assign(g_operators.size(), 0);


    for (int i = 0; i < g_operators.size(); ++i) {
        operator_queue.push(make_pair(g_operators[i], i));
    }

    for (int i = 0; i < g_variable_domain.size(); i++) {
        fact_schedule.push(make_pair(i, state[i]));
        fact_set[compute_hash(make_pair(i,state[i]))]=true;
        /*Insert fact to goal layer if it is contained in the goal_set*/
        if (goal_set.find(make_pair(i, state[i])) != goal_set.end()) {
            req_goal--;
        }

    }

}
 int MaxHeuristic::compute_hash(pair<int,int>& x)
 {
     return hash_arr[x.first] + x.second;
 }

void MaxHeuristic::achieve_facts(int &pos) {

    const vector<Effect>& effects = g_operators[pos].get_effects();
    //cout << "effect size " << effects.size() << endl;
    for (auto eff:effects) {
        //cout << "considered fact " << effects[e].var << " " << effects[e].val << endl;
        if (!fact_set[compute_hash(make_pair(eff.var, eff.val))]) {
            /* Fact is achieved for the first time. Schedule Fact and add to seen facts*/
            fact_set[compute_hash(make_pair(eff.var, eff.val))] = true;
            fact_schedule.push(make_pair(eff.var, eff.val));

            if (goal_set.find(make_pair(eff.var, eff.val)) != goal_set.end()) {
                /*Achieved fact is goal fact*/
                //cout << "achieved goal facts " << effects[e].var << " " << effects[e].val << endl;
                req_goal--;
            }
        }


    }
}



void MaxHeuristic::small_iteration() {

    int schedule_size = fact_schedule.size();


    while (schedule_size != 0) {
        schedule_size--;
        pair <int, int >fact = fact_schedule.front();
        fact_schedule.pop();
        //cout << "current fact " << fact.first << " " << fact.second << endl;

        /* Iterate over all actions preconditions*/
        int operator_queue_size = operator_queue.size();
        while (operator_queue_size > 0) {

            operator_queue_size--;
            pair<Operator, int> cur_op = operator_queue.front();
            operator_queue.pop();
            const vector<Condition>& preconditions = cur_op.first.get_preconditions();
            //cout << "current operator " << cur_op.second << endl;


            if (preconditions.size() == 0) {
                achieve_facts(cur_op.second);
                //cout << "achieved operator with no pre " << cur_op.second << endl;
            }
            else if (counter[cur_op.second] < preconditions.size()) {

                /*Check whether action is already exhausted*/
                /*This should actually be improved, as the for-loop continues even if the fact is already found*/
                for (int m = 0; m < preconditions.size(); ++m) {
                    if (preconditions[m].var == fact.first && preconditions[m].val == fact.second) {
                        counter[cur_op.second]++;
                        //cout << "fact in precondition " << fact.first << " " << fact.second <<endl ;
                        if (counter[cur_op.second] == preconditions.size()) {
                            achieve_facts(cur_op.second);
                            //cout << "achieved operator with pre" << cur_op.second << endl;
                        }
                    }
                }
                if (counter[cur_op.second] != preconditions.size()) {
                    operator_queue.push(cur_op);
                }
            }


        }

    }

}



bool MaxHeuristic::doStep() {
    timestep++;
    //cout << "timestep " << timestep << endl;
    small_iteration();
    int q_size = fact_schedule.size(); /*Number of new scheduled facts for given iteration*/
    if (req_goal == 0 || q_size == 0) {
        return false; /*No new facts were scheduled, so we can't achieve new facts OR we have achieved all goal facts*/
    }



    //cout << "timestep " << timestep << " Goals to achieve after It " << req_goal << " Scheduled fact for given it " <<q_size<< endl;

    return true; /*Facts were scheduled and not all goal facts are achieved*/

}
void MaxHeuristic::queue_clear(queue<pair<int, int>> q){
    while (!q.empty()) {
        q.pop();
   }
}
void MaxHeuristic::op_queue_clear(queue<pair<Operator, int>> q)
{
    while (!q.empty()) {
        q.pop();
    }
}



int MaxHeuristic::compute_heuristic(const State& state) {



    // TODO implementation
    queue_clear(fact_schedule);
    op_queue_clear(operator_queue);
    for (auto fact : fact_set) {
        fact = false; //Init fact_set by setting all facts to false
    }
    timestep = 0;
    req_goal = g_goal.size();


    init_layers_and_counter(state);


    //cout << "Goals to achieve that are not already achieved" << req_goal << endl;


    bool progress = true;
    while (progress) {
        //cout << "timestep " << timestep << " Goals to achieve before it " << req_goal << " Fact that are updated " << fact_schedule.size() << endl;
        progress = doStep();

    }
    //cout << "goals not achieved " << req_goal << endl;
    //cout << "h_value " << timestep << endl;
    if (req_goal > 0) {
        return DEAD_END; /* Not all goal facts could be achieved*/
    }
    else {
        return timestep;
    }

}
/*
State MaxHeuristic::regr(const State &state, const Operator &op) {
    const vector<Condition>& preconditions = op.get_preconditions();
    const vector<Effect>& effects = op.get_effects();
}*/

static Heuristic* _parse(OptionParser& parser) {
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new MaxHeuristic(opts);
}

static Plugin<Heuristic> _plugin("hmax", _parse);