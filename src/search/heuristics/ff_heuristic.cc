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

typedef std::pair<int,int> varVal;
template <typename T>
std::vector<const T*> convertFrom(std::vector<T>& source)
{
    std::vector<const T*> target(source.size());
    std::transform(source.begin(), source.end(), target.begin(), [](T& t) { return &t; });
    return target;
}

int FFHeuristic::compute_heuristic(const State &state)
{
    /* Init vector with all the variable values with infinite */
    std::vector<std::vector<int> > iterative_costs;
    iterative_costs.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        iterative_costs[var].resize(g_variable_domain[var], DEAD_END);
        iterative_costs[var][state[var]] = 0;
    }
     /* Init vector with all best-supporter functions */
    std::vector<std::vector<pair<Operator, bool>>> supporter_func;
    supporter_func.resize(g_variable_domain.size());
    for (unsigned var = 0; var < g_variable_domain.size(); var++) {
        supporter_func[var].resize(g_variable_domain[var], make_pair(g_operators[0], false));
        // supporter_func[var].resize(g_variable_domain[var], );
        // supporter_func[var][state[var]] = NULL;
        
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
                const Operator possible_action = g_operators[i];
                const int &possible_action_cost = possible_action.get_cost() + previous_cost;
                for (size_t e = 0; e < effects.size(); e++) {
                    if (iterative_costs[effects[e].var][effects[e].val] == DEAD_END || possible_action_cost < iterative_costs[effects[e].var][effects[e].val]) {
                        state_changed = true;
                        iterative_costs[effects[e].var][effects[e].val] = possible_action_cost;
                        supporter_func[effects[e].var][effects[e].val] = make_pair(possible_action, false);
                    }
                    
                }
            }
        }
    }
    /* Calculate relaxed plan */
    std::set<varVal> open;
    for (size_t g = 0; g < g_goal.size(); g++) {
        // add all goals that are not in initial state to the open set
        if (state[g_goal[g].first] != g_goal[g].second) {
            open.insert(open.end(), make_pair(g_goal[g].first, g_goal[g].second));
        }
    }
    std::set<varVal> closed;
    std::vector<Operator> relaxed_plan;
    /* Iterate over open set */
    while (open.size() > 0) {
        // for open as vector varVal const g = open.front();
        varVal const g = *(open.begin());
        open.erase(open.begin());
        closed.insert(g);
        auto &sf = supporter_func[g.first][g.second];
         // just add the action if it has not been added before
        if (!sf.second) {
            relaxed_plan.insert(relaxed_plan.end(), sf.first);
            // update the flag of used in relaxed plan in the vector 
            sf.second = true;
        }
        const vector<Condition> &preconditions = sf.first.get_preconditions();
        for (size_t p = 0; p < preconditions.size(); p++) {
            varVal precondition = make_pair(preconditions[p].var, preconditions[p].val);
            // if (!(closed.count(precondition) == 0)) {
            //     cout << "preconditions in closed"  << endl;
            // }
            // else if (state[preconditions[p].var] == preconditions[p].val) {
            //     cout << "preconditions in init state"  << endl;
            // } else if ((find(open.begin(), open.end(), precondition) != open.end())) {
            //     cout << "preconditions in open already"  << endl;
            // } else {
            //     count++;
            // }
            // not in initial state
            if (state[preconditions[p].var] != preconditions[p].val &&
                // nor in closed
                closed.count(precondition) == 0 &&
                // nor in opened already
                // TODO check if this is necessary
                open.count(precondition) == 0
                // for open as vector (find(open.begin(), open.end(), precondition) == open.end()
            ) 
            {
                // add to open vector at the end.
                open.insert(open.end(), precondition);
            }
        }
    }
    /* Check goals are reached */
    for (size_t g = 0; g < g_goal.size(); g++) {
        int cost = iterative_costs[g_goal[g].first][g_goal[g].second];
        if (cost == DEAD_END){
            return DEAD_END;
        }
    }
    // std::vector<const Operator *> r_plan = convertFrom(relaxed_plan);
    // cout << is_relaxed_plan(state, r_plan) << endl;
    /* Return size of relaxed_plan */
    return relaxed_plan.size();
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
