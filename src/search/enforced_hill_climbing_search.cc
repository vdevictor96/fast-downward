#include "enforced_hill_climbing_search.h"

#include "heuristic.h"
#include "plugin.h"
#include "successor_generator.h"
#include "utilities.h"

#include <cassert>
#include <algorithm>
#include <numeric>



using namespace std;

EnforcedHillClimbingSearch::EnforcedHillClimbingSearch(
    const Options& opts)
    : SearchEngine(opts),
    helpful_actions(opts.get<bool>("helpful_actions")),
    heuristic(opts.get<Heuristic* >("heuristic")),
    current_state(g_initial_state()),
    current_g(0)
{
    search_progress.add_heuristic(heuristic);
}

EnforcedHillClimbingSearch::~EnforcedHillClimbingSearch()
{


}

void EnforcedHillClimbingSearch::initialize()
{
    assert(heuristic != NULL);
    cout << "Conducting Enforced Hill Climbing Search" << endl;

    if (helpful_actions) {
        cout << "Performing helpful actions pruning" << endl;
    }

    evaluate(current_state, NULL, current_state);

    if (heuristic->is_dead_end()) {
        cout << "Initial state is a dead end, no solution" << endl;
        if (heuristic->dead_ends_are_reliable()) {
            exit_with(EXIT_UNSOLVABLE);
        } else {
            exit_with(EXIT_UNSOLVED_INCOMPLETE);
        }
    }

    search_progress.get_initial_h_values();

    current_h = heuristic->get_heuristic();
}

void EnforcedHillClimbingSearch::evaluate(const State& parent,
    const Operator* op, const State& state)
{
    search_progress.inc_evaluated_states();

    if (op != NULL) {
        heuristic->reach_state(parent, *op, state);
    }
    heuristic->evaluate(state);

    search_progress.inc_evaluations();
    // current_g is not the g-value of state, but rather of current_state
    // since it is only used for statistics output, it doesn't matter too much
    search_progress.check_h_progress(current_g);
}

void EnforcedHillClimbingSearch::get_applicable_operators(const State& state,
    vector<const Operator*>& ops)
{
    g_successor_generator->generate_applicable_ops(state, ops);

    search_progress.inc_expanded();
    search_progress.inc_generated_ops(ops.size());

    // TODO: implement helpful actions prunining here
    // Use the helpful_actions class variable to determine whether helpful
    // actions pruning is enabled or not.

#ifndef NDEBUG
    for (const Operator* op : ops) {
        assert(op->is_applicable(state));
    }
#endif
}

// Function to generate all combinations of the elements of a vector
std::vector<std::vector<int>> getCombinations(int n) {
    std::vector<std::vector<int>> combinations;
    std::vector<int> v(n);
    std::iota(v.begin(), v.end(), 0);  // Fill v with the numbers 1, 2, ..., n
    do {
        combinations.push_back(v);
    } while (std::next_permutation(v.begin(), v.end()));
    return combinations;
}


// this function is called only once for the input planning task.
// if current_state is a goal state, search will break out of the while loop,
// output some statistics and store the plan.
// if enforced hill-climbing did not find a solution, you need to "return FAILED;"
SearchStatus EnforcedHillClimbingSearch::hill_climbing()
{
    // functions that you have to use:

    // get_applicable_operators(state,  ops):
    // see description in header file

    // evaluate(parent, operator*, state):
    // after a call to evaluate(..), you can query the heuristic by using
    // heuristic->is_dead_end(): returns true if the heuristic detected state as a dead-end
    // heuristic->get_heuristic(): returns the heuristic value of state

    // NOTE: if for an evaluated state, heuristic->is_dead_end() returns true,
    // then heuristic->get_heuristic() returns DEAD_END.

    // g_state_registry->get_successor_state(state, operator):
    // returns the outcome state when applying operator to state

    // g_state_registry->lookup_state(id): if you stored the StateID of a state, you can
    // retrieve the corresponding state by a call to this function.

    // NOTE: To augment the statistics output, you can call search_progess->inc_generated()
    // whenever you generated a new state and search_progess->inc_dead_ends() if
    // you detected a new dead-end state

    // remember to keep the variables
    //      current_state
    //      current_h
    //      current_g
    //      plan
    // up-to-date

    // IMPORTANT: after each breadth-first-search for a state with smaller
    // heuristic value, you have to make sure that you append the entire path
    // from the old state to the new state to plan! Otherwise, the computed plan
    // will not be valid!

    // basic construct:
    // https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume28/coles07a-html/node5.html
    //
    // If Hill Climbing failed to find a new best heuristic,
    // we used first breath to find the nex solution. 
    // Therefor we use open_list, there is only one state, if we found a lower heuristic value.
    // otherwise every child state will be added to the open_list
    // and only deleted, when a new best heuristic have been found.
    // 
    // To save the right plan, we track indivually the plan for every opend state.
    // If we find a new best heuristic we append the indivual plan to the real plan and delete every indivual plan.
    // 
    // Additionaly we created a open_list restriction, no state can be opend twice.
    // For nomystery and sokoban we created a bracktracking method from AI 1.
    // By using Clause Learning we can track back to the junction to a wrong path.

    std::deque<StateID> open_list; // the breadth-first list also called frontier
    std::vector<const Operator*> ops; // all available operations for the current state
    std::deque<std::vector<const Operator*>> plans; // multiple plans, without the plan, that is tracked for every opend state
    std::vector<const Operator*> current_plan; // the plan for current_state
    std::deque<StateID> all_opend; // all states we already used will not be added to the open_list
    std::deque<StateID> opend; // states that have been opend and had a new min heuristic
    std::vector<StateID> clause_learning; // learned wrong paths, so nomystery and sokoban will be work with this version of ehc
    std::vector<const Operator*> best_plan;
    StateID best_state = current_state.get_id();
    bool clause_learning_failed = false;
    std::vector<vector<int>> possibilities;

    open_list.push_back(current_state.get_id());
    all_opend.push_back(current_state.get_id());
    plans.push_back(plan);

    // only stop if there is no more possibilities
    while (!open_list.empty()) {
        best_plan.clear();
        current_state = g_state_registry->lookup_state(open_list.front()); // initialize new current_state for hill_climbing algorithm
        open_list.pop_front();
        current_plan.clear(); 
        if (!plans.empty()) { // only if there havent been found a new min heuristic
            current_plan = plans.front(); // initialize plan for the current_state
            plans.pop_front();
        }
        ops.clear();
        get_applicable_operators(current_state, ops); // get the available operations
        for (int i = 0; i < ops.size(); i++) { // loop through every possible operation
            State next_state = g_state_registry->get_successor_state(current_state, *ops[i]); // state after applying the operator
            if ((!(std::find(all_opend.begin(), all_opend.end(), next_state.get_id()) != all_opend.end()))
                && (!(std::find(clause_learning.begin(), clause_learning.end(), next_state.get_id()) != clause_learning.end()))) {
                all_opend.push_back(next_state.get_id());
                evaluate(current_state, ops[i], next_state); // get statistics (heuristic value of successor)
                current_plan.push_back(ops[i]); // create plan for next_state
                if (heuristic->get_heuristic() < current_h) {
                    current_g = plan.size() + current_plan.size();
                    current_h = heuristic->get_heuristic(); // update current_h
                    best_plan = current_plan;
                    best_state = next_state.get_id();
                    if (test_goal(next_state)) { // next_state is goal state
                        for (int j = 0; j < current_plan.size(); j++) {
                            plan.push_back(current_plan[j]);
                        }
                        cout << "Found a goal state!" << endl;
                        assert(is_plan(plan));
                        set_plan(plan);
                        return SOLVED;
                    }
                }
                else {
                    open_list.push_back(next_state.get_id()); // no new best heuristic have been found, breath first search will continue 
                    plans.push_back(current_plan);
                }
                current_plan.pop_back(); // reset plan to current_state plan.
            }
        }
        if(!best_plan.empty()) {
            for (int a = 0; a < best_plan.size(); a++) {
                plan.push_back(best_plan[a]);
            }
            plans.clear();
            open_list.clear();
            open_list.push_back(best_state); // best_state will be expanded as next
            opend.push_back(best_state);
            clause_learning_failed = false;
            possibilities.clear();
        }
        if (open_list.empty()) { // Idea of AI Course 1, clause learning to make this ehc compatible with every domain.
            if (clause_learning_failed) {
                return FAILED;
            }
            if(possibilities.empty()){
                {
                    possibilities = getCombinations(opend.size());
                }
                clause_learning_failed = true;
            }
            for (int i = 0; i < opend.size(); i++) {
                clause_learning.clear();
                clause_learning.push_back(opend[possibilities.back()[i]]);
            }
            //clause_learning.push_back(opend.front());
            possibilities.pop_back();
            plan.clear();
            plans.clear();
            plans.push_back(plan);
            open_list.clear();
            open_list.push_back(g_initial_state().get_id());
            opend.clear();
            opend.push_back(g_initial_state().get_id());
            all_opend.clear();
        }
    }
    return FAILED;
}

auto EnforcedHillClimbingSearch::is_plan(const Plan& plan) -> bool
{
    auto state = g_initial_state();
    for (const auto* op : plan) {
        // verify that all operators in the plan are applicable
        if (!op->is_applicable(state))
            return false;
        state = g_state_registry->get_successor_state(state, *op);
    }
    // verify that the final state is a goal state
    return test_goal(state);
}

void EnforcedHillClimbingSearch::statistics() const
{
    search_progress.print_statistics();
}

static SearchEngine* _parse(OptionParser& parser)
{
    parser.document_synopsis("Hill-climbing", "");
    parser.add_option<Heuristic*>("eval", "evaluator for h-value");
    parser.add_option<bool>("helpful_actions", "", "false");

    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    EnforcedHillClimbingSearch* engine = 0;
    if (!parser.dry_run()) {
        Heuristic* eval = opts.get<Heuristic*>("eval");
        opts.set("heuristic", eval);

        engine = new EnforcedHillClimbingSearch(opts);
    }

    return engine;
}

static Plugin<SearchEngine> _plugin("ehc", _parse);

