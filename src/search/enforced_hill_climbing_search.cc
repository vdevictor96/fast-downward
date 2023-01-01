#include "enforced_hill_climbing_search.h"

#include "heuristic.h"
#include "plugin.h"
#include "successor_generator.h"
#include "utilities.h"

#include <cassert>

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

    //Pruning possiblities:
    //- Symmetry breaking
    //- Duplicate deletion
    /*
    for (const Operator* op : plan) {

        auto it = std::find(ops.begin(), ops.end(), op);
        if (it != plan.end()) {
            plan.erase(it);
        }
    }*/

#ifndef NDEBUG
    for (const Operator* op : ops) {
        assert(op->is_applicable(state));
    }
#endif
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

    std::deque<State> open_list; // the breadth-first list also called frontier
    std::vector<const Operator*> ops; // all available operations for the current state
    std::deque<std::vector<const Operator*>> plans;
    // intitialize
    open_list.push_back(current_state);
    plans.push_back(plan);
    // only stop if there is no more possibilities
    while (!open_list.empty()) {
        current_state = open_list.front(); // initialize new current_state for hill_climbing algorithm
        plan = plans.front(); // initialize plan for the current_state
        open_list.pop_front();
        plans.pop_front();
        ops.clear();
        get_applicable_operators(current_state, ops); // get the available operations
        for (int i = 0; i < ops.size(); i++) { // loop through every possible operation
            State next_state = g_state_registry->get_successor_state(current_state, *ops[i]); // state after applying the operator
            evaluate(current_state, ops[i], next_state); // get statistics (heuristic value of successor)
            plan.push_back(ops[i]); // create plan for next_state
            if (heuristic->get_heuristic() < current_h) {
                current_h = heuristic->get_heuristic(); // update current_h
                open_list.clear();
                open_list.push_back(current_state); // next_state will be expanded as next
                plans.clear();
                plans.push_back(plan);
                i = ops.size(); // stop iterating successors
                if (test_goal(next_state)) { // next_state is goal state
                    cout << "Found a goal state!" << endl;
                    assert(is_plan(plan));
                    set_plan(plan);
                    return SOLVED;
                }
            }
            else {
                open_list.push_back(next_state); // no new best heuristic have been found, breath first search will continue 
                plans.push_back(plan);
            }
            plan.pop_back(); // reset plan to current_state plan.
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

