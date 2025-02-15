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

// Combinations without order, with duplicates 
std::vector<std::vector<int>> getCombinations(int n, int depth) {
    std::vector<std::vector<int>> combinations;
    if (depth == 0) {
        combinations.push_back({});
        return combinations;
    }
    for (int i = 0; i < n; i++) {
        std::vector<std::vector<int>> subcombinations = getCombinations(n, depth - 1);
        for (auto& subcombination : subcombinations) {
            subcombination.push_back(i);
            std::sort(subcombination.begin(), subcombination.end());
            if (std::find(combinations.begin(), combinations.end(), subcombination) == combinations.end()) {
                combinations.push_back(subcombination);
            }
        }
    }
    return combinations;
}

// Order the possible clause learning:
//  clausels are first learned if they are closer it is to the problem.
bool compareVectors(const std::vector<int>& a, const std::vector<int>& b) {
    int sumA = 0;
    for (int x : a) sumA += x;
    int sumB = 0;
    for (int x : b) sumB += x;
    return sumA > sumB;
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
    // But I deleted the "break", because it will lead to really inefficent paths.
    // And added a list of all opend states, to there are no loops in opening nodes
    //
    // If Hill Climbing failed to find a new best heuristic,
    // we used first breath to find the nex solution. 
    // Therefor we use open_list, there is only one state, if we found a lower heuristic value.
    // otherwise every child state will be added to the open_list
    // and only deleted, when a new best heuristic have been found.
    // 
    // To save the right plan, we track indivually the plan for every opend state.
    // If we find a new best heuristic we append the indivual plan to the real plan and delete every indivual plan.

    std::deque<StateID> open_list; // the breadth-first list also called frontier
    std::vector<const Operator*> ops; // all available operations for the current state
    std::deque<std::vector<const Operator*>> plans; // multiple plans, without the plan, that is tracked for every opend state
    std::vector<const Operator*> current_plan; // the plan for current_state
    std::vector<StateID> opend;
    std::vector<StateID> dead_ends;
    std::vector<std::vector<int>> clausels;
    std::vector<StateID> clausel;
    std::deque<StateID> state_plan;
    bool clause_learning_failed = false;

    state_plan = {};
    open_list.push_back(current_state.get_id());
    plans.push_back(plan);

    // only stop if there is no more possibilities
    while (!open_list.empty()) {
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
            if ((!(std::find(opend.begin(), opend.end(), next_state.get_id()) != opend.end()))) {
                evaluate(current_state, ops[i], next_state); // get statistics (heuristic value of successor)
                opend.push_back(next_state.get_id());
                current_plan.push_back(ops[i]); // create plan for next_state
                if (heuristic->is_dead_end()) {
                    dead_ends.push_back(next_state.get_id());
                } 
                else if (heuristic->get_heuristic() < current_h) {
                    opend.clear();
                    opend.insert(opend.end(), clausel.begin(), clausel.end());
                    opend.insert(opend.end(), dead_ends.begin(), dead_ends.end());
                    current_g = plan.size() + current_plan.size();
                    current_h = heuristic->get_heuristic(); // update current_h
                    open_list.clear();
                    plans.clear();
                    open_list.push_front(next_state.get_id()); // best_state will be expanded as next
                    state_plan.push_back(current_state.get_id());
                    plans.push_front(current_plan);
                    clause_learning_failed = false;
                    clausel.clear();
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
        if (open_list.empty()) { // Idea of AI Course 1, clause learning to make this ehc compatible with every domain.
            //cout << clausels << endl;
            if (clause_learning_failed) {
                return FAILED;
            }
            if (clausels.empty()) {
                clausels = getCombinations(state_plan.size(), state_plan.size());
                std::sort(clausels.begin(), clausels.end(), compareVectors);
                clause_learning_failed = true;
            }
            for (int i = 0; i < state_plan.size(); i++) {
                clausel.clear();
                clausel.push_back(state_plan[clausels.back()[i]]);
            }
            //cout << "__" << endl;
            clausels.pop_back();
            plan.clear();
            plans.clear();
            plans.push_back(plan);
            open_list.clear();
            open_list.push_back(g_initial_state().get_id());
            state_plan.clear();
            opend.clear();
            opend.insert(opend.end(), clausel.begin(), clausel.end());
            opend.insert(opend.end(), dead_ends.begin(), dead_ends.end());

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

