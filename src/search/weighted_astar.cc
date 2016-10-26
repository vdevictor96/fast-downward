#include "weighted_astar.h"

#include "globals.h"
#include "g_evaluator.h"
#include "heuristic.h"
#include "option_parser.h"
#include "plugin.h"
#include "pruning_method.h"
#include "successor_generator.h"
#include "sum_evaluator.h"
#include "weighted_evaluator.h"
#include "standard_scalar_open_list.h"

#include <cassert>
#include <cstdlib>
#include <set>


using namespace std;

WeightedAstar::WeightedAstar(
    const Options &opts)
    : SearchEngine(opts),
      reopen_closed_nodes(opts.get<bool>("reopen_closed")),
      helpful_actions(opts.get<bool>("helpful_actions")),
      open_list(opts.get<OpenList<StateID> *>("open"))
{
    if (opts.contains("f_eval")) {
        f_evaluator = opts.get<ScalarEvaluator *>("f_eval");
    } else {
        f_evaluator = nullptr;
    }
    if (opts.contains("pruning")) {
        pruning = opts.get<PruningMethod *>("pruning");
    } else {
        pruning = nullptr;
    }
}

void WeightedAstar::initialize()
{
    cout << "Conducting best first search"
         << (reopen_closed_nodes ? " with" : " without")
         << " reopening closed nodes, (real) bound = " << bound
         << endl;
    if (helpful_actions) {
        cout << "using only helpful actions" << endl;
    }
    if (pruning != nullptr) {
        pruning->initialize();
    }
    assert(open_list != NULL);

    set<Heuristic *> hset;
    open_list->get_involved_heuristics(hset);

    for (set<Heuristic *>::iterator it = hset.begin(); it != hset.end(); ++it) {
        search_progress.add_heuristic(*it);
    }

    // add heuristics that are used in the f_evaluator. They are usually also
    // used in the open list and hence already be included, but we want to be
    // sure.
    if (f_evaluator) {
        f_evaluator->get_involved_heuristics(hset);
    }

    heuristic = *hset.begin();

    assert(heuristic != 0);

    const State &initial_state = g_initial_state();

    heuristic->evaluate(initial_state);

    open_list->evaluate(0, false);
    search_progress.inc_evaluated_states();
    search_progress.inc_evaluations();

    if (open_list->is_dead_end()) {
        cout << "Initial state is a dead end." << endl;
    } else {
        search_progress.get_initial_h_values();
        if (f_evaluator) {
            f_evaluator->evaluate(0, false);
            search_progress.report_f_value(f_evaluator->get_value());
        }
        search_progress.check_h_progress(0);
        SearchNode node = search_space.get_node(initial_state);
        node.open_initial(heuristic->get_value());

        open_list->insert(initial_state.get_id());
    }
}


void WeightedAstar::statistics() const
{
    search_progress.print_statistics();
    search_space.statistics();
    if (pruning != nullptr) {
        pruning->print_statistics();
    }
}

SearchStatus WeightedAstar::step()
{
    pair<SearchNode, bool> n = fetch_next_node();
    if (!n.second) {
        return FAILED;
    }
    SearchNode node = n.first;

    State s = node.get_state();
    if (check_goal_and_set_plan(s)) {
        return SOLVED;
    }

    vector<const Operator *> applicable_ops;

    g_successor_generator->generate_applicable_ops(s, applicable_ops);

    // TODO implement support for helpful actions here
    // use the helpful_actions variable to check if helpful actions are enabled.

    if (pruning != nullptr) {
        pruning->prune_operators(s, applicable_ops);
    }

    for (size_t i = 0; i < applicable_ops.size(); ++i) {
        const Operator *op = applicable_ops[i];

        assert(op->is_applicable(s));

        if ((node.get_real_g() + op->get_cost()) >= bound) {
            continue;
        }

        State succ_state = g_state_registry->get_successor_state(s, *op);
        search_progress.inc_generated();

        SearchNode succ_node = search_space.get_node(succ_state);

        // Previously encountered dead end. Don't re-evaluate.
        if (succ_node.is_dead_end()) {
            continue;
        }

        // update new path
        if (succ_node.is_new()) {
            /*
                Note that we must call reach_state for the
                heuristic for its side effects.
            */
            heuristic->reach_state(s, *op, succ_state);
        }

        if (succ_node.is_new()) {
            // We have not seen this state before.
            // Evaluate and create a new node.
            heuristic->evaluate(succ_state);

            succ_node.clear_h_dirty();
            search_progress.inc_evaluated_states();
            search_progress.inc_evaluations();

            // Note that we cannot use succ_node.get_g() here as the
            // node is not yet open. Furthermore, we cannot open it
            // before having checked that we're not in a dead end. The
            // division of responsibilities is a bit tricky here -- we
            // may want to refactor this later.
            open_list->evaluate(node.get_g() + get_adjusted_cost(*op), false);
            bool dead_end = open_list->is_dead_end();
            if (dead_end) {
                succ_node.mark_as_dead_end();
                search_progress.inc_dead_ends();
                continue;
            }

            int succ_h = heuristic->get_heuristic();

            succ_node.open(succ_h, node, op);

            open_list->insert(succ_state.get_id());

        } else if (succ_node.get_g() > node.get_g() + get_adjusted_cost(*op)) {
            // We found a new cheapest path to an open or closed state.
            if (reopen_closed_nodes) {
                if (succ_node.is_closed()) {
                    search_progress.inc_reopened();
                }
                succ_node.reopen(node, op);
                heuristic->set_evaluator_value(succ_node.get_h());

                open_list->evaluate(succ_node.get_g(), false);

                open_list->insert(succ_state.get_id());
            } else {
                // if we do not reopen closed nodes, we just update the parent pointers
                // Note that this could cause an incompatibility between
                // the g-value and the actual path that is traced back
                succ_node.update_parent(node, op);
            }
        }
    }

    return IN_PROGRESS;
}

pair<SearchNode, bool> WeightedAstar::fetch_next_node()
{
    while (true) {
        if (open_list->empty()) {
            cout << "Completely explored state space -- no solution!" << endl;

            SearchNode dummy_node = search_space.get_node(g_initial_state());
            return make_pair(dummy_node, false);
        }
        vector<int> last_key_removed;
        StateID id = open_list->remove_min(0);

        State s = g_state_registry->lookup_state(id);
        SearchNode node = search_space.get_node(s);

        if (node.is_closed()) {
            continue;
        }

        node.close();
        assert(!node.is_dead_end());
        update_jump_statistic(node);
        search_progress.inc_expanded();
        return make_pair(node, true);
    }
}

void WeightedAstar::dump_search_space()
{
    search_space.dump();
}

void WeightedAstar::update_jump_statistic(const SearchNode &node)
{
    if (f_evaluator) {
        heuristic->set_evaluator_value(node.get_h());
        f_evaluator->evaluate(node.get_g(), false);
        int new_f_value = f_evaluator->get_value();
        search_progress.report_f_value(new_f_value);
    }
}

void WeightedAstar::print_heuristic_values(const vector<int> &values) const
{
    for (size_t i = 0; i < values.size(); ++i) {
        cout << values[i];
        if (i != values.size() - 1) {
            cout << "/";
        }
    }
}

static SearchEngine *_parse_wastar(OptionParser &parser)
{
    parser.document_synopsis(
        "A* search",
        "A* is a special case of eager best first search that uses g+h "
        "as f-function. "
        "We break ties using the evaluator. Closed nodes are re-opened.");

    parser.add_option<ScalarEvaluator *>("eval", "evaluator for h-value");
    parser.add_option<int>("w", "heuristic weight", "1");
    parser.add_option<bool>("helpful_actions", "use helpful actions", "false");
    parser.add_option<PruningMethod *>("pruning", "use a pruning method", "",
                                       OptionFlags(false));

    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();

    WeightedAstar *engine = 0;
    if (!parser.dry_run()) {
        GEvaluator *g = new GEvaluator();
        vector<ScalarEvaluator *> sum_evals;
        sum_evals.push_back(g);
        ScalarEvaluator *eval = opts.get<ScalarEvaluator *>("eval");
        if (opts.get<int>("w") == 1) {
            sum_evals.push_back(eval);
        } else {
            WeightedEvaluator *w = new WeightedEvaluator(
                eval,
                opts.get<int>("w"));
            sum_evals.push_back(w);
        }
        ScalarEvaluator *f_eval = new SumEvaluator(sum_evals);

        // use eval for tiebreaking
        std::vector<ScalarEvaluator *> evals;
        evals.push_back(f_eval);
        evals.push_back(eval);
        OpenList<StateID> *open = \
                                  new TieBreakingOpenList<StateID>(evals, false, false);

        opts.set("open", open);
        opts.set("f_eval", f_eval);
        opts.set("reopen_closed", true);
        engine = new WeightedAstar(opts);
    }

    return engine;
}


static Plugin<SearchEngine> _plugin_astar("wastar", _parse_wastar);
