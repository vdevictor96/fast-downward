#ifndef EAGER_SEARCH_H
#define EAGER_SEARCH_H

#include <vector>

#include "evaluator.h"
#include "state.h"
#include "search_engine.h"
#include "search_progress.h"
#include "search_space.h"
#include "timer.h"

#include "open_list.h"

// Usage example: the command line option for using wastar with weight x and
// heuristic with name h is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(h(), w=x)"
// So, for the blind heuristic h = blind and weight x = 1 it is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(blind(), w=1)"
// For h^{FF} and weight 10
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(ff(), w=10)"
// And so on.
// To enable helpful actions you additionally need to pass helpful_actions=true,
// so for example running wastar with h^{FF}, weight 5, and helpful actions:
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(ff(), w=5, helpful_actions=true)"
// If you want to enable a pruning method, please check the file corresponding
// to the desired method for further details.
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


class Operator;
class Heuristic;
class Options;
class PruningMethod;
class ScalarEvaluator;

class WeightedAstar : public SearchEngine
{
    // Search Behavior parameters
    bool reopen_closed_nodes; // whether to reopen closed nodes upon finding lower g paths
    bool helpful_actions; // use helpful actions pruning
    PruningMethod *pruning; // the specified pruning method

    OpenList<StateID> *open_list;
    ScalarEvaluator *f_evaluator;

protected:
    SearchStatus step();
    std::pair<SearchNode, bool> fetch_next_node();
    void update_jump_statistic(const SearchNode &node);
    void print_heuristic_values(const std::vector<int> &values) const;

    Heuristic *heuristic;

    virtual void initialize();

public:
    WeightedAstar(const Options &opts);
    void statistics() const;

    void dump_search_space();
};

#endif
