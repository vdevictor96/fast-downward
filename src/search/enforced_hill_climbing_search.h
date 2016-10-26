#ifndef ENFORCED_HILL_CLIMBING_SEARCH_H
#define ENFORCED_HILL_CLIMBING_SEARCH_H

#include "globals.h"
#include "operator.h"
#include "search_engine.h"
#include "search_node_info.h"
#include "search_progress.h"
#include "search_space.h"
#include "state.h"

#include <map>
#include <vector>

// Usage example: the command line option for using enforced hill climbing
// using heuristic with name h is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "ehc(h())"
// So, for the h^{FF} heuristic
// ./fast-downward.py [path-to-PDDL-problem-file] --search "ehc(ff())"
// For h^{add}
// ./fast-downward.py [path-to-PDDL-problem-file] --search "ehc(hadd())"
// And so on.
// To enable helpful actions you additionally need to pass helpful_actions=true,
// so for example running enforced hill climbing with h^{FF} and helpful actions:
// ./fast-downward.py [path-to-PDDL-problem-file] --search "ehc(ff(), helpful_actions=true)"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


// The implementation of enforced hill climbing goes here. In particular
// consider the various TODO tags distributed across the code in
// enforced_hill_climbing.cc

class Options;

class EnforcedHillClimbingSearch : public SearchEngine
{

private:
    bool helpful_actions;

    Heuristic *heuristic;

    // the following four variables have to be kept up-to-date during the search

    // the state that is to be expanded next
    State current_state; // is initialized to the initial state

    // h-value of current_state
    int current_h;      // is initialized to the heuristic value of the initial state

    // g-value of current_state, only required for statistics output
    int current_g; // is initialized to 0

    // the sequence of actions that leads from the initial state of the task to current_state
    Plan plan;  // Plan is just an abbreviation for std::vector<const Operator*>

    SearchStatus hill_climbing();

    // puts all operators that are applicable in state into the ops vector
    void get_applicable_operators(const State &state,
                                  std::vector<const Operator *> &ops);

    // this should be called with the parent of the state that you want to evaluate, the op that
    // reached the state from its parent and the state itself
    // it will make the required calls to the heuristic and output some statistics
    void evaluate(const State &parent, const Operator *op, const State &state);

protected:
    virtual void initialize();

    virtual SearchStatus step()
    {
        return hill_climbing();
    };

public:
    EnforcedHillClimbingSearch(const Options &opts);
    virtual ~EnforcedHillClimbingSearch();

    virtual void statistics() const;
};

#endif
