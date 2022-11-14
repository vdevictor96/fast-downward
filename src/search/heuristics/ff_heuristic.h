#ifndef FF_HEURISTIC_H
#define FF_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the h^{FF} heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(ff())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


// In this class you are supposed to implement the h^{FF} heuristic using either
// h^{add} or h^{max} as best-supporter function. (Thus, before you start
// implementing h^{FF}, a working implementation of h^{add}/h^{max} is strongly
// recommended.)
//
// Helpful actions: in get_helpful_actions you should only add those
// operators of the relaxed plan to result that are applicable in the state on which
// compute_heuristic is called.
//
// NOTE: evaluate(state) (which in turn calls compute_heuristic(state)) has to
// be called before accessing the helpful actions through
// get_helpful_actions(result)!
//
// Hint: you can use the function is_relaxed_plan(state, relaxed_plan) to verify
// the correctness of your implementation.
//

#include <vector>

class FFHeuristic : public Heuristic
{
protected:
    // NOTE: We do not pass a reference to the relaxed plan as relaxed_plan is
    // modified internally. The function does not assume a particular order of actions.
    static bool is_relaxed_plan(const State &state,
                                std::vector<const Operator *> relaxed_plan);

    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    FFHeuristic(const Options &options);
    ~FFHeuristic() = default;
    virtual void get_helpful_actions(std::vector<const Operator *> &result);
};

#endif
