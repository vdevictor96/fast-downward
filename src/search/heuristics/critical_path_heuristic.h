#ifndef CRITICAL_PATH_HEURISTIC_H
#define CRITICAL_PATH_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the h^{2} heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(h_two())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class, you are supposed to implement the h^{2} heuristic. We suggest
// you implement the table based computation of h^{2} as presented in the
// lecture.
//
// NOTE: To get an efficient implementation you should try to precompute as many
// things as possible in the initialize() function. In particular, you should
// precompute all pairs of facts in the initialize() method, as well as store
// a relation of facts/pairs to actions indicating which action can be
// used to achieve which facts/pairs.

class CriticalPathHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    CriticalPathHeuristic(const Options &options);
    ~CriticalPathHeuristic() = default;
};

#endif
