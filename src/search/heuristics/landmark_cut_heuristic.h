#ifndef LM_CUT_HEURISTIC_H
#define LM_CUT_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the LM-cut heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(lmcut())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class, you should implement the LM-cut heuristic. Note that the
// computation of a single LM-cut value encompasses multiple computations of
// h^{max}.

// NOTE: the h^{max} implementation that you use to compute LM-cut has to
// support action costs 0 and 1! (It is up to you to reuse/adapt the code
// in max_heuristic.h, or to (re-)implement h^{max} from scratch.)

class LMCutHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    LMCutHeuristic(const Options &options);
    ~LMCutHeuristic() = default;
};

#endif
