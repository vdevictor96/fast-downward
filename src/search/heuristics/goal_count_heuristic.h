#ifndef GOAL_COUNT_HEURISTIC_H
#define GOAL_COUNT_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using the goal count heuristic in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(gc())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.

// In this class you should implement the goal counting heuristic. The goal
// count heuristic assigns a state s to the number of unachieved goal facts in s.
// This is, given a state s and the goal G, h^{GC}(s) = |G \setminus s|.

class GoalCountHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    GoalCountHeuristic(const Options &options);
    ~GoalCountHeuristic() = default;
};

#endif
