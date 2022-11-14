#ifndef MAS_HEURISTIC_H
#define MAS_HEURISTIC_H

#include "../heuristic.h"

// Usage example: the command line option for using h^{MS} in astar is
// ./fast-downward.py [path-to-PDDL-problem-file] --search "wastar(merge-and-shrink())"
// Obviously, [path-to-PDDL-problem-file] has to be replaced by the actual path
// to the PDDL problem file.


/*
 The M&S heuristic incrementally builds a transition system (TS) that represents an abstraction
 of the planning task.

 For implementing the M&S heuristic you need to implement five main functions:
  1) Create atomic transition systems (one TS for each variable)
  2) Compute initial-state and goal distances on a TS
  3) Merge operation on two TSs (given two TS, return the merged TS)
  4) Shrink operation on a TS: given a TS, and a mapping from the states in
      that TS to a smaller set of states, return the shrunk TS
  5) Heuristic lookup: given a state, lookup the corresponding abstract state, and its heuristic value.

  1-4 are used in initialize() to construct the TS, 5 is used in compute_heuristic()

  We recommend to implement them in this order: 1, 2, 5, 3, 4.  1, 2, and 5 are sufficient
  to start testing (though it will be a very bad heuristic).

  For implementing this, you'll need to use two data structures: one for representing a
  TS, and the other to represent the mapping from states to abstract states.

  You are free of choosing any data structure that you like for the TSs. Note that it is
  not necessary to represent this as a graph in memory (this could be quite
  innefficient). It is sufficient to have a unique identifier for each node and
  representing the transitions as a <predecessor, op, successor> tuples.

  For representing the mapping from states to abstract states, the cascading tables
  representation is recommended.  This is described in detail in Section 4.3 of the paper
  Merge-and-Shrink Abstraction: A Method for Generating Lower Bounds in Factored State
  Spaces by Malte Helmert, Patrik Haslum, Joerg Hoffmann and Raz Nissim.

  Notice that we have not discussed at all the merge and shrink strategies, i.e., how to
  pick which two TS are merged (operation 3) and which mapping is applied in the shrinking
  (operation 4). In principle any choice (e.g., arbitrary or random) is possible and you
  can obtain full points independently of this decision. In practice, for M&S to be a good
  heuristic, clever criteria should be used to take these decisions. We'll give extra
  points depending on the strategies that you come up with.
*/




class MASHeuristic : public Heuristic
{
protected:
    virtual void initialize();
    virtual int compute_heuristic(const State &state);
public:
    MASHeuristic(const Options &options);
    ~MASHeuristic() = default;
};

#endif
