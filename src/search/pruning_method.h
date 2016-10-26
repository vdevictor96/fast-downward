#ifndef PRUNING_METHOD_H
#define PRUNING_METHOD_H

#include <stdlib.h>
#include <vector>

// Interface for successor prunining methods, used in search (e.g. partial order
// reduction and dominance pruning).
//
// IMPORTANT: remove_pruned_operator(state, ops) must remove all operators from ops
// that should NOT be considered in search!

class Operator;
class State;

class PruningMethod
{

    size_t num_unpruned_successors_generated;
    size_t num_pruned_successors_generated;

protected:
    PruningMethod();

    virtual void remove_pruned_operators(const State &state,
                                         std::vector<const Operator *> &ops) = 0;
public:
    virtual ~PruningMethod() = default;
    virtual void initialize() = 0;
    void prune_operators(const State &state,
                         std::vector<const Operator *> &ops);
    virtual void print_statistics() const;
};

#endif
