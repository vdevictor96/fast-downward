#include "pruning_method.h"

#include <iostream>

using namespace std;

PruningMethod::PruningMethod()
    : num_unpruned_successors_generated(0),
      num_pruned_successors_generated(0)
{
}

void PruningMethod::prune_operators(const State &state,
                                    std::vector<const Operator *> &ops)
{
    num_unpruned_successors_generated += ops.size();
    remove_pruned_operators(state, ops);
    num_pruned_successors_generated += ops.size();
}

void PruningMethod::print_statistics() const
{
    cout << "total successors before pruning: "
         << num_unpruned_successors_generated << endl
         << "total successors after pruning: "
         << num_pruned_successors_generated << endl;
}
