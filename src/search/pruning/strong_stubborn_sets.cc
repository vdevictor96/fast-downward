#include "strong_stubborn_sets.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"

#include <algorithm>
#include <cassert>

using namespace std;


StrongStubbornSets::StrongStubbornSets()
{
    cout << "pruning method: strong stubborn sets" << endl;
}

void StrongStubbornSets::initialize()
{
    cout << "initializing Strong Stubborn Sets" << endl;
}

void StrongStubbornSets::remove_pruned_operators(
    const State &/*state*/, vector<const Operator *> &/*ops*/)
{

    // TODO implement this
}

static PruningMethod *_parse(OptionParser &parser)
{
    parser.document_synopsis(
        "Stubborn sets simple",
        "Stubborn sets represent a state pruning method which computes a subset "
        "of applicable operators in each state such that completeness and "
        "optimality of the overall search is preserved. As stubborn sets rely "
        "on several design choices, there are different variants thereof. "
        "The variant 'StubbornSetsSimple' resolves the design choices in a "
        "straight-forward way. For details, see the following papers: "
        "Yusra Alkhazraji, Martin Wehrle, Robert Mattmueller, Malte Helmert"
        "A Stubborn Set Algorithm for Optimal Planning"
        "http://ai.cs.unibas.ch/papers/alkhazraji-et-al-ecai2012.pdf"
        "Proceedings of the 20th European Conference on Artificial Intelligence "
        "(ECAI 2012)"
        "891-892"
        "IOS Press 2012"
        "Martin Wehrle, Malte Helmert"
        "Efficient Stubborn Sets: Generalized Algorithms and Selection Strategies"
        "http://www.aaai.org/ocs/index.php/ICAPS/ICAPS14/paper/view/7922/8042"
        "Proceedings of the 24th International Conference on Automated Planning "
        " and Scheduling (ICAPS 2014)"
        "323-331"
        "AAAI Press, 2014");

    if (parser.dry_run()) {
        return 0;
    }

    return new StrongStubbornSets();
}

static Plugin<PruningMethod> _plugin("strong_stubborn_sets", _parse);
