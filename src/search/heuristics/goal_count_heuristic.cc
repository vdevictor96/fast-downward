#include "goal_count_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"


using namespace std;

GoalCountHeuristic::GoalCountHeuristic(const Options &opts)
    : Heuristic(opts)
{
}

void GoalCountHeuristic::initialize()
{
    cout << "Initializing goal counting heuristic..." << endl;
}

int GoalCountHeuristic::compute_heuristic(const State &state) 
{
    int unachieved_goals = 0;
    for (size_t i = 0; i < g_goal.size(); ++i) {
        if (state[g_goal[i].first] != g_goal[i].second) {
            unachieved_goals++;
        }
    }
    return unachieved_goals;
}

static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new GoalCountHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("gc", _parse);
