#include "pattern_database_heuristic.h"

#include "../globals.h"
#include "../operator.h"
#include "../option_parser.h"
#include "../plugin.h"
#include "../state.h"

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>



using namespace std;


PDBHeuristic::PDBHeuristic(const Options &opts)
    : Heuristic(opts),
      m_test_pattern(opts.contains("test_pattern") ?
		     opts.get_list<int>("test_pattern") : vector<int>()) {
    N_ind.assign(g_variable_domain.size(), 0);
}
int PDBHeuristic::unrank(int r,int var) {
    int temp = r / N_ind[var];
    return temp % g_variable_domain[var];
}
int PDBHeuristic::rankState(const State& state) {
    int sum = 0;
    for (auto &i : patterns) {
        sum = N_ind[i] * state[i];
    }
    return sum;
}

int PDBHeuristic::rank(vector <int> &s) {
    int sum = 0;
    for (auto &i:patterns) {
        sum += N_ind[i] * s[i];
    }
    return sum;
}
bool PDBHeuristic::goal_test(vector <int> &s) {
    for (auto &i : g_goal) {
        if (find(patterns.begin(), patterns.end(), i.first) != patterns.end()) {
            if (s[i.first] != i.second) {
                return false;
            }
        }
    }
    return true;
}
vector <pair<Operator,int>> PDBHeuristic::check_applicable_ops(vector <int> &pat) {
    vector <pair<Operator,int>> pattern_ops;
    int count = 0;
    for (auto &op : g_operators) {
        for (auto &eff : op.get_effects()) {
            if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
                pattern_ops.push_back(make_pair(op,count));
                break;           
            }              
        }
        count++;

    }
    return pattern_ops;
}

bool PDBHeuristic::op_applicable(Operator &op,vector<int> &s) {
    for (auto &i: op.get_preconditions()) {
        if (find(patterns.begin(), patterns.end(), i.var) != patterns.end()) {
            if (s[i.var] != i.val) {
                return false;
            }
        }

    }
    return true;
}
void PDBHeuristic::apply_operation(Operator& op, vector <int>& s) {
    for (auto &eff : op.get_effects()) {
        if (find(patterns.begin(), patterns.end(), eff.var) != patterns.end()) {
            s[eff.var] = eff.val;
        }
    }
}

void PDBHeuristic::computePDB() {
    int N = 1;
    
    
    for (auto &i:patterns) {
        N_ind[i] = N;
        N *= g_variable_domain[i]; //Compute 
        
    }
    PDB.assign(N, -1);
    vector <pair<Operator,int>> applicable_ops=check_applicable_ops(patterns);
    vector <int> s(g_variable_domain.size());

    for (int r = 0; r < N; ++r) {     
        for (auto &j:patterns) {
            s[j] = unrank(r, j);
        }
        if (goal_test(s)) {
            PDB[r] = 0;
            list.push(r);
        }

        for (auto &op:applicable_ops) {
            if (op_applicable(op.first,s)) {
                vector <int> s2 = s;
                apply_operation(op.first, s2);
                int r2 = rank(s2);
                if (r2 == 18 || r2 == 19 || r2 == 20) {
                    cout << "s2" << endl;

                }
                if (adjList.find(r2) == adjList.end()) {
adjList.insert(make_pair(r2, r));
                }
                else {
                    adjList.at(r2).insert(r); //The graph is backwards
                }


            }
        }
    }
    Dijkstra();

}



void PDBHeuristic::initialize()
{
    cout << "Initializing PDB heuristic..." << endl;

    if (!m_test_pattern.empty()) {
        // Use m_test_pattern
        patterns = m_test_pattern;
        computePDB();

        // TODO implementation
    }
    else {
        // Use automatic method
        patterns = { 1,4 };
        computePDB();

        // TODO implementation
    }

}



void PDBHeuristic::Dijkstra() { //This is actual breadth-first search with unit cost
    int h = 0;

    int init = list.size();
    while (init > 0) {
        closed_list.insert(list.front());
        list.push(list.front());
        list.pop();
        init--;
    }


    while (!list.empty()) {
        h++;
        int size = list.size();
        while (size > 0) {
            int front = list.front();
            closed_list.insert(front);
            if (adjList.find(front) != adjList.end()) {
                for (auto& neighbour : adjList.at(front)) {
                    if (closed_list.find(neighbour) == closed_list.end()) {
                        list.push(neighbour);
                        PDB[neighbour] = h;
                    }
                }
            }


            list.pop();
            size--;
        }


    }
}


int PDBHeuristic::compute_heuristic(const State& state)
{
    int r = rankState(state);

    // TODO implementation
    int h = PDB[r];
    if (h == -1) {
        return DEAD_END;
    }
    else {
        return h;
    }
}

bool PDBHeuristic::check_orthogonality() {

    for (auto& pat : pattern_collection) {
        applicable_ops_collection.push_back(check_applicable_ops(pat));
    }
    for (int i = 0; i < applicable_ops_collection.size(); ++i) {
        for (int j = i + 1; j < applicable_ops_collection.size(); ++j){
            for (auto& op1 : applicable_ops_collection[i])

                for (auto &op2:applicable_ops_collection[j]) {
                    if (op1.second == op2.second) {
                        return false;
                    }

                }

        }
    }

    return true;
    
}



static Heuristic *_parse(OptionParser &parser)
{
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<int>("test_pattern", "", "[]", OptionFlags(false));
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    } else {
        return new PDBHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("pdb", _parse);
