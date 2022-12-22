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
int PDBHeuristic::unrank(int r,int var,int ind) {
    int temp = r / N_ind_collection[ind][var];
    return temp % g_variable_domain[var];
}
int PDBHeuristic::rankState(const State& state, int ind) {
    int sum = 0;
    for (auto &i : pattern_collection[ind]) {
        sum = N_ind_collection[ind][i] * state[i];
    }
    return sum;
}

int PDBHeuristic::rank(vector <int> &s, int ind) {
    int sum = 0;
    for (auto &i:pattern_collection[ind]) {
        sum += N_ind_collection[ind][i] * s[i];
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

    closed_list_collection.resize(pattern_collection.size());
    list_collection.resize(pattern_collection.size());
    adjList_collection.resize(pattern_collection.size());
    //N_ind_collection.resize(pattern_collection.size());

    
    vector <int> n_arr(g_variable_domain.size());
    fill(n_arr.begin(), n_arr.end(), 0);
    for (auto &pat : pattern_collection) {
        int N = 1;
        int count = 0;
        N_ind_collection.push_back(n_arr);
        for (auto& i : pat) {
            N_ind_collection[count][i] = N;
            N *= g_variable_domain[i]; //Compute 
            
        }
        N_ind[count] = N;
        count++;
    }

        
    //Init PDB with -1 for all available patterns
    for (size_t i = 0; i < pattern_collection.size();i++) {
        vector<int> s(N_ind[i]);
        fill(s.begin(), s.end(), -1);
        PDB_collection.push_back(s);
    }

    //Find applicable operations for each pattern
    for (auto& pat : pattern_collection) {
        applicable_ops_collection.push_back(check_applicable_ops(pat));
    }
    int ind = 0;
    for (auto& pat : pattern_collection) {
        vector <int> s(g_variable_domain.size());
        for (int r = 0; r < N_ind[ind]; ++r) {
            for (auto& j : pat) {
                s[j] = unrank(r, j,ind);
            }
            if (goal_test(s)) {
                PDB_collection[ind][r] = 0;
                closed_list_collection[ind].insert(r);
                list_collection[ind].push(r);
            }
            for (auto& op : applicable_ops_collection[ind]) {
                if (op_applicable(op.first, s)) {
                    vector <int> s2 = s;
                    apply_operation(op.first, s2);
                    int r2 = rank(s2,ind);
                    if (adjList_collection[ind].find(r2) == adjList_collection[ind].end()) {
                        adjList_collection[ind].insert(make_pair(r2, r));
                    }
                    else {
                        adjList_collection[ind].at(r2).insert(r); //The graph is backwards
                    }
                }
            }
        }
        Dijkstra(ind);
        ind++;

    }

    

}



void PDBHeuristic::initialize()
{
    cout << "Initializing PDB heuristic..." << endl;

    if (!m_test_pattern.empty()) {
        // Use m_test_pattern
        pattern_collection.push_back(m_test_pattern);
        computePDB();

        // TODO implementation
    }
    else {
        // Use automatic method
        pattern_collection.push_back ( { 1,4 });
        computePDB();

        // TODO implementation
    }

}



void PDBHeuristic::Dijkstra(int ind) { //This is actual breadth-first search with unit cost
    int h = 0;


    while (!list_collection[ind].empty()) {
        h++;
        int size = list_collection[ind].size();
        while (size > 0) {
            int front = list_collection[ind].front();
            closed_list_collection[ind].insert(front);
            if (adjList_collection[ind].find(front) != adjList_collection[ind].end()) {
                for (auto& neighbour : adjList_collection[ind].at(front)) {
                    if (closed_list_collection[ind].find(neighbour) == closed_list_collection[ind].end()) {
                        list_collection[ind].push(neighbour);
                        PDB_collection[ind][neighbour] = h;
                    }
                }
            }


            list_collection[ind].pop();
            size--;
        }


    }
}


int PDBHeuristic::compute_heuristic(const State& state)
{
    vector<int> pattern_rank(pattern_collection.size());
    for (size_t i = 0; i < pattern_collection.size(); i++) {
        pattern_rank.push_back(rankState(state, i));
    }

    // Canocial pattern database heuristic
    int max = 0;
    for (size_t i= 0;i<pattern_collection.size();i++) {
        int h=PDB_collection[i][pattern_rank.at(i)];
        if (h == -1) {
            return DEAD_END;
        }else if (h>max){
            max = h;
        }
    }
    return max;
}

bool PDBHeuristic::check_orthogonality() {


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
