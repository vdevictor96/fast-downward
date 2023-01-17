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


PDBHeuristic::PDBHeuristic(const Options& opts)
    : Heuristic(opts),
    m_test_pattern(opts.contains("test_pattern") ?
        opts.get_list<int>("test_pattern") : vector<int>()) {

}
int PDBHeuristic::unrank(int r, int var, int ind) {
    int temp = r / N_ind_collection[ind][var];
    return temp % g_variable_domain[var];
}
int PDBHeuristic::rankState(const State& state, int ind) {
    int sum = 0;
    for (auto& i : pattern_collection[ind]) {
        sum = N_ind_collection[ind][i] * state[i];
    }
    return sum;
}

int PDBHeuristic::rank(vector <int>& s, int ind) {
    int sum = 0;
    for (auto& i : pattern_collection[ind]) {
        sum += N_ind_collection[ind][i] * s[i];
    }
    return sum;
}
bool PDBHeuristic::goal_test(vector <int>& s, vector <int>& pat) {
    for (auto& i : g_goal) {
        if (find(pat.begin(), pat.end(), i.first) != pat.end()) {
            if (s[i.first] != i.second) {
                return false;
            }
        }
    }
    return true;
}
vector <pair<Operator, int>> PDBHeuristic::check_applicable_ops(vector <int>& pat) {
    vector <pair<Operator, int>> pattern_ops;
    int count = 0;
    for (auto& op : g_operators) {
        for (auto& eff : op.get_effects()) {
            if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
                pattern_ops.push_back(make_pair(op, count));
                break;
            }
        }
        count++;
    }
    return pattern_ops;
}

bool PDBHeuristic::op_applicable(Operator& op, vector<int>& s, vector <int>& pat) {
    for (auto& i : op.get_preconditions()) {
        if (find(pat.begin(), pat.end(), i.var) != pat.end()) {
            if (s[i.var] != i.val) {
                return false;
            }
        }

    }
    return true;
}
void PDBHeuristic::apply_operation(Operator& op, vector <int>& s, vector<int>& pat) {
    for (auto& eff : op.get_effects()) {
        if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
            s[eff.var] = eff.val;
        }
    }
}

void PDBHeuristic::computePDB() {

    closed_list_collection.resize(pattern_collection.size());
    list_collection.resize(pattern_collection.size());
    adjList_collection.resize(pattern_collection.size());
    N_ind.assign(pattern_collection.size(), 0);
    //N_ind_collection.resize(pattern_collection.size());


    vector <int> n_arr(g_variable_domain.size());
    fill(n_arr.begin(), n_arr.end(), 0);
    int count = 0;
    for (auto& pat : pattern_collection) {
        int N = 1;
        N_ind_collection.push_back(n_arr);
        for (auto& i : pat) {
            N_ind_collection[count][i] = N;
            N *= g_variable_domain[i]; //Compute 
        }
        N_ind[count] = N;
        count++;
    }


    //Init PDB with -1 for all available patterns
    for (size_t i = 0; i < pattern_collection.size(); i++) {
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


        //for debugging
        vector <int> s_goal(g_variable_domain.size());
        for (int r = 0; r < N_ind[ind]; ++r) {
            for (auto& j : pat) {
                s[j] = unrank(r, j, ind);
            }
            if (goal_test(s, pat)) {
                s_goal = s;
            }
        }


        
        for (int r = 0; r < N_ind[ind]; ++r) {
            for (auto& j : pat) {
                s[j] = unrank(r, j, ind);
            }

            if (goal_test(s, pat)) {
                PDB_collection[ind][r] = 0;
                closed_list_collection[ind].insert(r);
                list_collection[ind].push(r);
            }
            for (auto& op : applicable_ops_collection[ind]) {
                if (op_applicable(op.first, s, pat)) {
                    vector <int> s2 = s;
                    apply_operation(op.first, s2, pat);
                    /*if (s2 == s_goal) {
                        cout << " " << endl;
                    }*/
                    int r2 = rank(s2, ind);
                    if (adjList_collection[ind].find(r2) == adjList_collection[ind].end()) {

                        adjList_collection[ind].insert({ r2, {r} });
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
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        
       // pattern_collection.push_back({0,5,23,43,8,15});
        computePDB();
        if (pattern_collection.size() > 1) {
            create_orthogonality_graph();
            max_cliques_ind = find_max_cliques();
        }
        

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
    vector<int> pattern_rank;
    for (size_t i = 0; i < pattern_collection.size(); i++) {
        pattern_rank.push_back(rankState(state, i));
    }

    //standard case with one pattern
    if (pattern_collection.size() == 1) {
       // cout << PDB_collection[0][pattern_rank.at(0)]<<endl;
        return PDB_collection[0][pattern_rank.at(0)];
    }
    else {
        // Canocial pattern database heuristic
        int max = 0;
        int h;
        int sum;
        for (auto& clique : max_cliques_ind) {
            sum = 0;
            for (auto& pat_ind : clique) {
                h= PDB_collection[pat_ind][pattern_rank.at(pat_ind)];
                //This could be a problem because of my way of computing the PDB Space
                if (h == -1) {
                    return DEAD_END;
                }
                else {
                    sum += h;
                }
            }
            if (sum > max) {
                max = sum;
            }
        }
        return max;
    }


}

vector <int> PDBHeuristic::naive_pattern_selection() {
    vector <int> pat;
    //add random goal variable. The number 10 is also a random hyperparameter.
    for (int i = 0; i < g_goal.size(); i += 10) {
        int g = rand() % g_goal.size();
        bool exists = false;
        for (auto& p : pat) {
            if (p == g_goal[g].first) {
                exists = true;
            }
        }
        if (!exists) {
            pat.push_back(g_goal[g].first);
        } 
    }
    //add some random variables
    for (int i = 0; i < g_variable_domain.size(); i += 10) {
        int g = rand() % g_variable_domain.size();
        bool exists = false;
        for (auto& p : pat) {
            if (p == g) {
                exists = true;
            }
        }
        if (!exists) {
            pat.push_back(g);
        }
    }
    return pat;
}

void PDBHeuristic::create_orthogonality_graph() {
    //orthogonality_graph.resize(pattern_collection.size());
    //Create a matrix with the  patterns as nodes and arcs means orthogonality between patterns
    vector<bool> row(pattern_collection.size(), true);
    for (int i = 0; i < pattern_collection.size(); i++) {
        orthogonality_graph.push_back(row);
    }

    for (int i = 0; i < applicable_ops_collection.size(); ++i) {
        for (int j = i + 1; j < applicable_ops_collection.size(); ++j) {

            //Check whether two applicable operators are the same for two patterns. If there are, they are by definition not orthogonal
            for (auto& op1 : applicable_ops_collection[i]) {

                for (auto& op2 : applicable_ops_collection[j]) {
                    if (op1.second == op2.second) {
                        orthogonality_graph[i][j] = false;
                        orthogonality_graph[j][i] = false; //For symmetry
                        break;
                    }
                }
                if (orthogonality_graph[i][j] == false) {
                    break;
                }
            }
        }
    }
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        orthogonality_graph[i][i] = false;
    }

}


vector <int> PDBHeuristic::find_adjoinable_vertex(unordered_set<int> Q) {
   
    vector <int> adjoinable_nodes;
    unordered_set <int> temp;
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        
        if (find(Q.begin(), Q.end(), i) == Q.end()) {
            temp = Q;
            temp.insert(i);
            if (is_clique(temp)) {
                adjoinable_nodes.push_back(i);
            }
        }
    }
    return adjoinable_nodes;
}

vector<unordered_set<int>> PDBHeuristic::expand_clique(unordered_set<int>Q) {
    //The method seems to work. It returns all max cliques that can be expanded from an input set
    unordered_set<int> temp;
    vector<unordered_set<int>> pot_cliques;
    vector<vector<unordered_set<int>>> temp_pot_cliques;



    vector <int> adjoinable_nodes = find_adjoinable_vertex(Q);
    if (adjoinable_nodes.size() == 0) {
        return { Q };
    }
    for (auto ad_node : adjoinable_nodes) {
        temp = Q;
        temp.insert(ad_node);
        temp_pot_cliques.push_back(expand_clique(temp));
    }
    int max = 0;

    //return all max cliques 
    for (auto &vec_max_clique : temp_pot_cliques) {
        for (auto &max_clique : vec_max_clique) {
            if (max <= max_clique.size()) {
                max = max_clique.size();
                pot_cliques.push_back(max_clique);
            }
        }

    }
    return pot_cliques;

}


bool PDBHeuristic::compare_cliques(unordered_set<int>& A, unordered_set <int>& B) {
    int count = 0;
    for (auto& node : A) {
        if (find(B.begin(), B.end(), node) != B.end()) {
            count++;
        }
    }
    return count == B.size(); //If all nodes from A are found in B, then they are equal. A cannot be a subset of B, because of the previous expansion
}

vector <vector<int>> PDBHeuristic::find_max_cliques() {
    //The algorithm is far away from potential worst-case performance Guarantees like O(3^{n/3}) as stated by
    // Etsuji Tomita, Akira Tanaka and Haruhisa Takahashi   in 
    // The Worst-Case Time Complexity for Generating All Maximal Cliques
    //Proceedings of the 10th Annual International Conference on Computing and Combinatorics(COCOON 2004), pp. 161 - 170, 2004.

    //However the algorithm is quite complex and the orthogonality graphs are quite small
    vector <unordered_set<int>> max_cliques;
    vector<vector<int>> clique_pattern;
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        unordered_set <int> temp;
        temp.insert(i);
        vector<unordered_set <int>> vec_temp_set = expand_clique(temp);
        for (auto& clique : vec_temp_set) {
            max_cliques.push_back(clique);
        }
    }

    //Delete duplicates
    for (size_t i = 0; i < max_cliques.size(); i++) {
        for (size_t j = i + 1; j < max_cliques.size(); j++) {
            if (!max_cliques[i].empty() && !max_cliques[j].empty()) {
                if (compare_cliques(max_cliques[i], max_cliques[j])) {
                    max_cliques[j].clear();
                }
            }

        }
    }

    for (auto& max_clique : max_cliques) {
        if (!max_clique.empty()) {
            vector <int> clique;
            for (auto& node : max_clique) {
                clique.push_back(node);
            }
            clique_pattern.push_back(clique);
        }
    }
    return clique_pattern;


}

bool PDBHeuristic::is_clique(unordered_set<int>& set) {
    for (auto& node1 : set) {
        for (auto& node2 : set) {
            if (!orthogonality_graph[node1][node2]) {
                if (node1 == node2) {
                    continue;
                }
                else {
                    return false;
                }
            }
        }
    }
    return true;
}




static Heuristic* _parse(OptionParser& parser)
{
    Heuristic::add_options_to_parser(parser);
    parser.add_list_option<int>("test_pattern", "", "[]", OptionFlags(false));
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return 0;
    }
    else {
        return new PDBHeuristic(opts);
    }
}

static Plugin<Heuristic> _plugin("pdb", _parse);
