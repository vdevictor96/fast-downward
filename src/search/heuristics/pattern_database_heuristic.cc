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
int PDBHeuristic::unrank( int r, int var, int ind) {
    int temp = r / N_ind_collection[ind][var];
    assert(temp >= 0);
    return temp % g_variable_domain[var];
}
 int PDBHeuristic::rankState(const State& state, int ind) {
     int sum = 0;
    for (auto& i : pattern_collection[ind]) {
        sum += N_ind_collection[ind][i] * state[i];
    }
    assert(sum >= 0&&sum<=N_ind[ind]);
    return sum;
}

 int PDBHeuristic::rank(vector <int>& s, int ind) {
     int sum = 0;
    for (auto& pat_var : pattern_collection[ind]) {
        sum += N_ind_collection[ind][pat_var] * s[pat_var];
    }
    assert(sum >= 0 && sum <= N_ind[ind]);
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
vector <int> PDBHeuristic::check_applicable_ops(vector <int>& pat) {
    vector <int> pattern_ops;
    int count = 0;
    for (int op = 0; op < g_operators.size();op++) {
        for (auto& eff : g_operators[op].get_effects()) {
            if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
                pattern_ops.push_back(count);
                break;
            }
        }
        count++;
    }
    return pattern_ops;
}

bool PDBHeuristic::op_applicable(int op, vector<int>& s, vector <int>& pat) {
    for (auto& i : g_operators[op].get_preconditions()) {
        if (find(pat.begin(), pat.end(), i.var) != pat.end()) {
            if (s[i.var] != i.val) {
                return false;
            }
        }
    }
    return true;
}

vector<int> PDBHeuristic::apply_operation(int op, vector <int> s, vector<int>& pat) {
    for (auto& eff : g_operators[op].get_effects()) {
        if (find(pat.begin(), pat.end(), eff.var) != pat.end()) {
            s[eff.var] = eff.val;
        }
    }
    return s;
}

void PDBHeuristic::computePDB() {
    //The PDB computation follows the sudocode of the paper
    //"Efficient Implementation of Pattern Database Heuristics for Classical Planning" by Sievers, Ortlieb & Helmert, 2012
    //Without their made improvements


    //closed list for BFS
    closed_list_collection.resize(pattern_collection.size());
    //open list for BFS
    list_collection.resize(pattern_collection.size());
    //adjacency list for graph pf abstract stae space
    adjList_collection.resize(pattern_collection.size());
    //number of states in abstract problem for a given pattern
    N_ind.assign(pattern_collection.size(), 0);


    vector < int> n_arr(g_variable_domain.size());
    fill(n_arr.begin(), n_arr.end(), 0);
    int count = 0;
    //N_ind_collection is for unranking and ranking
    for (auto& pattern : pattern_collection) {
        int N = 1;
        N_ind_collection.push_back(n_arr);
        for (auto& var : pattern) {
            N_ind_collection[count][var] = N;
            N *= g_variable_domain[var]; 
        }
        N_ind[count] = N; //For a pattern, this is the number of states in the abstract state space
        count++;
    }


    //Init PDB with -1 for all available patterns
    for ( int i = 0; i < pattern_collection.size(); i++) {
        vector<int> abstract_space_vec(N_ind[i]+1);
        fill(abstract_space_vec.begin(), abstract_space_vec.end(), -1);
        PDB_collection.push_back(abstract_space_vec);
    }

    //Find applicable operations for each pattern
    for (auto& pattern : pattern_collection) {
        applicable_ops_collection.push_back(check_applicable_ops(pattern));
    }
    int ind = 0;
    for (auto& pattern : pattern_collection) {
        list_collection[ind].push(N_ind[ind]); //Add state with 0-action cost
        closed_list_collection[ind].insert(N_ind[ind]);

        vector <int> abstract_state(g_variable_domain.size());
        //For a pattern iterate over all states (represended as index)
        for (int state_rank = 0; state_rank < N_ind[ind]; ++state_rank) {
            

            //unrank abstract state
            for (auto& pat_var : pattern) {
                abstract_state[pat_var] = unrank(state_rank, pat_var, ind);
            }

            if (goal_test(abstract_state, pattern)) {
                adjList_collection[ind][N_ind[ind]].insert(state_rank);
         
                //PDB_collection[ind][state_rank] = 0;
                //closed_list_collection[ind].insert(state_rank);
                //list_collection[ind].push(state_rank);
            }
            //Consider only relevant operations. Other operators have no effect and add if applicabale only self-transitions
            for(auto &op:applicable_ops_collection[ind]) {
                if (op_applicable(op, abstract_state, pattern)) {
                    vector <int> next_state = apply_operation(op, abstract_state, pattern);           
                    int next_state_rank = rank(next_state, ind);
                    adjList_collection[ind][next_state_rank].insert(state_rank); //The graph is backwards
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

        vector<vector<int>> temp_col;
        for (int i = 0; i < 10; i++) {
            temp_col= disjoint_pattern_selection();
            for (auto& pat : temp_col) {
                pattern_collection.push_back(pat);
            }
        }

        /*
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        pattern_collection.push_back(naive_pattern_selection());
        */

        


        

        computePDB();
        if (pattern_collection.size() > 1) {
            create_orthogonality_graph();
            max_cliques_ind = find_max_cliques();
        }
        

    }

}


void PDBHeuristic::Dijkstra(int ind) { //This is actual breadth-first search with unit cost

    while (!list_collection[ind].empty()) {
        int front = list_collection[ind].front();
        
            for (auto& neighbour : adjList_collection[ind][front]) {
                if (closed_list_collection[ind].find(neighbour) == closed_list_collection[ind].end()) {
                    list_collection[ind].push(neighbour);
                    PDB_collection[ind][neighbour] = PDB_collection[ind][front] + 1;
                    closed_list_collection[ind].insert(neighbour);
                }
            }
        list_collection[ind].pop();
            
    }
}


int PDBHeuristic::compute_heuristic(const State& state)
{
    vector<int> pattern_rank;
    for (int i = 0; i < pattern_collection.size(); i++) {
        pattern_rank.push_back(rankState(state, i));
    }

    //standard PDB with one pattern
    if (pattern_collection.size() == 1) {
        return PDB_collection[0][pattern_rank[0]];
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
vector<vector<int>>PDBHeuristic::disjoint_pattern_selection() {
    unordered_set<int> var_set;
    vector<unordered_set<int>> pat_sel;
    for (int var = 0; var < g_variable_domain.size(); var++) {
        var_set.insert(var);
    }
    for (auto& goal : g_goal) {
        pat_sel.push_back({ goal.first });
   
    }
    int count = 0;
    while (!var_set.empty()&&count<10) {
        int var = rand() % g_variable_domain.size();
        if (var_set.find(var) != var_set.end()) {
            int pat_ind = rand() % pat_sel.size();
            var_set.erase(var);
            pat_sel[pat_ind].insert(var);
            int temp = count;
            count = 0;
            int n_dom = 1;
            for (auto var_dom : pat_sel[pat_ind]) {
                n_dom *= g_variable_domain[var_dom];
            }
            if (n_dom > 5000) {// This controls the search space for a pattern
                pat_sel[pat_ind].erase(var);
                count = temp; //Terminate after a number of attempts
                count++;
                if (g_variable_domain[var] < 500) { //Make sure the domain is not huge
                    var_set.insert(var);
                }
            }

        }
    }
    vector<vector<int>> pat_vec;
    for (auto& set : pat_sel) {
        vector <int> pat;
        for (auto& var : set) {
            pat.push_back(var);
        }
        pat_vec.push_back(pat);
    }
    return pat_vec;
}



vector <int> PDBHeuristic::naive_pattern_selection() {
    vector <int> pat;
    //add random goal variable. There a few hyperparameter here, which should keep the state space under controll. However this does really
    //regulate the space for huge problems
    for (int i = 0; i < g_goal.size(); i += 8) {
        int g = rand() % g_goal.size();

        //Small hack so that the state space is under controll. Has obvious drawbacks
        if (g_variable_domain[g_goal[g].first] < 20) {
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
    }
    //add some random variables
    for (int i = 0; i < g_variable_domain.size(); i += 8) {
        int g = rand() % g_variable_domain.size();
        if (g_variable_domain[g] < 20) {
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


        sort(pat.begin(), pat.end());
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
            //This is checked by comparing the operator IDs
            for (auto& op1 : applicable_ops_collection[i]) {
                for (auto& op2 : applicable_ops_collection[j]) {
                    if (op1 == op2) {
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
    //Set diagonal false, this is a design choice for later clique computations
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        orthogonality_graph[i][i] = false;
    }

}


vector <int> PDBHeuristic::find_adjoinable_vertex(unordered_set<int> Q) {
    //For a clique Q return all nodes that would expand Q, while remaining a clique
   
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
    //For Q=1, we want ALL cliques. Imagine for node 1 two max clique {1,2,3} and {1,2,4}. The followling makes sure to return both of them
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

    //However the algorithm is quite complex and the orthogonality graphs are quite small, so it should be fine
    vector <unordered_set<int>> max_cliques;
    vector<vector<int>> clique_pattern;

    //Expand all max_cliques for each node. This implies a lot of repeated computation
    // e.g. a max_clique is {2,4}, than the following method will also yield {4,2}, which then later has to be deleted.
    for (int i = 0; i < orthogonality_graph.size(); i++) {
        unordered_set <int> temp;
        temp.insert(i);
        vector<unordered_set <int>> vec_temp_set = expand_clique(temp);
        for (auto& clique : vec_temp_set) {
            max_cliques.push_back(clique);
        }
    }

    //Delete duplicates.
    for ( int i = 0; i < max_cliques.size(); i++) {
        for ( int j = i + 1; j < max_cliques.size(); j++) {
            if (!max_cliques[i].empty() && !max_cliques[j].empty()) {
                if (compare_cliques(max_cliques[i], max_cliques[j])) {
                    max_cliques[j].clear();
                }
            }

        }
    }

    //Convert sets to vectors
    //I should have used vectors in the first place instead of sets
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
    //Check for all nodes if they have an edge
    for (auto& node1 : set) {
        for (auto& node2 : set) {
            if (!orthogonality_graph[node1][node2]) {
                //identical node is always connected
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
