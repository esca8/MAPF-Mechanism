#pragma once
#include "common.h"
#include "PP.h"
#include "SpaceTimeAStar.h"
#include <numeric>
#include <queue>

#ifndef IMM_H
#define IMM_H

class IMM {
public:
    // memo: list of tuples (partial ordering, welfare, cost)
    vector<tuple<vector<set<int>>, double, double> > memo;
    unordered_map<string, pair<double, double>> memo2;

    vector<Agent> agents;
    bool solution_found;
    boost::filesystem::path logdir;

    IMM(Instance& instance, int screen, int seed,
        boost::filesystem::path logdir);
    static void print_partial_order(vector<set<int>>& partial_order);
    bool is_in_memo(string s);
    int removeStrongerMemoEntry(vector<set<int>> partial_order);
    void run(int n_runs, double time_out_sec);

    double solution_cost = -2;
    double max_social_welfare = INT_MIN;

    queue<pair<graph_t, string>> PartialOrdersQueue; // (dag, dag hash) queue
    set<string> PartialOrdersQueuedHashes;

private:
    // input params
    Instance& instance;
    int screen;
    vector<SpaceTimeAStar*> search_engines;
    vector<Path*> best_paths;

    mt19937 gen;
    int seed;

    // [i] stores the weighted sum of cost without agent i that corresponds to
    // the maximum welfare
    vector<double> min_sum_of_cost_wo_i;
    // [i] stores the max welfare without agent i
    vector<double> max_welfare_wo_i;

    double runtime_uncached_pp1 = 0;
    double runtime_cached_lookup = 0;
    double runtime_update_permutation = 0;

    void reverse_all_edge_combos_helper(graph_t graph_orig, graph_t graph_upd,
                                        int ind1, set<int>::iterator ind2);
    void reverse_all_edge_combos(graph_t graph);
};

#endif //IMM_H
