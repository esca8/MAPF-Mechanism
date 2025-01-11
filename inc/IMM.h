#pragma once
#include "common.h"
#include "PP.h"
#include "SpaceTimeAStar.h"
#include <numeric>

#ifndef IMM_H
#define IMM_H

class IMM {
public:
    // memo: list of tuples (partial ordering, welfare, cost)
    vector<tuple<vector<set<int> >, double, double> > memo;
    vector<Agent> agents;
    bool solution_found;
    boost::filesystem::path logdir;

    IMM(Instance& instance, int screen, int seed,
        boost::filesystem::path logdir);
    static void print_partial_order(vector<set<int>>& partial_order);
    bool is_in_memo(const vector<int>& total_order);
    int removeStrongerMemoEntry(vector<set<int>> partial_order);
    void run(int n_runs, double time_out_sec);

    double solution_cost = -2;
    double max_social_welfare = INT_MIN;

    typedef vector<set<int>> PO;
    vector<PO> partial_orders;

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
};

#endif //IMM_H
