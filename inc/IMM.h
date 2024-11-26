#pragma once
#include "common.h"
#include "PP.h"
#include "SpaceTimeAStar.h"
#include <numeric>

#ifndef IMM_H
#define IMM_H

class IMM {
public:
    // memo: list of tuples (partial ordering, welfare)
    vector<pair<vector<set<int> >, double> > memo;
    vector<Agent> agents;
    bool solution_found;

    IMM(Instance& instance, int screen, int seed);
    static void print_partial_order(vector<set<int>>& partial_order);
    bool is_in_memo(const vector<int>& total_order);
    void run(int n_runs, double time_out_sec);

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
};

#endif //IMM_H
