#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path)
{
    for (const auto& state : path)
    {
        os << state.location;  // << "(" << state.is_single() << "),";
    }
    return os;
}

bool isSamePath(const Path& p1, const Path& p2)
{
    if (p1.size() != p2.size())
        return false;
    for (unsigned i = 0; i < p1.size(); i++)
    {
        if (p1[i].location != p2[i].location)
            return false;
    }
    return true;
}

string doubleToStr(double d, int precision)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << d;
    std::string s = stream.str();
    return s;
}

vector<int> softmax_ordering(vector<int>& agents_to_arrange,
                             vector<double>& predictions_to_arrange)
{
    assert(agents_to_arrange.size() == predictions_to_arrange.size());  // debug
    int num_agents = agents_to_arrange.size();
    vector<int> ordering;
    double curr_total_sum = std::accumulate(predictions_to_arrange.begin(),
                                            predictions_to_arrange.end(), 0.0);

    while (!agents_to_arrange.empty())
    {
        double running_sum = 0;
        double curr_total_sum = std::accumulate(
            predictions_to_arrange.begin(), predictions_to_arrange.end(), 0.0);
        double cutoff_value = fRand(0, curr_total_sum);
        for (int k = 0; k < predictions_to_arrange.size(); k++)
        {
            running_sum += predictions_to_arrange[k];
            if (running_sum >= cutoff_value)
            {
                ordering.push_back(agents_to_arrange[k]);
                agents_to_arrange.erase(agents_to_arrange.begin() + k);
                predictions_to_arrange.erase(predictions_to_arrange.begin() +
                                             k);
                running_sum = 0;
                break;
            }
        }
    }
    return ordering;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

string get_curr_time_str()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();

    // Convert the time to a time_t object
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // Convert the time to a string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ss.str();
    return timestamp;
}

string get_uuid()
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    return boost::uuids::to_string(uuid);
}

void write_to_json(json to_write, boost::filesystem::path filename)
{
    // Open a file for writing
    std::ofstream outfile(filename.string());

    // Write the JSON object to the file
    outfile << std::setw(4) << std::setfill(' ') << to_write << std::endl;

    // Close the file
    outfile.close();
}

void write_config_to_file(boost::program_options::variables_map vm,
                          boost::filesystem::path filename)
{
    json config;
    for (const auto& it : vm)
    {
        // std::cout << it.first.c_str() << " ";
        auto& value = it.second.value();
        if (auto v = boost::any_cast<bool>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<int>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<double>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<std::string>(&value))
            config[it.first] = *v;
        else
            std::cout << "Unknown var type for config param" << it.first
                      << std::endl;
    }
    write_to_json(config, filename);
}

double weighted_path_cost(vector<Path*> paths, vector<double> costs)
{
    assert(paths.size() == costs.size());

    double weighted_sum = 0;
    for (int i = 0; i < paths.size(); i++)
    {
        weighted_sum += (double)(paths[i]->size() - 1) * costs[i];
    }
    return weighted_sum;
}

bool areDoubleSame(double dFirstVal, double dSecondVal)
{
    return std::abs(dFirstVal - dSecondVal) < 1E-5;
}

int find_index(vector<int> v, int to_find)
{
    for(int i = 0; i < v.size(); i++)
    {
        if (v[i] == to_find)
            return i;
    }
    return -1;
}

vector<int> topoSort(graph_t dag)
{
    int n = dag.size();
    vector<int> topo_ord(0);
    vector<int> in_degrees(n, 0);
    queue<int> q;
    for(int i = 0; i < n; i++)
    {
        for(int j : dag[i])
        {
            in_degrees[j]++;
        }
    }
    for(int i = 0; i < n; i++)
    {
        if(in_degrees[i] == 0) q.push(i);
    }
    while(!q.empty() && topo_ord.size() < n)
    {
        int i = q.pop();
        topo_ord.push_back(i);
        for(int j : dag[i])
        {
            in_degrees[j]--;
            if(in_degrees[j] == 0) q.push(j);
        }
    }
    if(topo_ord.size() != n)
    {
        cerr << "error: graph contains cycles" << endl;
    }
}

// check for cycles involving node i (for a connected component)
bool containsCycleHelper(graph_t graph, vector<bool> ancestors, vector<bool> visited, int curr) {
    visited[curr] = true;
    ancestors[curr] = true;
    for(int j : graph[curr])
    {
        if(ancestors[j]) return false; // has a cycle
        if(!visited[j] && containsCycleHelper(graph, ancestors, visited, j)) return true;
    }
    return true;
}

bool containsCycle(graph_t graph)
{
    int n = graph.size();
    vector<bool> visited(n, 0);
    vector<bool> ancestors(n, 0);
    for(int i = 0; i < n; i++)
    {
        if(!visited[i] && containsCycleHelper(graph, ancestors, visited, i)) return true;
    }
    return false;
}

// convert set to string (iter traverses the set in order)
string setToString(set<int> s)
{
    stringstream ss;
    for (auto it = s.begin(); it != s.end(); it++)
    {
        if(it != s.begin()) ss << ",";
        ss << *it;
    }
    return ss.str();
}

string hashDAG(graph_t graph)
{
    stringstream ss;
    vector<int> topo_ord = topoSort(graph);
    for(int node : topo_ord)
    {
        ss << node;
        ss << ":" << setToString(graph[node]) << "|";
    }
    return ss.str();
}

void reverseEdge(graph_set_t graph, int src, int dest)
{
    assert(graph[src].find(dest) != graph[src].end());
    graph[src].erase(dest);
    graph[dest].insert(src);
}