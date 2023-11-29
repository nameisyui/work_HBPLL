#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <Hop/HBPLL_two_hop_labels.h>

struct HBPLL_v1_node
{
public:
    int vertex, parent_vertex, hop;
    double priority_value;
};

bool operator<(HBPLL_v1_node const &x, HBPLL_v1_node const &y)
{
    return x.priority_value > y.priority_value;
}
typedef typename boost::heap::fibonacci_heap<HBPLL_v1_node>::handle_type HBPLL_v1_node_handle;

/**
 * Hop-Bounded Dijkstra, 在HBPLL生成索引过程中被并行调用
 * @param v_k                   遍历起点
 * @param N                     顶点总个数
 * @param upper_k               遍历边数上限, 即在一条路径上遍历超过了k条边则停止遍历, 一般情况下不会用到
 */
void HB_thread_function_HBDIJ_Qhandle(int v_k, int N, int upper_k)
{
    /* get unique thread id */
    mtx_599[max_N_599 - 1].lock();
    int used_id = Qid_599.front();
    Qid_599.pop();
    mtx_599[max_N_599 - 1].unlock();

    /* Temp_L_vk_599 stores the label (dist and hop) of vertex v_k */
    queue<int> Temp_L_vk_changes;
    mtx_599[v_k].lock();
    int L_vk_size = L_temp_599[v_k].size();
    for (int i = 0; i < L_vk_size; i++)
    {
        int L_vk_vertex = L_temp_599[v_k][i].vertex;
        Temp_L_vk_599[used_id][L_vk_vertex].push_back({L_temp_599[v_k][i].distance, L_temp_599[v_k][i].hop});
        Temp_L_vk_changes.push(L_vk_vertex);
    }
    mtx_599[v_k].unlock();

    /* dist_hop_599 stores the shortest distance from vk to any other vertices with its hop_cst */
    queue<int> dist_hop_changes;
    dist_hop_599[used_id][v_k] = {0, 0};
    dist_hop_changes.push(v_k);
    /* {vertex, hop} -> ptr */
    map<pair<int, int>, pair<HBPLL_v1_node_handle, double>> Q_handle;
    boost::heap::fibonacci_heap<HBPLL_v1_node> Q;
    HBPLL_v1_node node;
    node.vertex = v_k;
    node.parent_vertex = v_k;
    node.hop = 0;
    node.priority_value = 0;
    // Q.push({node})返回一个指针
    Q_handle[{v_k, 0}] = {Q.push({node}), node.priority_value};
    two_hop_label_v1 xx;
    long long int new_label_num = 0;
    while (Q.size() > 0)
    {
        //------------------Begin TODO------------------
        HBPLL_v1_node temp = Q.top();
        Q.pop();
        if (v_k <= temp.vertex)
        {
            double min_distance = std::numeric_limits<double>::max();
            for(auto it:L_temp_599[temp.vertex] ){
                double temp_distance=it.distance+Temp_L_vk_599[used_id][it.vertex][0].first;
                double temp_hop=it.hop+Temp_L_vk_599[used_id][it.vertex][0].second;
                if(temp_hop<=temp.hop&&min_distance>temp_distance){
                    min_distance=temp_distance;
                }
            }
            if (min_distance > temp.priority_value)
            {
                xx.distance = temp.priority_value;
                xx.hop = temp.hop;
                xx.parent_vertex = temp.parent_vertex;
                xx.vertex = v_k;
                L_temp_599[temp.vertex].push_back(xx);
                if (temp.vertex == v_k)
                {
                    Temp_L_vk_599[used_id][v_k].push_back({xx.distance, xx.hop});
                    Temp_L_vk_changes.push(v_k);
                }
                dist_hop_599[used_id][temp.vertex] = {xx.distance, xx.hop};
                dist_hop_changes.push(temp.vertex);
                int new_hop=temp.hop+1;
                if (new_hop <= upper_k)
                {
                    auto neighbors = ideal_graph_599[temp.vertex];

                    for (auto it : neighbors)
                    {
                        double dv = xx.distance + it.second;
                        double Q_vh = std::numeric_limits<double>::max();
                        if (Q_handle.find({it.first, temp.hop + 1}) != Q_handle.end())
                        {
                            double Q_vh = min(Q_vh, Q_handle[{it.first, temp.hop + 1}].second);
                            if (dv < Q_vh)
                            {
                                auto ptr = Q_handle[{it.first, temp.hop + 1}].first;
                                auto new_node = *ptr;
                                new_node.priority_value = dv;
                                Q_handle[{it.first, temp.hop + 1}].second=dv;
                                Q.update(ptr, new_node);
                            }
                        }
                        else
                        {
                            HBPLL_v1_node new_node = {it.first, temp.vertex, temp.hop + 1, dv};
                            Q_handle[{it.first, temp.hop + 1}] = {Q.push(new_node), dv};
                        }
                    }
                }
            }
            //-------------------END TODO-------------------
            // exit(1);
        }
    }
    while (Temp_L_vk_changes.size() > 0)
    {
        vector<pair<double, int>>().swap(Temp_L_vk_599[used_id][Temp_L_vk_changes.front()]);
        Temp_L_vk_changes.pop();
    }

    while (dist_hop_changes.size() > 0)
    {
        dist_hop_599[used_id][dist_hop_changes.front()] = {std::numeric_limits<double>::max(), 0};
        dist_hop_changes.pop();
    }

    mtx_599[v_k].lock();
    vector<two_hop_label_v1>(L_temp_599[v_k]).swap(L_temp_599[v_k]);
    mtx_599[v_k].unlock();

    mtx_599[max_N_599 - 1].lock();
    Qid_599.push(used_id);
    mtx_599[max_N_599 - 1].unlock();
}

/**
 * 最短距离查询函数
 * @param L         索引集
 * @param source    查询起点
 * @param terminal  查询终点
 * @param hop_cst   查询边数(跳数)限制
 * @return          最短距离
 */
double HB_extract_distance_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst)
{
    /*return std::numeric_limits<double>::max() is not connected*/
    if (hop_cst < 0)
    {
        return std::numeric_limits<double>::max();
    }
    if (source == terminal)
    {
        return 0;
    }
    else if (hop_cst == 0)
    {
        return std::numeric_limits<double>::max();
    }

    double distance = std::numeric_limits<double>::max();
    //------------------Begin TODO------------------
    for (auto it_s = L[source].begin(); it_s != L[source].end(); it_s++)
    {
        for (auto it_t = L[terminal].begin(); it_t != L[terminal].end(); it_t++)
        {
            if (it_s->vertex == it_t->vertex and it_s->hop + it_t->hop <= hop_cst)
            {
                distance = min(distance, it_s->distance + it_t->distance);
            }
        }
    }
    //-------------------END TODO-------------------
    return distance;
}

/**
 * 最短距离查询函数
 * @param L         索引集
 * @param source    查询起点
 * @param terminal  查询终点
 * @param hop_cst   查询边数(跳数)限制
 * @return          路径上边组成的集合, 如最短路径是 1->2->3 则返回 [{1,2},{2,3}]
 */
vector<pair<int, int>> HB_extract_path_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst)
{
    vector<pair<int, int>> paths;
    if (source == terminal)
    {
        return paths;
    }
    //------------------Begin TODO------------------
    // 递归结束
    if (hop_cst == 1)
    {
        for (auto it_s = L[source].begin(); it_s != L[source].end(); it_s++)
        {
            if (it_s->vertex == terminal)
            {
                paths.push_back({source, terminal});
            }
        }
        return paths;
    }
    int mid = 0, s_pre = 0, t_pre = 0;
    int s_hop_cst = 0, t_hop_cst = 0;
    double distance = std::numeric_limits<double>::max();
    // 递归部分
    for (auto it_s = L[source].begin(); it_s != L[source].end(); it_s++)
    {
        for (auto it_t = L[terminal].begin(); it_t != L[terminal].end(); it_t++)
        {
            if (it_s->vertex == it_t->vertex and it_s->hop + it_t->hop <= hop_cst and distance > it_s->distance + it_t->distance)
            {
                paths.clear();
                mid = it_s->vertex;
                vector<pair<int, int>> pair_s = HB_extract_path_v1(L, source, mid, it_s->hop);
                vector<pair<int, int>> pair_t = HB_extract_path_v1(L, mid, terminal, it_t->hop);
                paths.insert(paths.end(), pair_s.begin(), pair_s.end());
                paths.insert(paths.end(), pair_t.begin(), pair_t.end());
            }
        }
    }
    //-------------------END TODO-------------------
    return paths;
}
