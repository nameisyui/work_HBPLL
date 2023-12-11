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
            double min_distance = numeric_limits<double>::max();
            mtx_599[temp.vertex].lock();
            for (auto it : L_temp_599[temp.vertex])
            {
                for (auto it_t : Temp_L_vk_599[used_id][it.vertex])
                {
                    if (it_t.second + it.hop <= temp.hop)
                    {
                        min_distance = min(min_distance, it_t.first + it.distance);
                    }
                }
            }
            mtx_599[temp.vertex].unlock();

            if (min_distance > temp.priority_value)
            {
                xx.distance = temp.priority_value;
                xx.hop = temp.hop;
                xx.vertex = v_k;
                xx.parent_vertex = temp.parent_vertex;

                mtx_599[temp.vertex].lock();
                L_temp_599[temp.vertex].push_back(xx);
                mtx_599[temp.vertex].unlock();

                int new_hop = temp.hop + 1;
                if (new_hop <= upper_k)
                {
                    auto neighbors = ideal_graph_599[temp.vertex];
                    for (auto it : neighbors)
                    {
                        double dv = xx.distance + it.second;
                        bool check1=dv < dist_hop_599[used_id][it.first].first;//更近的距离
                        bool check2=new_hop < dist_hop_599[used_id][it.first].second;//更远但有更小的hop
                        if (check1 or check2)
                        {
                            if (Q_handle.find({it.first, new_hop}) != Q_handle.end())//有记录过
                            {
                                if (dv < Q_handle[{it.first, new_hop}].second)
                                {
                                    auto ptr = Q_handle[{it.first, new_hop}].first;
                                    HBPLL_v1_node new_node = {it.first, temp.vertex,new_hop, dv};
                                    new_node.priority_value = dv;
                                    Q_handle[{it.first, new_hop}].second = dv;
                                    Q.update(ptr, new_node);
                                }
                            }
                            else
                            {
                                HBPLL_v1_node new_node = {it.first, temp.vertex, new_hop, dv};
                                Q_handle[{it.first, new_node.hop}] = {Q.push(new_node), dv};
                            }
                            // 其中
                            if (check1)//更新最短距离
                            {
                                dist_hop_599[used_id][it.first] = pair<double, int>(dv, new_hop);
                                dist_hop_changes.push(it.first);
                            }
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
    auto it_s = L[source].begin(), it_t = L[terminal].begin();
    auto it_s_end = L[source].end(), it_t_end = L[terminal].end();
    while (it_s != it_s_end && it_t != it_t_end)
    {
        if (it_s->vertex == it_t->vertex)
        {
            auto vertex = it_s->vertex;
            auto temp_t = it_t; // 应该回溯到的位置
            while (it_s != it_s_end && it_s->vertex == vertex)
            {
                it_t = temp_t; // 回溯
                while (it_t != it_t_end && it_t->vertex == vertex)
                {
                    if (it_s->hop + it_t->hop <= hop_cst)
                    {
                        distance = min(distance, it_s->distance + it_t->distance);
                    }
                    it_t++;
                }
                it_s++;
            }
        }
        else if (it_s->vertex < it_t->vertex)
        {
            it_s++;
        }
        else
        {
            it_t++;
        }
    }
    //-------------------END TODO-------------------
    return distance;
}

/**
 * 最短路径查询函数
 * @param L         索引集
 * @param source    查询起点
 * @param terminal  查询终点
 * @param hop_cst   查询边数(跳数)限制
 * @return          路径上边组成的集合, 如最短路径是 1->2->3 则返回 [{1,2},{2,3}]
 */
vector<pair<int, int>> HB_extract_path_v1(vector<vector<two_hop_label_v1>> &L, int source, int terminal, int hop_cst)
{
    // cout<<source<<" "<<terminal<<" "<<hop_cst<<endl;
    vector<pair<int, int>> paths;
    if (source == terminal)
    {
        return paths;
    }
    //------------------Begin TODO------------------
    double distance = numeric_limits<double>::max();
    int pre_s, pre_t; // 两个label的前驱结点
    auto it_s = L[source].begin();
    auto it_t = L[terminal].begin();
    auto it_s_end = L[source].end(), it_t_end = L[terminal].end();
    while (it_s != it_s_end && it_t != it_t_end)
    {
        // 一旦发现相等，s先不动，t先向后遍历，直到it_t->vertex改变，
        // 此时让t回溯到刚发现相等时的位置，让s向后一位，然后继续向后遍历t，以此类推,
        // 直到遍历完所有的it_s->vertex==it_t->vertex
        if (it_s->vertex == it_t->vertex)
        {
            auto vertex = it_s->vertex;
            auto temp_t = it_t; // t应该回溯到的位置
            while (it_s != it_s_end && it_s->vertex == vertex)
            {
                it_t = temp_t; // 回溯t
                while (it_t != it_t_end && it_t->vertex == vertex)
                {
                    if (it_s->hop + it_t->hop <= hop_cst)
                    {
                        double dis = it_s->distance + it_t->distance;
                        if (distance > dis)
                        {
                            distance = dis;
                            pre_s = it_s->parent_vertex;
                            pre_t = it_t->parent_vertex;
                        }
                    }
                    it_t++;
                }
                // t遍历完成，轮到s++
                it_s++;
            }
        }
        else if (it_s->vertex < it_t->vertex)
        {
            it_s++;
        }
        else
        {
            it_t++;
        }
    }
    pair<int, int> end_path;
    bool have_end = false;
    if (distance < numeric_limits<double>::max() - 1)
    {
        if (source != pre_s)
        {
            paths.push_back({source, pre_s});
            source = pre_s;
            hop_cst--;
        }
        if (terminal != pre_t)
        {
            have_end = true;
            end_path = {pre_t, terminal};
            // paths.push_back({terminal, pre_t});
            terminal = pre_t;
            hop_cst--;
        }
    }
    else
    { // 两点之间无路径
        return paths;
    }
    // 递归寻找新路径
    vector<pair<int, int>> new_edges = HB_extract_path_v1(L, source, terminal, hop_cst);

    if (new_edges.size() > 0)
    {
        paths.insert(paths.end(), new_edges.begin(), new_edges.end());
    }
    if (have_end)
    {
        paths.push_back(end_path);
    }
    //-------------------END TODO-------------------
    return paths;
}
