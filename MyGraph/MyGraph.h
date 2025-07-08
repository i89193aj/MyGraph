#pragma once

#define Build_DLL

#ifdef Build_DLL 
#define DLL_API _declspec(dllexport)
#else 
#define DLL_API _declspec(dllimport)
#endif // BuildDLL _declspec(DLLExport)

#include <string>
#include <iostream>
#include <vector>       // 若成員變數用到 std::vector
#include <map>          // 若有 std::map 變數
#include <unordered_map>// 若有 std::unordered_map 變數
#include <queue>
#include <iomanip>

class MyGraph {
private:
    
public:
    // 預設建構式
    MyGraph() = default;
    
    // ======= Leetcode Solutions =======
    std::vector<int> Leetcode_Sol_xxx(std::vector<int>& numbers, int target, int _solution);
    std::vector<int> Exam_xxx(std::vector<int>& numbers, int target);
    // ======= Leetcode Solutions =======

};

class Dijkstra {
    typedef std::pair<int, int> T;
    std::vector<std::unordered_map<int,int>> graph;
    std::vector<int> dist;
public:
    Dijkstra() = default;
    Dijkstra(std::vector<std::unordered_map<int, int>>& _graph) :graph(_graph), dist(_graph.size()) {}
    Dijkstra(std::vector<std::vector<T>>& _graph) :graph(_graph.size()),dist(_graph.size(),INT_MAX) {
        for (int i = 0; i < _graph.size(); i++) {
            for (T& node : _graph[i])
                graph[i][node.first] = node.second;
        }
    }

    void push(int _from, int _to, int _weight) {
        int new_size = std::max(_from, _to) + 1;
        if (new_size > graph.size()) {
            graph.resize(new_size);
            dist.resize(new_size,INT_MAX);
        }

        graph[_from][_to] = _weight;          
    }

    void erase(int _from, int _to) {
        if (_from + 1 > graph.size() || !graph[_from].count(_to))
            return;
        graph[_from].erase(_to);
    }

    const std::vector<int>& getlist(int _start) {
        if (graph.empty()) return{};
        
        dist.assign(graph.size(),INT_MAX);
        dist[_start] = 0; 
        std::priority_queue<T, std::vector<T>, std::greater<T>> minheap;//注意input：他用距離比較!
        minheap.emplace(dist[_start],_start);

        while (!minheap.empty()) {
            T cur = minheap.top(); minheap.pop();
            int cur_node = cur.second;
            int cur_dist = cur.first;
            //這行非必要因為下面那行，也會幫你過濾，只是這行可以把heap裡面多餘的node直接省略，不會再跑下去for!
            if (cur_dist > dist[cur_node]) 
                continue;

            for (std::unordered_map<int, int>::iterator it = graph[cur_node].begin();it != graph[cur_node].end();it++) {
                int next_node = it->first;
                int next_dist = it->second;
                if (dist[next_node] > dist[cur_node] + next_dist) {
                    dist[next_node] = dist[cur_node] + next_dist;
                    minheap.emplace(dist[next_node],next_node);
                }
            }
        }
        return dist;
    }

    void clean() {
        graph.clear();
        std::vector<std::unordered_map<int, int>>().swap(graph);
        //graph.shrink_to_fit();
        dist.clear();
        std::vector<int>().swap(dist);
    }

    // 輔助函數：列印最短路徑
    void print_dist(int from) {
        std::cout << "從節點 " << from << " 出發的最短距離：" << std::endl;
        for (int i = 0; i < dist.size(); ++i) {
            if (dist[i] == INT_MAX) {
                std::cout << "節點 " << i << ": 無法到達" << std::endl;
            }
            else {
                std::cout << "節點 " << i << ": " << dist[i] << std::endl;
            }
        }
        std::cout << "----------------------" << std::endl;
    }
    
    ~Dijkstra() = default;
};

struct Edge {
    int _from;
    int _to;
    int _weight;
    Edge() = default;
    Edge(int from, int to, int weight)
        : _from(from), _to(to), _weight(weight) {
    }
};



class BellmanFord {
    using  T = std::pair<int, int>;
    std::vector<Edge> edge;
    std::vector<int> dist;
public:
    BellmanFord() = default;
    BellmanFord(std::vector<std::vector<T>> _graph) :dist(_graph.size(),INT_MAX) {
        for (int i = 0; i < _graph.size(); i++) {
            for (T& neighbor : _graph[i]) {
                Edge temp;
                temp._from = i;
                temp._to = neighbor.first;
                temp._weight = neighbor.second;
                edge.emplace_back(temp);
            }
        }
    }

    void push(int _from, int _to, int _weight) {
        int new_size = std::max(_from,_to) + 1;
        if (new_size > dist.size())
            dist.resize(new_size,INT_MAX);
        edge.emplace_back(_from,_to,_weight);
    }

    void erase (int _from, int _to){
        for (std::vector<Edge>::iterator it = edge.begin(); it != edge.end(); it++) {
            if (it->_from == _from && it->_to == _to) {
                edge.erase(it);
                break;
            }          
        }
        edge.shrink_to_fit();
    }

    const std::vector<Edge>& get_edges() const {
        return edge;
    }


    const std::vector<int> getlist(int _start) {
        if (edge.empty()) return {};
        dist.assign(dist.size(),INT_MAX);
        dist[_start] = 0;
        
        //dist.size() - 1：n個節點、跑n - 1個邊
        for (int i = 0; i < dist.size() - 1; i++) {
            for (Edge& cur : edge) {
                if (dist[cur._from] != INT_MAX && dist[cur._to] > dist[cur._from] + cur._weight)
                    dist[cur._to] = dist[cur._from] + cur._weight;
            }
        }
        return dist;
    }

    bool IsNegative_cycle(int _start) {
        if (edge.empty()) return false;
        dist.assign(dist.size(), INT_MAX);
        dist[_start] = 0;

        for (int i = 0; i < dist.size() - 1; i++) {
            for (Edge& cur : edge) {
                if (dist[cur._from] != INT_MAX && dist[cur._to] > dist[cur._from] + cur._weight)
                    dist[cur._to] = dist[cur._from] + cur._weight;
            }
        }

        for (Edge& cur : edge) {
            if (dist[cur._from] != INT_MAX && dist[cur._to] > dist[cur._from] + cur._weight)
                return true;
        }
        return false;
    }

    void print_result() {
        for (int i = 0; i < dist.size(); ++i) {
            std::cout << "  dist[" << i << "] = ";
            if (dist[i] == INT_MAX) std::cout << "INF\n";
            else std::cout << dist[i] << '\n';
        }
        std::cout << "----------------------" << std::endl;
    }

    void clean() {
        edge.clear();
        std::vector<Edge>().swap(edge);
        dist.clear();
        std::vector<int>().swap(dist);
    }

    ~BellmanFord() = default;

};

class FolydWarshall {
    typedef std::pair<int, int> T;//[neignbor,weight]
    int** dist = nullptr;
    int size;
public:
    FolydWarshall() = default;
    FolydWarshall(std::vector<std::vector<T>>& _graph) {
        build(_graph);
        run();
    }

    void build(std::vector<std::vector<T>>& _graph) {
        //if (_graph.empty()) return;
        size = _graph.size();

        dist = new int* [size]();    // 全部初始化為 0
        for (int i = 0; i < size; i++) {
            dist[i] = new int[size];
            for (int j = 0; j < size; j++) {
                dist[i][j] = (i == j) ? 0 : INT_MAX;
            }
        }

        for (int i = 0; i < size; i++) {
            for (T& cur : _graph[i]) {
                dist[i][cur.first] = cur.second;
            }
        }
    }

    void push(std::vector<std::vector<T>>& _graph) {
        cleen();
        build(_graph);
        run();
    }

    int** getlist() const{
        return dist;
    }

    void run() {
        for (int k = 0; k < size; ++k) {
            for (int i = 0; i < size; ++i) {
                for (int j = 0; j < size; ++j) {
                    if (dist[i][k] < INT_MAX && dist[k][j] < INT_MAX)
                        dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                }
            }
        }
    }

    void cleen() {
        if (!dist) return;
        for (int i = 0; i < size; i++) {
            delete [] dist[i];
            dist[i] = nullptr;
        }
        delete[] dist;
        dist = nullptr;
        size = 0;
    }

    void printMatrix() {
        std::cout << "最短距離矩陣:\n";
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                if (dist[i][j] == INT_MAX)
                    std::cout << std::setw(4) << "INF";
                else
                    std::cout << std::setw(4) << dist[i][j];
            }
            std::cout << '\n';
        }
        std::cout << "----------------------" << std::endl;
    }

    ~FolydWarshall() {
        cleen();
    }
};

class TopologicalSort {
    int* indegree = nullptr;
    int* order = nullptr;
    int size = 0;
public:
    TopologicalSort() = default;
    TopologicalSort(const std::vector<std::vector<int>>& _adj) {
        if(indegree || order) 
            clean();
        build(_adj);
        run(_adj);
    }

    void build(const std::vector<std::vector<int>>& _adj) {
        size = _adj.size();
        indegree = new int[size]();

        for (int i = 0; i < size; i++) {
            for (int node : _adj[i]) {
                indegree[node]++;
            }
        }
    }

    void push(const std::vector<std::vector<int>>& _adj) {
        if (indegree || order)
            clean();
        build(_adj);
        run(_adj);
    }

    void run(const std::vector<std::vector<int>>& _adj){
        if (!indegree) return;
        std::queue<int> q;
        int* p = indegree;
        for (int i = 0; i < size; ++i, ++p) {
            if (*p == 0)
                q.emplace(i);
        }

        delete[] order;  // 確保之前分配的被刪除
        order = new int[size]();
        p = order;
        int count = 0;
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            *(p++) = cur;
            count++;

            for (int next : _adj[cur]) {
                if (!--indegree[next]) {
                    q.emplace(next);
                }
            }
        }

        if (count < size) {
            std::cerr << "Warning: Graph contains a cycle; topological sort incomplete.\n";
        }
        //p = nullptr;區域變數沒有用
    }

    int* getlist() const {
        return order;
    }

    void clean() {
        delete [] order;
        order = nullptr;
        delete[] indegree;
        indegree = nullptr;
        size = 0;
    }

    void print() {
        if (!order) return;
        for (int i = 0; i < size; ++i) {
            std::cout << order[i] << ' ';
        }
        std::cout << "\n----------------------" << std::endl;
    }

    ~TopologicalSort() {
        clean();
    }
};



extern DLL_API MyGraph ClassTemplateInstance;
