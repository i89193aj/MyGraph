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
#include <stack>

struct Edges {
    int from, to, weight;
    bool operator<(const Edges& other) const {
        return weight < other.weight;
    }
};

class MyGraph {
private:
    typedef std::pair<int, int> T;
    //edge:[from, to, weight]
    static bool Myless (const std::vector<int>& a, const std::vector<int>& b) {
        return a[2] < b[2];
    }
    /*struct CompareEdge {
        bool operator()(const vector<int>& a, const vector<int>& b) const {
            return a[2] < b[2]; // 按照 edge 的 weight 排序
        }
    };*/
public:
    // 預設建構式
    MyGraph() = default;
    #pragma region Minimun Spanning Tree (最小生成樹)
    //Kruskal(MST)：回傳最小權重
    int Kruskal_Weight(int n, std::vector<std::vector<int>> _edges);
    //Kruskal(MST)：回傳最小生成樹的連接狀態
    std::vector<std::vector<int>> Kruskal_State(int n, std::vector<std::vector<int>> _edges);
    //Prim(MST)：回傳最小權重
    int Prim_Weight(int n, std::vector<std::vector<T>>& adj);
    //Prim(MST)：回傳最小生成樹的連接狀態
    std::vector<std::vector<int>> Prim_State(int n, std::vector<std::vector<T>>& adj);
    #pragma endregion

    #pragma region Connected Component
    std::vector<T> findBridges(int n, std::vector<std::vector<int>> adj_graph);
    std::vector<std::vector<int>> findSCCs_Tarjan(int n, std::vector<std::vector<int>> adj_graph);
    std::vector<std::vector<int>> findSCCs_Kosaraju(int n, std::vector<std::vector<int>> adj_graph);
    std::vector<std::vector<int>> findWWCs(int n, std::vector<std::vector<int>> adj_graph);
    #pragma endregion

    void printGraph(const std::vector<std::vector<int>>& adj_graph);
    void printGraphPair(std::vector<T>& adj);
    void printGraphCC(std::vector<std::vector<int>>& adj_graph);

    

    
    
    // ======= Leetcode Solutions =======
    std::vector<int> Leetcode_Sol_xxx(std::vector<int>& numbers, int target, int _solution);
    std::vector<int> Exam_xxx(std::vector<int>& numbers, int target);
    // ======= Leetcode Solutions =======

};

class TarjanBridge {
    typedef std::pair<int, int> T;
    friend class MyGraph;

    int timer = 0, node_count = 0;
    std::vector<int> low, visited_time;
    std::vector<std::vector<int>> adjacencylist;
    std::vector<T> bridge;
    
    void dfs(int u,int parent) {
        low[u] = visited_time[u] = timer++;

        for (int v : adjacencylist[u]) {
            if (v == parent) continue; //防止用父邊當成回邊
            if (visited_time[v] == -1) {
                dfs(v,u);
                //看看剛剛dfs跑的下一層有沒有"回邊"到自己的祖先 or 最祖先，如果有這裡也要連通，回傳給父節點使用!
                low[u] = std::min(low[u],low[v]);   
                if (low[v] > visited_time[u]) {
                    bridge.emplace_back(u,v);
                }
            }
            else {
                low[u] = std::min(low[u],low[v]);
            }
        }
    }

    void InitialParam(int n, std::vector<std::vector<int>>& adj_graph) {
        node_count = n;
        adjacencylist = adj_graph;
        low.assign(n, -1);
        visited_time.assign(n, -1);
        bridge.clear();
        timer = 0;
    }

    std::vector<T> findBridges() {
        for (int i = 0; i < node_count; i++) {
            if (visited_time[i] == -1)
                dfs(i,-1);
        }
        return bridge;
    }

public:
    TarjanBridge() = default;
    TarjanBridge(int n, std::vector<std::vector<int>>& adj_graph) {
        InitialParam(n, adj_graph);
    }
};

class TarjanSCC {
    friend class MyGraph;

    int timer = 0, node_count = 0;
    std::vector<int> low, visited_times, instack;
    std::vector<std::vector<int>> adjacencylist;
    std::stack<int> stk;
    std::vector<std::vector<int>> SCCs;

    void dfs(int u) {
        low[u] = visited_times[u] = timer++;
        stk.emplace(u);
        instack[u] = true;

        for (int v : adjacencylist[u]) {
            if (visited_times[v] == -1) {
                dfs(v);
                low[u] = std::min(low[u], low[v]);
            }
            else if (instack[v]) { //重要：如果父節點不在stack內，那就代表不夠成環，所以不會更新low[u]
                low[u] = std::min(low[u],low[v]);
            }
        }
        //這邊把強連通的節點從stack拿出來並放在一起(白話：就是參與有環的節點放在一起)
        if (low[u] == visited_times[u]) {
            
            std::vector<int> SCC;
            int v;
            do {
                v = stk.top(); stk.pop();
                instack[v] = false;
                SCC.emplace_back(v);
            } while (u != v);
            SCCs.emplace_back(SCC);
        }
    }

    void InitialParam(int n, std::vector<std::vector<int>>& adj_graph) {
        node_count = n, timer = 0;
        adjacencylist = adj_graph;
        low.assign(node_count,-1);
        visited_times.assign(node_count,-1);
        instack.assign(node_count,false);
        SCCs.clear();
        while (!stk.empty())
            stk.pop();
    }

    std::vector<std::vector<int>> findSCCs() {
        for (int i = 0; i < node_count; i++)
            if (visited_times[i] == -1)
                dfs(i);

        return SCCs;
    }
public:
    TarjanSCC() = default;
    TarjanSCC(int n, std::vector<std::vector<int>>& adj_graph) {
        InitialParam(n, adj_graph);
    }
};

class Kosaraju {
    friend class MyGraph;

    int node_count = 0;
    std::vector<std::vector<int>> adjacencylist, revadjacencylist;
    std::vector<bool> visited;
    std::stack<int> order;  //DFS離開點的順序 (最後的節點，會先離開!)
    std::vector<std::vector<int>> SCCs;

    void dfs1(int u) {
        visited[u] = true;
        for (int v : adjacencylist[u]) {
            if (visited[v]) continue;
            dfs1(v);
        }

        order.emplace(u);
    }

    void dfs2(int u, std::vector<int>& _SCC) {
        visited[u] = true;
        _SCC.emplace_back(u);
        for (int v : revadjacencylist[u]) {
            if (visited[v]) continue;
            dfs2(v, _SCC);
        }
    }

    void InitialParam(int n, std::vector<std::vector<int>>& adj_graph) {
        node_count = n;
        adjacencylist = adj_graph;
        revadjacencylist.assign(node_count, {});
        for (int u = 0; u < node_count; u++) {
            for (int v : adjacencylist[u]) {
                revadjacencylist[v].emplace_back(u);
            }
        }

        visited.assign(node_count,false);
        while (!order.empty()) order.pop();
        SCCs.clear();
    }

    std::vector<std::vector<int>> findSCCs() {
        for (int u = 0; u < node_count; u++) {
            if (visited[u]) continue;
            dfs1(u);
        }

        std::fill(visited.begin(),visited.end(),false);//將visited填滿某值

        while (!order.empty()) {
            int u = order.top(); order.pop();
            if (visited[u]) continue;
            std::vector<int> SCC;
            dfs2(u, SCC);
            SCCs.emplace_back(SCC);
        }

        return SCCs;
    }
public:
    Kosaraju() = default;
    Kosaraju(int n, std::vector<std::vector<int>>& adj_graph) {
        InitialParam(n, adj_graph);
    }
};

class WCC {
    friend class MyGraph;
    int node_count = 0;
    std::vector<std::vector<int>> undirected_graph;
    std::vector<bool> visited;
    std::vector<std::vector<int>> WCCs;

    void dfs(int u, std::vector<int>& _WCC) {
        visited[u] = true;
        _WCC.emplace_back(u);
        for (int v : undirected_graph[u]) {
            if (visited[v]) continue;
            dfs(v, _WCC);
        }
    }

    void InitialParam(int n, std::vector<std::vector<int>>& adj_graph) {
        node_count = n;
        undirected_graph.assign(node_count, {});
        for (int u = 0; u < node_count; u++) {
            for (int v : adj_graph[u]) {
                undirected_graph[u].emplace_back(v);
                undirected_graph[v].emplace_back(u);
            }
        }
        visited.assign(node_count,false);
        WCCs.clear();
    }

    std::vector<std::vector<int>> findWCCs() {
        for (int u = 0; u < node_count; u++) {
            std::vector<int> WCC;
            if (visited[u]) continue;
            dfs(u, WCC);
            WCCs.emplace_back(WCC);
        }
        return WCCs;
    }

public:
    WCC() = default;
    WCC(int n, std::vector<std::vector<int>> adj_graph) {
        InitialParam(n, adj_graph);
    }
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

class UnionFind {
    friend class MyGraph;
    std::vector<int> parent;
    std::vector<int> rank;
public:
    UnionFind() = default;
    explicit UnionFind(int n) :parent(n), rank(n) {
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    explicit UnionFind(std::vector<std::vector<int>> vec) {
        int rows = vec.size(), cols = vec[0].size();
        parent.resize(rows * cols);
        rank.resize(rows * cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = r * cols + c;
                parent[idx] = idx;
            }
        }
    }

    explicit UnionFind(std::vector<int> vec) {
        int n = vec.size();
        parent.resize(n);
        rank.resize(n);
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    int find(int x) {
        while (x != parent[x]) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    int find2(int x) {
        if (x != parent[x])
            parent[x] = find2(parent[x]);
        return parent[x];
    }

    bool unite(int x, int y) {
        int root_x = this->find(x);
        int root_y = this->find(y);

        if (root_x == root_y) return false;

        if (rank[root_x] < rank[root_y])
            parent[root_x] = root_y;
        else if (rank[root_x] > rank[root_y])
            parent[root_y] = root_x;
        else {
            parent[root_y] = root_x;
            rank[root_x]++;
        }
        return true;
    }

    bool sameSet(int x, int y) {
        return find(x) == find(y);
    }

    void clear() {
        rank.clear();
        parent.clear();

        rank.shrink_to_fit();
        parent.shrink_to_fit();
    }

    ~UnionFind() {
        clear();
    }
};


extern DLL_API MyGraph ClassTemplateInstance;
