/*Oscar MyGraph*/

#include <iostream>
#include <cstdlib>  // rand(), srand()
#include <ctime>    // time()
#include <random>   // C++11 亂數庫
#include <vector>   // std::vector
#include <numeric>  // std::iota
#include <map>      //std::map
#include <unordered_map>  //std::unordered_map
#include <unordered_set>  //std::unordered_set
#include <functional>

#include <cassert>

#include"MyGraph.h"
using namespace std;

DLL_API MyGraph ClassTemplateInstance;
enum LeetcodeExam {
    Leetcodexxx,

    None,
};


int main()
{
    //try case
    LeetcodeExam ExamEnum = Leetcodexxx;    //ChangeForExam
    //intput
    vector<int> vInput1 = { 7,13,11,10,1 };              
    vector<int> vInput2 = { 7,13,11,10,1 };              
    vector<vector<int>> vvInput1 = { {1,2} ,{2,3},{3,4},{1,3} };
    string strinput1 = "bab";
    string strinput2 = "xaabacxcabaaxcabaax";
    int iInput1 = 0;int iInput2 = 0;
    //output
    int Ans = 0; vector<int> AnsVector; string AnsStr = "";bool Ansbool = false;
    MyGraph* Implementation = new MyGraph();

    switch (ExamEnum)
    {
    case Leetcodexxx:
        AnsVector = Implementation->Leetcode_Sol_xxx(vInput1, iInput1,1);
        break;

    default:
        break;
    }

    #pragma region MyGraph
    #pragma region Dijkstra Alogrithm
    std::cout << "====== 測試一、 Dijkstra 測試 ======" << std::endl;
    std::cout << "測試1. Dijkstra 功能：" << std::endl;
    Dijkstra d;
    // 加邊
    d.push(0, 1, 2);
    d.push(0, 2, 4);
    d.push(1, 2, 1);
    d.push(1, 3, 7);
    d.push(2, 4, 3);
    d.push(3, 4, 1);

    // 計算從節點 0 出發的最短路徑
    vector<int> dist = d.getlist(0);
    d.print_dist(0);
    std::cout << "測試2. Dijkstra 建構式 vector<unordered_map<int, int>>：" << std::endl;
    // 測試 2：用 vector<unordered_map<int,int>> 建構子
    vector<unordered_map<int, int>> raw_graph2(5);
    raw_graph2[0][1] = 2;
    raw_graph2[0][2] = 4;
    raw_graph2[1][2] = 1;
    raw_graph2[1][3] = 7;
    raw_graph2[2][4] = 3;
    raw_graph2[3][4] = 1;
    Dijkstra d2(raw_graph2);
    auto dist2 = d2.getlist(0);
    d2.print_dist(0);
    std::cout << "測試3. Dijkstra 建構式 vector<vector<pair<int, int>>>：" << std::endl;
    // 測試 3：用 vector<vector<pair<int,int>>> 建構子
    vector<vector<pair<int, int>>> raw_graph3(5);
    raw_graph3[0].emplace_back(1, 2);
    raw_graph3[0].emplace_back(2, 4);
    raw_graph3[1].emplace_back(2, 1);
    raw_graph3[1].emplace_back(3, 7);
    raw_graph3[2].emplace_back(4, 3);
    raw_graph3[3].emplace_back(4, 1);
    Dijkstra d3(raw_graph3);
    auto dist3 = d3.getlist(0);
    d3.print_dist(0);
    /*預期輸出，從節點 0 出發的最短距離：
        節點 0: 0
        節點 1: 2
        節點 2: 3
        節點 3: 9
        節點 4: 6 */
    #pragma endregion

    #pragma region Bellman-Ford Alogrithm
    std::cout << "\n====== 測試二、 Bellman-Ford 測試 ======" << std::endl;
    std::cout << "測試1. Bellman-Ford 建構式：" << std::endl;
    vector<vector<pair<int, int>>> graph1 = {
        {{1, 2}, {2, 4}}, // 0 → 1(w=2), 2(w=4)
        {{2, 1}},         // 1 → 2(w=1)
        {}               // 2
    };
    BellmanFord bf(graph1);
    auto bv = bf.getlist(0);
    bf.print_result();
    std::cout << "測試2. Bellman-Ford push功能測試：" << std::endl;

    bf.push(2, 0, 7); // 加一條 2→0(w=7)
    auto bv2 = bf.getlist(1);
    bf.print_result();

    std::cout << "測試3. Bellman-Ford erase功能測試：" << std::endl;
    bf.erase(0, 1); // 移除 0→1
    auto bv3 = bf.getlist(0);
    bf.print_result();

    std::cout << "測試4. IsNegative_cycle (正常情況)：" << std::endl;
    cout << (bf.IsNegative_cycle(0) ? "有負環\n" : "無負環\n");

    std::cout << "測試5. IsNegative_cycle (加上負環)：" << std::endl;
    bf.push(1, 0, -10); // 1→0
    bf.push(0, 1, -10); // 0→1，構成負環
    cout << (bf.IsNegative_cycle(0) ? "有負環\n" : "無負環\n");

    std::cout << "測試6. Bellman-Ford clean功能測試：" << std::endl;
    bf.clean();
    auto bv4 = bf.getlist(0);
    cout << "size = " << bv4.size() << ", capacity = " << bv4.capacity() << '\n'; // 應該是 0
    /*=== 測試建構式 ===
        dist[0] = 0
        dist[1] = 2
        dist[2] = 3

      === 測試 push ===
        dist[0] = 8
        dist[1] = 0
        dist[2] = 1

      === 測試 erase ===
        dist[0] = 0
        dist[1] = INF
        dist[2] = 4

      === 測試 IsNegative_cycle (正常情況) ===
       無負環

      === 測試加上負環 ===
       有負環

      === 測試 clean ===
       size = 0, capacity = 0
    */
    #pragma endregion

    #pragma region Floyd-Warshall
    std::cout << "\n====== 測試三、 Floyd-Warshall 測試 ======" << std::endl;
    vector<vector<pair<int, int>>> graph = {
        { {1, 2}, {2, 4} },   // 0 → 1(w=2), 2(w=4)
        { {2, 1} },           // 1 → 2(w=1)
        {}                   // 2
    };

    cout << "測試1. Floyd-Warshall 建構式：\n";
    FolydWarshall fw(graph);
    int** fwdist = fw.getlist();
    fw.printMatrix();

    cout << "測試2. Floyd-Warshall 功能 push()：\n";
    vector<vector<pair<int, int>>> new_graph = {
        { {1, 3} },       // 0 → 1(w=3)
        { {2, 5} },       // 1 → 2(w=5)
        { {0, 1} }        // 2 → 0(w=1)
    };
    fw.push(new_graph);
    fwdist = fw.getlist();
    fw.printMatrix();

    cout << "測試2. Floyd-Warshall 功能 空 graph ：\n";
    vector<vector<pair<int, int>>> empty_graph(0);
    fw.push(empty_graph); // 也應該安全
    cout << "done.\n";
    #pragma endregion

    #pragma region Topological Sort
    std::cout << "\n====== 測試四、 Topological Sort 測試 ======" << std::endl;
    // 測試1：建構式 + 拓撲排序
    std::vector<std::vector<int>> graph2 = {
        {1, 2}, // 0 -> 1, 2
        {2},    // 1 -> 2
        {}      // 2
    };
    TopologicalSort ts1(graph2);
    int* order1 = ts1.getlist();
    std::cout << "Test1. Topological order:\n";
    ts1.print();


    // 測試2：用push更新圖，並跑拓撲排序
    std::vector<std::vector<int>> graph3 = {
        {2},    // 0 -> 2
        {2},    // 1 -> 2
        {}      // 2
    };
    ts1.push(graph3);
    int* order2 = ts1.getlist();
    cout << "測試2. TopologicalSort 功能 push：\n";
    ts1.print();

    // 測試3：含有環的圖，應該會印警告
    std::vector<std::vector<int>> graph4 = {
        {1},    // 0 -> 1
        {2},    // 1 -> 2
        {0}     // 2 -> 0，形成環
    };
    ts1.push(graph4);
    int* order3 = ts1.getlist();
    cout << "測試3. TopologicalSort 功能 (測試環)：\n";
    ts1.print();

    // 測試4：進階 DAG
    std::vector<std::vector<int>> graph5 = {
    {1, 2},      // 0 → 1, 2
    {3},         // 1 → 3
    {3, 4},      // 2 → 3, 4
    {5},         // 3 → 5
    {5, 6},      // 4 → 5, 6
    {7},         // 5 → 7
    {7},         // 6 → 7
    {8},         // 7 → 8
    {},          // 8
    {2, 4}       // 9 → 2, 4
    };
    ts1.push(graph5);
    int* order4 = ts1.getlist();
    cout << "測試4. TopologicalSort 進階 DAG：\n";
    ts1.print();
    /*說明：
      這張圖有多種可能合法拓撲序列，舉例幾個：
      0 9 1 2 4 3 6 5 7 8
      9 0 2 4 6 1 3 5 7 8
      9 0 1 2 3 4 5 6 7 8
      任何順序只要滿足以下條件就合法：
      0 必須在 1, 2, 3, 4, 5, 7, 8 前面
      9 必須在 2, 4, 3, 5, 6, 7, 8 前面
      1 在 3, 5, 7, 8 前
      2 在 3, 4, 5, 6, 7, 8 前
      4 在 5, 6, 7, 8 前
      6 在 7, 8 前
      3, 5, 7 在 8 前*/
    #pragma endregion

    #pragma region UnionFind
    std::cout << "\n====== 測試五、 Union Find 測試 ======" << std::endl;
    // 建構子：UnionFind(int n)
    cout << "測試1. Union Find 建構式 UnionFind(int n)：\n";
    int uf_size = 5;
    UnionFind uf1(uf_size);
    cout << "所有初始 root：";
    for (int i = 0; i < uf_size; i++)
        cout << uf1.find(i) << " ";
    assert(uf1.sameSet(0, 1) == false);
    uf1.unite(0, 1);
    assert(uf1.sameSet(0, 1) == true);
    uf1.unite(1, 2);
    assert(uf1.sameSet(0, 2) == true);
    cout << "\n相連(0 ~ 2)後：";
    for (int i = 0; i < uf_size; i++)
        cout << uf1.find(i) << " ";
    cout << "\n[OK] 測試1. Union Find 功能 & 建構式，通過!\n";

    // 建構子：UnionFind(vector<int>)
    cout << "測試2. Union Find 建構式 UnionFind(vector<int>)：\n";
    std::vector<int> vec_uf = { 10, 20, 30 };
    UnionFind uf2(vec_uf);
    cout << "所有初始 root：";
    for (int i = 0; i < vec_uf.size(); i++)
        cout << uf2.find(i) << " ";
    assert(uf2.sameSet(1, 2) == false);
    uf2.unite(1, 2);
    assert(uf2.sameSet(1, 2) == true);
    cout << "\n相連(1 ~ 2)後：";
    for (int i = 0; i < vec_uf.size(); i++)
        cout << uf2.find(i) << " ";
    cout << "\n[OK] 測試2. Union Find 功能 & 建構式，通過!\n";


    // 建構子：UnionFind(vector<vector<int>>)
    cout << "測試3. Union Find 建構式 UnionFind(vector<vector<int>>)：\n";
    std::vector<std::vector<int>> grid_uf = {
        {1, 0, 1},
        {0, 1, 0},
        {1, 1, 1}
    };
    UnionFind uf3(grid_uf); // 應該有 9 個節點
    cout << "所有初始 root：";
    for(int r = 0;r < grid_uf.size();r++)
        for (int c = 0; c < grid_uf[0].size(); c++) {
            int idx_uf = r * grid_uf[0].size() + c;
            cout << uf3.find(idx_uf) << " ";
        }
    int cols = grid_uf[0].size();
    int a = 0 * cols + 0; // (0,0)
    int b = 1 * cols + 1; // (1,1)
    assert(uf3.sameSet(a, b) == false);
    uf3.unite(a, b);
    assert(uf3.sameSet(a, b) == true);
    cout << "\n相連([0,0] - [1,1])後：";
    for (int r = 0; r < grid_uf.size(); r++)
        for (int c = 0; c < grid_uf[0].size(); c++) {
            int idx_uf = r * grid_uf[0].size() + c;
            cout << uf3.find(idx_uf) << " ";
        }
    // 測試 clear() 與 destructor
    uf1.clear(); // 不會崩
    cout << "\n[OK] 測試3. Union Find 功能 & 建構式，通過!\n";
    std::cout << "[OK] 所有 Union Find 測試通過!" << std::endl;  


    cout << "測試4. Union Find 大量連通：\n";
    int uf_size2 = 20;
    UnionFind uf4(uf_size2);
    cout << "所有初始 root：";
    for (int i = 0; i < uf_size2; i++)
        cout << uf4.find(i) << " ";

    for (int i = 1; i < uf_size2; i++) {
        if(i % 3 == 1)
            uf4.unite(1, i);
        else if(i % 3 == 2)
            uf4.unite(2, i);
        else
            uf4.unite(0, i);
    }
    cout << "\n3的餘數分root：";
    for (int i = 0; i < uf_size2; i++)
        cout << uf4.find(i) << " ";
    uf4.unite(0, 1);// => rank[0] = 2,rank[1] = 1
    uf4.unite(0, 2);
    cout << "\n最後全部相連：";
    for (int i = 0; i < uf_size2; i++)
        cout << uf4.find(i) << " ";
    cout << "\n[OK] 測試4. Union Find 功能 & 建構式，通過!\n";

    #pragma endregion

    #pragma region MST
    std::cout << "\n====== 測試六、 MST (Minimum Spanning Tree) 測試 ======" << std::endl;
    MyGraph graph_MST;
    std::vector<std::vector<int>> edges_MST = {
        {0, 1, 1},  // 0 - 1 (1)
        {0, 2, 4},  // 0 - 2 (4)
        {1, 2, 2},  // 1 - 2 (2)
        {1, 3, 6},  // 1 - 3 (6)
        {2, 3, 3}   // 2 - 3 (3)
    };
    std::cout << "測試圖(Graph edges):\n";
    for (const auto& edge : edges_MST) {
        std::cout << edge[0] << " - " << edge[1] << " (" << edge[2] << ")\n";
    }

    // 建立鄰接表 (vector<vector<pair<int,int>>>)，用 pair<鄰接點, 權重>
    std::vector<std::vector<std::pair<int, int>>> adj(4);
    for (auto& e : edges_MST) {
        int u = e[0], v = e[1], w = e[2];
        adj[u].emplace_back(v, w);
        adj[v].emplace_back(u, w); // 無向圖雙向加入
    }
    int NodeSize_MST = 4;

    // 測試 Kruskal_Weight
    int kruskal_w = graph_MST.Kruskal_Weight(NodeSize_MST, edges_MST);
    std::cout << "測試1. Kruskal MST total weight: " << kruskal_w << std::endl;

    // 測試 Kruskal_State
    auto kruskal_mst = graph_MST.Kruskal_State(NodeSize_MST, edges_MST);
    std::cout << "測試2. Kruskal MST edges:" << std::endl;
    for (auto& e : kruskal_mst) {
        std::cout << e[0] << " - " << e[1] << " weight " << e[2] << std::endl;
    }

    // 測試 Prim_Weight
    int prim_w = graph_MST.Prim_Weight(NodeSize_MST, adj);
    std::cout << "測試3. Prim MST total weight: " << prim_w << std::endl;

    // 測試 Prim_State
    auto prim_mst = graph_MST.Prim_State(NodeSize_MST, adj);
    std::cout << "測試4. Prim MST edges:" << std::endl;
    for (auto& e : prim_mst) {
        if (e[0] == -1) continue; // 根節點的 parent 是 -1，跳過
        std::cout << e[0] << " - " << e[1] << " weight " << e[2] << std::endl;
    }
    #pragma endregion

    #pragma region Kosaraju && Tarjan (CC = Connected Component)
    std::cout << "\n====== 測試七、 Kosaraju && Tarjan (CC = Connected Component) 測試 ======" << std::endl;
    std::cout << "測試1. Tarjan find bridges (無向圖)：\n";
    int CC_n = 5;
    /* Bridge 測試圖： 答案：{1,3},{3,4}
            0
           / \
          1 - 2
          |
          3
          |
          4
    */
    vector<vector<int>> graphfor_Bridge = {
        {1, 2},
        {0, 2, 3},
        {0, 1},
        {1, 4},
        {3}
    };
    MyGraph tarjanBridge;
    std::cout << "Bridge 測試圖：\n";
    tarjanBridge.printGraph(graphfor_Bridge);
    // SCC (Tarjan)
    auto bridges = tarjanBridge.findBridges(CC_n, graphfor_Bridge);
    std::cout << "Bridge answer：{ 1,3 }, { 3,4 }\n";
    std::cout << "Tarjan find Bridges Result:\n";
    tarjanBridge.printGraphPair(bridges);


    std::cout << "測試2. Strongly Connected Component(SCC) (有向圖)：\n";
    /* Strongly Connected Component(SCC) & Weakly connected component(WCC) 測試圖：
        0 →  1 → 2 → 3 → 4
        ↑         ↓
        └─————   
    */
    int CC_node = 5;
    vector<vector<int>> graphforCC = {
        {1},     // 0
        {2},     // 1
        {0, 3},  // 2
        {4},     // 3
        {}       // 4
    };

    // SCC (Tarjan)
    MyGraph GraphCC;
    std::cout << "Connected Component 測試圖：\n";
    GraphCC.printGraph(graphforCC);
    std::cout << "SCCs answer = { 0, 1, 2 }, { 3 }, { 4 }\n";
    std::cout << "WCCs answer = { 0, 1, 2, 3, 4 }\n";
    cout << "Tarjan SCCs Result:\n";
    auto sccs = GraphCC.findSCCs_Tarjan(CC_node, graphforCC);
    GraphCC.printGraphCC(sccs);

    // SCC (Kosaraju)
    cout << "\nKosaraju SCCs Result:\n";
    auto kos_sccs = GraphCC.findSCCs_Kosaraju(CC_node, graphforCC);
    GraphCC.printGraphCC(kos_sccs);


    // WCC
    cout << "\nWeakly Connected Components Result:\n";
    auto wccs = GraphCC.findWWCs(CC_node, graphforCC);
    GraphCC.printGraphCC(wccs);
    #pragma endregion

    #pragma endregion

    return 0;
}

#pragma region MyGraph
int MyGraph::Kruskal_Weight(int n, std::vector<std::vector<int>> _edges) {
    //O(E log E) ≒ O(N^2 * log N^2) ≒ O(N^2 * log N)，每個節點最多有n - 1個，n個節點 => n * (n - 1)/2 (除以2是因為最後兩個節點只有一個邊，會重複一次)
    std::sort(_edges.begin(), _edges.end(), Myless);
    UnionFind uf(n);
    int mst_weight = 0;
    //O(E) ≒ O(N^2)
    for (std::vector<int>& edge : _edges) {
        int from = edge[0], to = edge[1];
        if (uf.unite(from, to)) {
            mst_weight += edge[2];
        }
    }
    return mst_weight;
}

std::vector<std::vector<int>> MyGraph::Kruskal_State(int n, std::vector<std::vector<int>> _edges) {
    std::sort(_edges.begin(), _edges.end(), Myless);
    UnionFind uf(n);
    std::vector<std::vector<int>> MST;
    for (std::vector<int> edge : _edges) {
        int from = edge[0], to = edge[1];
        if (uf.unite(from, to)) {
            MST.emplace_back(edge);
        }
    }
    return MST;
}

int MyGraph::Prim_Weight(int n, std::vector<std::vector<T>>& adj) {
    std::vector<bool> visited(n);
    std::priority_queue<T, std::vector<T>, std::greater<T>> minheap;
    minheap.emplace(0, 0);//// 從節點 0 開始 {權重, 節點}
    int mst_weight = 0;

    while (!minheap.empty()) {
        T cur = minheap.top(); minheap.pop();
        int cur_weight = cur.first;
        int cur_node = cur.second;

        if (visited[cur_node]) continue;
        visited[cur_node] = true;
        mst_weight += cur_weight;

        for (T& next : adj[cur_node]) {
            if (visited[next.first]) continue;
            minheap.emplace(next.second, next.first);
        }
    }

    return mst_weight;
}

std::vector<std::vector<int>> MyGraph::Prim_State(int n, std::vector<std::vector<T>>& adj) {
    std::vector<bool> visited(n);
    std::priority_queue<T, std::vector<T>, std::greater<T>> minheap;
    minheap.emplace(0, 0);//// 從節點 0 開始 {權重, 節點}
    std::vector<std::vector<int>> MST;
    std::vector<int> parent(n, -1);     // parent[i] 為從哪個點來的
    std::vector<int> weights(n, 1e9);       // 每個點的最小邊權重

    while (!minheap.empty()) {
        T cur = minheap.top(); minheap.pop();
        int cur_weight = cur.first;
        int cur_node = cur.second;
        if (visited[cur_node]) continue;
        visited[cur_node] = true;
        MST.emplace_back(vector<int>{parent[cur_node], cur_node, cur_weight});

        for (T& next : adj[cur_node]) {
            int next_node = next.first;
            int next_weight = next.second;
            if (visited[next_node] || next_weight >= weights[next_node]) continue;
            weights[next_node] = next_weight;
            parent[next_node] = cur_node;
            minheap.emplace(next_weight, next_node);
        }
    }

    return MST;
}

std::vector<pair<int,int>> MyGraph::findBridges(int n, std::vector<std::vector<int>> adj_graph) {
    TarjanBridge solver;
    solver.InitialParam(n, adj_graph);
    return solver.findBridges();
}

std::vector<std::vector<int>> MyGraph::findSCCs_Tarjan(int n, std::vector<std::vector<int>> adj_graph) {
    TarjanSCC solver;
    solver.InitialParam(n, adj_graph);
    return solver.findSCCs();
}

std::vector<std::vector<int>> MyGraph::findSCCs_Kosaraju(int n, std::vector<std::vector<int>> adj_graph) {
    Kosaraju solver;
    solver.InitialParam(n, adj_graph);
    return solver.findSCCs();
}

std::vector<std::vector<int>> MyGraph::findWWCs(int n, std::vector<std::vector<int>> adj_graph) {
    WCC solver;
    solver.InitialParam(n, adj_graph);
    return solver.findWCCs();
}

void MyGraph::printGraph(const vector<vector<int>>& adj_graph) {
    for (int i = 0; i < adj_graph.size(); ++i) {
        cout << i << " -> ";
        if (adj_graph[i].empty()) {
            cout << "X";
        }
        else {
            for (int j = 0; j < adj_graph[i].size(); ++j) {
                cout << adj_graph[i][j];
                if (j < adj_graph[i].size() - 1) cout << ", ";
            }
        }
        cout << endl;
    }
}


void MyGraph::printGraphPair(std::vector<pair<int,int>>& adj) {
    for (auto& [u, v] : adj) {
        std::cout << u << " - " << v << "\n";
    }
}

void MyGraph::printGraphCC(std::vector<std::vector<int>>& _CC) {
    for (auto& comp : _CC) {
        cout << "{";
        for (size_t i = 0; i < comp.size(); ++i) {
            cout << comp[i];
            if (i + 1 < comp.size()) cout << ", ";
        }
        cout << "} ";
    }
}

#pragma endregion

#pragma region Leetcode xxx. ExamName
//Leetcode xxx. ExamName
vector<int> MyGraph::Leetcode_Sol_xxx(vector<int>& numbers, int target,int _solution) {
    switch (_solution)
    {
    case 1:
        return Exam_xxx(numbers, target);
    default:
        return std::vector<int>{}; // 確保所有路徑都有回傳值
    }

    return{};
}

vector<int> MyGraph::Exam_xxx(vector<int>& numbers, int target) {
    return {};
}
#pragma endregion


