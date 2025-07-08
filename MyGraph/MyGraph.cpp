/*Oscar MyGraph*/

#include <iostream>
#include <cstdlib>  // rand(), srand()
#include <ctime>    // time()
#include <random>   // C++11 亂數庫
#include <vector>   // std::vector
#include <numeric>  // std::iota
#include <map>      //std::map
#include <unordered_map>  //std::unordered_map

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


    return 0;
    #pragma endregion

    
}

#pragma region MyGraph

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


