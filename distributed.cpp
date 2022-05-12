#include <cstdio>
//#include <conio.h>  
#include <cstdlib>  
#include <time.h>  
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#include <cstring>
#include <memory>
#include <cmath>
#include <ctime>
#include <set>
#include <direct.h>
#include <io.h>
#include <iomanip>

using namespace std;

struct Task
{
    int id;
    int born_node;
    int born_domain;
    float data_size;
    float delay;
    float born_time;

    int failed = 0;//是否超时
    int finished = 0;//是否已完成
    int done = 0;//任务是否已经被分配
    int end_node = 0;
    int route_bucket[150];//任务路径数组，第i个元素如果为1证明i节点被走过
    int current_node;
    float weight = 0;
    float left_time;


};


struct Node
{
    int id;
    float capacity;
    float remain;
    float domain_num;
    int current_task[10000];//该节点中任务id
    int current_task_num = 0;
    int test = 0;
};


//string d_route = "./cov_data_30_v0.csv";  // 格式化网络数据文件
string rd_route = "./data_30_v0.csv";    // 原始网络数据文件
string o_route = "./out_30_v0.csv";  // 输出数据路径
string work_list = "./1.csv";

int K, N, D;  //聚类的数目，节点数，数据的维数
float** net_data;  // 存放数据
float** cp_diff;  // 存放各点间算力差
int* in_cluster;  //标记每个点属于哪个聚类
float** cluster_center;  //存放每个聚类的中心点
vector<vector<int>> points_list;  //存放分域后的各域节点
float T1, T2, K1=0.6, K2=0.5;  //每轮更新的时延约束及其系数

float** r_net_data;  // 存放数据
float** r_cp_diff;  // 存放各点间算力差
int ttimes =20;
double dur = 0;
clock_t start_t;
clock_t end_t;
clock_t cout_start;
clock_t cout_end;
double cout_t;

int node_num; 
Node node_list[200]; //节点数据

int total_task_num;//总任务队列
Task total_task_list[200010];

int task_num;
Task task_list[6500];//任务数据

float node_graph[200][200];//整个图节点传输时延的邻接矩阵

float total_value = 0;

float distributed_t = 0;
float system_t = 0;//当前时间
float schedule_t = 0;//调度时间
int current_task_id;//目前进行到哪个任务了
int finish_task_num = 0;

const int Population_size = 10; //种群数,为偶数
int population[300][6500];//种群矩阵
const float crossover_rate = 0.5; //交叉互换概率         
const float mutation_rate = 0.01; //变异概率
const int iteration_num = 10;  //迭代次数
int live[100][6500];//存活种群矩阵

double domainDiv();
void kmeans();
float** createarray(int m, int n);
void freearray(float** p);
// void loadData(int* d, int* n);
// void storeData();
float getDistance(float avector[], float bvector[], int n);
void cluster();
float getDifference();
void getCenter(int in_cluster[]);
void incrementalGreedy();
void getController();

void loadCsvData();
void storeCsvData();


void init(float** work, float** route) //初始化task_list,node和domain_list
{
    //cout << "gooood!" << endl;
    node_num = route[0][1];
    total_task_num = work[0][0];
    //total_task_num = 600;
    
    //int cycle = rand()%4 + 6;
    float born_t = 0;
    int interval_t = 1;
    int task_per_sec = 3;
    for(int i = 0; i < total_task_num; i++)//初始化总任务
    {
        Task t;
        t.id = i;
        t.born_node = work[i + 1][1];
        t.data_size = 10*work[i + 1][2];
        t.delay = work[i + 1][3];
        t.left_time = work[i + 1][3];
        //t.born_domain = index[t.born_node];
        t.current_node = work[i + 1][1];
        if(i % (interval_t * task_per_sec) == 0 && i != 0)
            born_t += interval_t;
        t.born_time = born_t;
        for(int j = 0; j < node_num; j++)
            t.route_bucket[j] = 0;
        t.route_bucket[int(work[i + 1][1])] = 1;
        total_task_list[i] = t;
        //if(cycle == 0)
        //{
            //cycle = 2;
            //tt += 1;
        //}
    }

    for(int i = 0; i < node_num; i++)//初始化节点
    {
        Node n;
        n.id = i;
        n.capacity = route[i + 1][node_num];
        n.remain = route[i + 1][node_num];
        //n.domain_num = index[i];
        n.current_task_num = 0;
        //for(int j = 0; j < task_num; j++)
            //if(task_list[j].born_node == i)
            //{
                //n.current_task[n.current_task_num] = j;
                //n.current_task_num ++;
            //}
        node_list[i] = n;
    }

    for(int i = 0; i < node_num; i++)//初始化节点路径拓扑
        for(int j = 0; j < node_num; j++) 
            node_graph[i][j] = route[i + 1][j];
            
    
}
void update(float** work, float** route)
{
    for(int i = 0; i < node_num; i++)
    {
        node_list[i].current_task_num = 0;
        for(int j = 0; j < task_num; j++)
            if(task_list[j].born_node == i)
            {
                node_list[i].current_task[node_list[i].current_task_num] = j;
                node_list[i].current_task_num ++;
            }
    }
}


void crossover(int gene_len)//对种群进行一次交叉互换
{
    if(gene_len != 0)
    {
    int order[6500];
    for(int i = 0; i < Population_size; i++)//初始化顺序数组
        order[i] = i;
    for(int i = (Population_size - 1); i > 0; i--)//洗牌算法洗混排序
        {
            int pick_num;
            if(i != 1)
                pick_num = rand()%(i - 1);
            else
                pick_num = 0;
            int temp_num = order[i];
            order[i] = order[pick_num];
            order[pick_num] = temp_num;
        }
    
    for(int i = 0; i < Population_size; i+=2)//按照顺序交叉互换
    {
        if(rand()/double(RAND_MAX) <= crossover_rate)
        {
            int end = rand()%gene_len;//交叉互换结束点s
            int start = rand()%(end + 1);//交叉互换开始点
            for(int j = start; j <= end; j++)
            {
                int temp_num;
                temp_num = population[Population_size + order[i]][j];
                population[Population_size + order[i]][j] = population[Population_size + order[i + 1]][j];
                population[Population_size + order[i + 1]][j] = temp_num;
            }
        }
    }
    }
    //cout << "okk ";
}

void mutation(int gene_len, int gene_class[200], int class_len)//对种群进行一次变异
{
    if(gene_len!=0)
    {
    for(int i = Population_size; i < 2 * Population_size; i++)
        if(rand()/double(RAND_MAX) <= mutation_rate)
        {
            float aa = gene_class[rand()%class_len];
            if(rand()%(gene_len + 1) != 0 && aa > 0)
                population[i][rand()%gene_len] = aa;
            else
                population[i][rand()%gene_len] = 0;
        }
            
    }
}

void duplicate(int gen_len)//将种群复制一份到矩阵下面
{
    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < gen_len; j++)
           population[i + Population_size][j] = population[i][j];
}

void distributed_population_factory(Node n, Task T_list[6000], int T_list_num)//每个域生成种群，种群个体为贪心算法所得
{
    float copy = node_list[n.id].remain;
    for(int i = 0;i < Population_size; i++)
        for(int j = 0; j < T_list_num; j++)
            population[i][j] = 0;
    
    for(int i = 0; i < Population_size; i++)
    {
        int task_order[10050];//选取物品的顺序
        for(int j = 0; j < T_list_num; j++)//初始化顺序数组
            task_order[j] = j;
        for(int j = (T_list_num - 1); j > 0; j--)//洗牌算法洗混排序
        {
            int pick_num = 0;
            if(j != 1)
                pick_num = rand()%(j - 1);
            int temp_num = task_order[j];
            task_order[j] = task_order[pick_num];
            task_order[pick_num] = temp_num;
        }
        //for(int j = 0; j < T_list_num; j++)//输出顺序数组
            //cout << task_order[j] << " ";
        //cout << endl;
        
        for(int j = 0; j < T_list_num; j++)//按顺序将物品装入背包
        {
            float weight = T_list[task_order[j]].data_size / T_list[task_order[j]].left_time;
            //cout << " weight:" << weight;
             if(weight <= node_list[n.id].remain)
            {
                node_list[n.id].remain -= weight;
                population[i][task_order[j]] = 1;
                //break;
            }
            //cout << " remain:" << n.remain;
        }
        //cout << endl;
        node_list[n.id].remain = copy;
    }       
}

void distributed_selection(Node n, Task T_list[6500], int T_list_len)//将父种群和子种群进行筛选
{
    float value_fitness[200];
    for(int i = 0; i < 2 * Population_size; i++)//计算种群适应值
    {
        int flag = 1;
        float contain = 0;
        for(int j = 0; j < T_list_len; j++)//判断是否为可行解
        {
            if(population[i][j] != 0)
            {
                float weight = T_list[j].data_size / T_list[j].left_time;
                contain += weight;
            }
        }
        if(contain > n.remain)
            flag = 0;
        if(flag == 1)//计算基因适应值
        {
            float value_sum = 0;
            for(int j = 0; j < T_list_len; j++)
            {
                if(population[i][j] != 0)
                {
                    float weight = T_list[j].data_size / T_list[j].left_time;
                    value_sum += 1/weight;
                }
            }                
            value_fitness[i] = value_sum;
        }
        else
            value_fitness[i] = 0;
    }
    /*
    cout << endl;
    for(int i = 0; i < 2 * Population_size; i++)
        cout << value_fitness[i] << " ";
    cout << endl;
    */
    for(int i = 0; i < Population_size; i++)//挑选种群中价值较大的个体
    {
        float max = 0;
        int max_num = 0;
        for(int j = 0; j < 2 * Population_size; j++)
        {
            if(max <= value_fitness[j])
            {
                max = value_fitness[j];
                max_num = j;
            }
        }
        for(int j = 0; j < T_list_len; j++)
            live[i][j] = population[max_num][j];
        value_fitness[max_num] = 0;
    }
    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < T_list_len; j++)
            population[i][j] = live[i][j];

}
int knapsack(Node node, Task T_list[6000], int T_list_len, int index[1000])
{
    
    int count = 0;
    int gene_class[2] = {0, 1};
    distributed_population_factory(node, T_list, T_list_len);
    for(int i = 0; i < iteration_num; i++)
    {
        duplicate(T_list_len);
        crossover(T_list_len);
        mutation(T_list_len, gene_class, 2);
        distributed_selection(node, T_list, T_list_len);
    }
    for(int i = 0; i < T_list_len; i++)
        if(population[0][i] != 0)
            {
                node_list[node.id].remain -= T_list[i].data_size/T_list[i].left_time;
                task_list[index[i]].weight = T_list[i].data_size/T_list[i].left_time;
                total_value += T_list[i].left_time/T_list[i].data_size;
                task_list[index[i]].done = 1;
                task_list[index[i]].end_node = node.id;
                count++;
            }
    
    
    return count;
}

void distributed()
{
    for(int i = 0; i < task_num; i++)
    {
        task_list[i].end_node = 0;
        task_list[i].left_time = task_list[i].delay;
        task_list[i].current_node = task_list[i].born_node;
        for(int k = 0; k < node_num; k++)
            task_list[i].route_bucket[k] = 0;
        task_list[i].route_bucket[i] = 1;
    }
    int count = 0;
    //int c = 10000;
    Task *Tlist = (Task*) malloc(sizeof(Task) * 6000);
    int round = 0;
    int index[1000];
    while(count < task_num && round < node_num)
    {
        cout_start = clock();
        cout << "round:" << round;
        cout_end = clock();
        dur = (double)(cout_end - cout_start) * 1000;
        schedule_t -= dur/CLOCKS_PER_SEC;
        //cout << "  2.1 ";
        for(int i = 0; i < node_num; i++)//节点内部背包
        {
            for(int j = 0; j < node_list[i].current_task_num; j++)
            {
                Tlist[j] = task_list[node_list[i].current_task[j]];
                index[j] = node_list[i].current_task[j];
            }
            int c = 0;
            if(node_list[i].current_task_num != 0)
            {
                cout_start = clock();
                cout <<" node: " << i << " task_num: "<<node_list[i].current_task_num;
                cout_end = clock();
                dur = (double)(cout_end - cout_start) * 1000;
                schedule_t -= dur/CLOCKS_PER_SEC;
                c = knapsack(node_list[i], Tlist, node_list[i].current_task_num, index);
            }
                
            count += c;
            for(int j = 0; j < node_list[i].current_task_num; j++)//清空节点的current task
                node_list[i].current_task[j] = 0;
            node_list[i].current_task_num = 0;
        }
        cout << endl;
        //cout << "ok!";
        //if (count >= task_num)
            //break;
        //cout << "  2.2 ";
        for(int i = 0; i < task_num; i++)//向其他节点传输
        {
            if(task_list[i].done == 0 && task_list[i].end_node != -1)
            {
                int current = task_list[i].current_node;
                float min = 1000;
                int des = -1;
                for(int j = 0; j < node_num; j++)
                {
                    if(min > node_graph[current][j] && node_graph[current][j] == 1 && task_list[i].route_bucket[j] != 1 && task_list[i].left_time > node_graph[current][j] && current != j)
                    {
                        min = node_graph[current][j];
                        des = j;
                    }
                }
                if(des >= 0)
                {
                    node_list[des].current_task[node_list[des].current_task_num] = i;
                    node_list[des].current_task_num++;

                    task_list[i].current_node = des;
                    task_list[i].route_bucket[des] = 1;
                    task_list[i].left_time -= min;
                }
                else
                {

                    task_list[i].end_node = -1;
                    count ++;
                    for(int k = 0; k < node_num; k++)
                    {
                        task_list[i].route_bucket[k] = 0;
                    }
                    task_list[i].route_bucket[i] = 1;
                }
            }
        }
        round ++;
        //cout << endl;
        //cout <<  "count: "<<count << endl;
    }
    free(Tlist);
    for(int i = 0; i< task_num; i++)
    {
        task_list[i].left_time = task_list[i].delay;
    }
    //cout << "good!" << endl;
}

int task_update()
{
    int result = 0;
    //cout<<"system t:" << system_t;
    task_num = 0;
    for(int i = 0; i < total_task_num; i++)//找到当前时间所有任务序列
    {
        if(total_task_list[i].born_time < system_t)
            current_task_id = total_task_list[i].id;
    }
    //cout <<" current_task_id:"<< current_task_id;
    for(int i = 0; i <= current_task_id; i++)
    {
        if(total_task_list[i].finished == 0)
        {
            if(total_task_list[i].born_time + total_task_list[i].delay < system_t)
            {
                if(total_task_list[i].done == 0)//标记超时任务
                {
                    total_task_list[i].failed = 1;
                    result++;
                }
                    
                else//释放任务资源
                {   
                    node_list[total_task_list[i].end_node].remain += total_task_list[i].weight;
                    finish_task_num++;
                }
                    
                total_task_list[i].finished = 1;
            }
            else
            {
                if(total_task_list[i].done == 0)
                {
                    task_list[task_num] = total_task_list[i];
                    task_num++;
                }
            }
        }
    }
    //cout <<" task_num:" << task_num << endl;
    return result;
}
int main()
{
    //cout << "aa";
    /*
    float work[6000][10] = {{5, 10},
                            {0, 1, 0, 12, 6},//任务列表，格式为 （id，类型，来源，数据量，容忍时延）
                            {1, 2, 0, 12, 8},
                            {2, 1, 1, 18, 6},
                            {3, 2, 1, 20, 10},
                            {4, 1, 2, 5,  7},
                            {5, 2, 2, 12, 12},
                            {6, 1, 3, 15, 8},
                            {7, 2, 3, 22, 11},
                            {8, 1, 4, 20, 9},
                            {9, 2, 4, 7,  14}};
    float route[200][200] = {{6, 5},
                             {0, 2, 2, 4, 4, 6},//各个节点之间最短传输时延，最后一列为各节点算力
                             {2, 0, 4, 3, 5, 8},
                             {2, 4, 0, 6, 5, 12},
                             {4, 3, 6, 0, 2, 7},
                             {4, 5, 5, 2, 0, 10}};
    float domain_input[100][100] = {{2},//分域数量
                                    {0, 3, 7, 4, 0, 1, 3},//域内情况，格式为（控制器id，域内节点数，域内最小算力，节点到控制器最大时延）
                                    {4, 2, 10, 5, 2, 4}};

    
    */
    
    std::vector<vector<float>> work0;//读入work
    ifstream fp1(work_list); //定义声明一个ifstream对象，指定文件路径
    string line1;
    //getline(fp,line); //跳过列名，第一行不做处理
    while (getline(fp1,line1))
    { //循环读取每行数据
        vector<float> data_line;
        string number;
        istringstream readstr(line1); //string数据流化
        //将一行数据按'，'分割
        for(int j = 0;j < 4;j++)
        { //可根据数据的实际情况取循环获取
            getline(readstr,number,','); //循环读取数据
            data_line.push_back(atof(number.c_str())); //字符串传float
        }
        work0.push_back(data_line); //插入到vector中
    }
    //cout<<"aa";
    
    std::vector<vector<float>> route0;//读入route
    ifstream fp2(rd_route); //定义声明一个ifstream对象，指定文件路径
    string line2;
    //getline(fp,line); //跳过列名，第一行不做处理
    while (getline(fp2,line2))
    { //循环读取每行数据
        vector<float> data_line;
        string number;
        istringstream readstr(line2); //string数据流化
        //将一行数据按'，'分割
        for(int j = 0;j < 101;j++)
        { //可根据数据的实际情况取循环获取
            getline(readstr,number,','); //循环读取数据
            data_line.push_back(atof(number.c_str())); //字符串传float
        }
        route0.push_back(data_line); //插入到vector中
    }


    float **work=(float ** )malloc(sizeof(float *)*(work0[0][0] + 1)); 
    for(int i = 0;i <= work0[0][0];++i) 
        work[i] = (float*)malloc(sizeof(float)*10);
    
    //cout << work0[0][0];

    for(int i = 0; i <= work0[0][0];++i)
        for(int j = 0; j < 4;++j)
            work[i][j] = work0[i][j];
    
    //cout << "dd";
   

    //float route[101][101];
    float **route=(float ** )malloc(sizeof(float *)*(101)); 
    for(int i = 0;i <= 101;++i) 
        route[i] = (float*)malloc(sizeof(float)*101);
    int a = route0[0][0]; 
    for(int i = 0; i < a;i++)
        for(int j = 0; j < a ;j++)
        {
            float b = route0[i][j];
            //cout << b;
            route[i][j] = b;
        }
           
  
    //cout << "aa";       
    //cout << "ss";
    init(work, route);
    
    //double dur;
    //div_time = domainDiv();
    

    //cout << "ss";
    int count = 0;
    int t_flag = 0;
    int done_flag = 0;
    int jump_ms = 1;
    int total_dead = 0;


    
    Task *Tlist = (Task*) malloc(sizeof(Task) * 200);
    
    int **population_copy=NULL;//复制population数组 
    
    population_copy=(int ** )malloc(sizeof(int *)*10); 
    for(int i = 0;i < 20;++i)
        population_copy[i] = (int*)malloc(sizeof(int)*100);


    //system_t += 1000;
    while(done_flag == 0 && count < 10000)
    {
            
        //cout <<endl <<  "1" << " ";
        done_flag = 1;
        int dead = task_update();
        
        update(work, route);
        

        for(int i = 0; i < total_task_num; i++)
        {
            if(total_task_list[i].finished == 0)
            {
                done_flag = 0;
                break;
            }
        }

            //continue;
        if(task_num == 0)
        {
            system_t += jump_ms;
            if(total_task_list[total_task_num].done == 1)
                t_flag = 1;
            continue;
        }
            
        //distribute
        //cout << "2" << " ";

        start_t = clock();//分布式
        distributed();
        end_t = clock();
        dur = (double)(end_t - start_t) * 1000;
        schedule_t += dur/CLOCKS_PER_SEC;

        //cout << "3" << " ";
        
        
        //cout << endl << "node_remain: ";
        for(int i = 0; i < node_num;i++)
            //cout << node_list[i].remain << " ";
            
        //cout << endl;
        for(int i = 0; i < task_num; i++)//将task_num信息同步到总表上
            total_task_list[task_list[i].id] = task_list[i];
        //cout << endl;
        //更改node csv文件
        
        ofstream ou(rd_route);
        ou << node_num + 1 << "," << node_num << endl;
        for(int i = 1; i <= node_num; i++)
        {
            for(int j = 0; j < node_num; j++)
            {
                ou << route[i][j] << ",";
            }
            ou << node_list[i - 1].remain << endl;
        }
        

        system_t += schedule_t;
        //cout << "7" << endl;
        

        count ++; 

        //cout <<" count: " << count;//当前时隙id 0
        //cout << " time: "<< schedule_t + kmeans_t <<" ";//当前时隙长短1
        //cout << " task_num: " << task_num;//当前时隙需要处理任务量2
        //cout <<" failed task num: " << dead<< " ";//本时隙超时任务数3

        cout << count << "," << schedule_t << "," << finish_task_num << "," <<  dead << endl;
        finish_task_num = 0;
        //cout << " total_value: " << total_value << " ";
        for(int i = 0; i < current_task_id; i++)
        if(total_task_list[i].failed == 1)
            total_dead ++;
        //cout << " current_dead:" << total_dead << " ";
        //cout << " current_task_id: " << current_task_id << " ";
        //cout << " time: " << system_t;
        cout << task_num << endl;
        cout << total_value << "," << total_dead << "," << current_task_id << "," << system_t << endl << endl;
        total_dead = 0;
        schedule_t = 0;
        //cout << task_num << endl; 
        //if(system_t > 60000)
            //break;
    }
}


