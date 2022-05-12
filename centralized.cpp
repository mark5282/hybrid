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
    int end_node = -1;
    int route_bucket[150];//任务路径数组，第i个元素如果为1证明i节点被走过
    int current_node;
    float weight = 0;
    float left_time;


};

struct Domain
{
    int id;
    int controller_id;
    int member_num;
    int member_id[150];
    float capacity;
    float remain;
    float max_delay;
    float min_capacity;
    float compute_time = 0;

    float duplicate_time = 0;
    float crossover_time = 0;
    float mutation_time = 0;
    float selection_time = 0;
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

int node_num; 
Node node_list[200]; //节点数据

int domain_num;
Domain domain_list[100];//域数据

int total_task_num;//总任务队列
Task total_task_list[200010];

int task_num;
Task task_list[6500];//任务数据

float domain_graph[100][100];//域间传输时延的邻接矩阵
float node_graph[200][200];//整个图节点传输时延的邻接矩阵

const int Population_size = 10; //种群数,为偶数
int population[300][6500];//种群矩阵
const float crossover_rate = 0.5; //交叉互换概率         
const float mutation_rate = 0.01; //变异概率
const int iteration_num = 10;  //迭代次数
int live[100][6500];//存活种群矩阵

float max_fitness[20][20];
int max_fitness_num = 0;
float total_value = 0;

float our_t = 0;//计时
float distributed_t = 0;
float centralized_t = 0;
float system_t = 0;//当前时间
float kmeans_t = 0;//分域时间
float schedule_t = 0;//调度时间
int current_task_id;//目前进行到哪个任务了
int tttt = 20;
float balance_fitness_max = 0;
int finish_task_num = 0;

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


void init(float** work, float domain_input[100][100], float** route) //初始化task_list,node和domain_list
{
    //cout << "gooood!" << endl;
    domain_num = domain_input[0][0];
    node_num = route[0][1];
    total_task_num = work[0][0];
    //total_task_num = 600;
    
    //int cycle = rand()%4 + 6;
    int cycle = 2;
    float tt = 0;
    int interval_t = 1;
    int task_per_sec = 3;
    for(int i = 0; i < total_task_num; i++)//初始化总任务
    {
        Task t;
        t.id = i;
        t.born_node = work[i + 1][1];
        t.data_size = ttimes*work[i + 1][2];
        t.delay = work[i + 1][3];
        t.left_time = work[i + 1][3];
        //t.born_domain = index[t.born_node];
        t.current_node = work[i + 1][1];
        if(i % (interval_t * task_per_sec) == 0 && i != 0)
            tt += interval_t;
        t.born_time = tt;
        for(int j = 0; j < node_num; j++)
            t.route_bucket[j] = 0;
        t.route_bucket[int(work[i + 1][1])] = 1;
        total_task_list[i] = t;
        cycle--;
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
void update(float** work, float domain_input[100][100], float** route)
{
    
    domain_num = domain_input[0][0];
    for(int i = 0; i < domain_num; i++)//初始化域
    {
        Domain d;
        d.id = i;
        d.controller_id = domain_input[i + 1][0];
        d.member_num = domain_input[i + 1][1];
        d.min_capacity = domain_input[i + 1][2];
        d.max_delay = domain_input[i + 1][3];
        d.remain = 0;
        for(int j = 0; j < d.member_num; j++)
        {
            d.member_id[j] = domain_input[i + 1][j + 4];
            d.remain += route[d.member_id[j] + 1][node_num];
        }
        d.capacity = d.remain;
        domain_list[i] = d;    
    }
    
    int index[200];//记录每个节点属于哪个域
    for(int i = 0; i < domain_input[0][0]; i++)
        for(int j = 0; j < domain_input[i + 1][1]; j++)
            index[int(domain_input[i + 1][j + 4])] = i;
    for(int i = 0; i < task_num; i++)
        task_list[i].born_domain = index[task_list[i].born_node];


    for(int i = 0; i < node_num; i++)
    {
        node_list[i].domain_num = index[i];
        node_list[i].current_task_num = 0;
        for(int j = 0; j < task_num; j++)
            if(task_list[j].born_node == i)
            {
                node_list[i].current_task[node_list[i].current_task_num] = j;
                node_list[i].current_task_num ++;
            }
    }

    for(int i = 0; i < domain_num; i++)//初始化域间路径拓扑
        for(int j = 0; j < domain_num; j++)
            domain_graph[i][j] = node_graph[domain_list[i].controller_id][domain_list[j].controller_id];
}
void centralized_init()
{
    domain_num = 1;

    domain_list[0].id = 0;
    domain_list[0].member_num = node_num;
    domain_list[0].remain = 0;
    for(int i = 0; i < node_num; i++)
        domain_list[0].member_id[i] = i;    


    
    for(int i = 0; i < task_num; i++)//初始化任务
        task_list[i].born_domain = 0;
    
    for(int i = 0; i < node_num; i++)//初始化节点
        node_list[i].domain_num = 0;
    
    for(int i = 0; i < node_num; i++)
    {
        for(int j = 0; j < task_num; j++)
            if(task_list[j].born_node == i)
            {
                node_list[i].current_task[node_list[i].current_task_num] = j;
                node_list[i].current_task_num ++;
            }
    }
}
/*void domain_graph_generation()//生成域邻接矩阵
{
    for(int i = 0; i < domain_num; i++)
        for(int j = 0; j < domain_num; j++)
            domain_graph[i][j] = node_graph[domain_list[i].controller_id][domain_list[j].controller_id];
}*/
float weight_compute(Domain start_domain, Domain destination, Task task)//计算从一个节点传输至另一个节点的任务重量
{
    float result = 0;
    float delay;
    if(start_domain.id == destination.id)
        delay = task.delay - destination.max_delay;
    else
        delay = task.delay - destination.max_delay - start_domain.max_delay - domain_graph[start_domain.id][destination.id];
    result = task.data_size / delay;
    if(delay <= 0)
        result = 10000000;
    return result;
}
void population_factory()//生成种群，种群个体为贪心算法所得
{
    for(int i = 0; i < Population_size; i++)
    {
        int task_order[6500];//选取物品的顺序
        for(int j = 0; j < task_num; j++)//初始化顺序数组
            task_order[j] = j;
        for(int j = (task_num - 1); j > 0; j--)//洗牌算法洗混排序
        {
            int pick_num = 0;
            if(j != 1)
                pick_num = rand()%(j - 1);
            int temp_num = task_order[j];
            task_order[j] = task_order[pick_num];
            task_order[pick_num] = temp_num;
        }
        int domain_order[200];//选取域的顺序
        for(int j = 0; j < domain_num; j++)//初始化顺序数组
            domain_order[j] = j;
        for(int j = (domain_num - 1); j > 0; j--)//洗牌算法洗混排序
        {
            int pick_num = 0;
            if(j != 1)
                pick_num = rand()%(j - 1);
            int temp_num = domain_order[j];
            domain_order[j] = domain_order[pick_num];
            domain_order[pick_num] = temp_num;
        }
        for(int j = 0; j < task_num; j++)//按顺序将物品装入背包
        {
            for(int k = 0; k < domain_num; k++)
            {
                int task_domain = task_list[task_order[j]].born_domain;
                float weight = weight_compute(domain_list[task_domain], domain_list[domain_order[k]], task_list[task_order[j]]);
                if(weight <= domain_list[domain_order[k]].remain && weight < domain_list[domain_order[k]].min_capacity)
                {
                    domain_list[domain_order[k]].remain -= weight;
                    population[i][task_order[j]] = domain_order[k] + 1;
                    j++;
                    //cout << "task " << order[j] <<" transmit to domain " << k + 1 << endl;
                    if(j >= task_num)
                        break;
                }
            }
        }
        for(int j = 0; j < domain_num; j++)
            domain_list[j].remain = domain_list[j].capacity;
    }       
}

float domain_weight_compute(Node start_node, Node destination, Task task)//域内计算从一个节点传输至另一个节点的任务重量
{
    float result = 0;
    float delay = task.delay;
    if(start_node.id != destination.id)
        delay = task.delay - node_graph[start_node.id][destination.id];
    result = task.data_size / delay;
    if(delay <= 0)
        result = 100000;
    return result;
}

void domain_population_factory(Domain d, Task T_list[6000], int T_list_num)//每个域生成种群，种群个体为贪心算法所得
{
    for(int i = 0;i < Population_size; i++)
        for(int j = 0; j < T_list_num; j++)
            population[i][j] = 0;

    for(int i = 0; i < Population_size; i++)
    {
        int task_order[6500];//选取物品的顺序
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
        int node_order[200];//选取背包的顺序
        for(int j = 0; j < d.member_num; j++)//初始化顺序数组
            node_order[j] = d.member_id[j];
        for(int j = (d.member_num - 1); j > 0; j--)//洗牌算法洗混排序
        {
            int pick_num = 0;
            if(j != 1)
                pick_num = rand()%(j - 1);
            int temp_num = node_order[j];
            node_order[j] = node_order[pick_num];
            node_order[pick_num] = temp_num;
        }
        //for(int j = 0; j < task_num; j++)//输出顺序数组
            //cout << order[j] << " ";
        //cout << endl;
        for(int j = 0; j < T_list_num; j++)//按顺序将物品装入背包
        {
            for(int k = 0; k < d.member_num; k++)
            {
                Node start_node = node_list[T_list[task_order[j]].born_node];
                Node destination = node_list[node_order[k]];
                float weight = domain_weight_compute(start_node, destination, T_list[task_order[j]]);
                if(weight <= node_list[node_order[k]].remain)
                {
                    node_list[node_order[k]].remain -= weight;
                    population[i][task_order[j]] = node_order[k] + 1;
                    break;
                }
            }
            for(int j = (d.member_num - 1); j > 0; j--)//洗牌算法洗混排序
            {
                int pick_num = 0;
                if(j != 1)
                    pick_num = rand()%(j - 1);
                int temp_num = node_order[j];
                node_order[j] = node_order[pick_num];
                node_order[pick_num] = temp_num;
            }
        }
        for(int j = 0; j < d.member_num; j++)
            node_list[d.member_id[j]].remain = node_list[d.member_id[j]].capacity;
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
void selection()//将父种群和子种群进行筛选
{
    float value_fitness[200];
    float balance_fitness[200];
    for(int i = 0; i < 2 * Population_size; i++)//计算种群适应值
    {
        int flag = 1;
        float bucket[200];
        for(int j = 0; j < domain_num; j++)
            bucket[j] = 0;
        for(int j = 0; j < task_num; j++)//判断是否为可行解
        {
            if(population[i][j] != 0)
            {
                Domain start = domain_list[task_list[j].born_domain];
                Domain end = domain_list[population[i][j] - 1];
                float weight = weight_compute(start, end, task_list[j]);
                if(weight > end.min_capacity)
                {
                    flag = 0;
                    break;
                }
                bucket[population[i][j] - 1] += weight;
            }
        }
        for(int j = 0; j < domain_num; j++)
            if(bucket[j] > domain_list[j].capacity)
                flag = 0;
        if(flag == 1)//计算基因适应值
        {
            float value_sum = 0;
            float max_percentage = 0;
            float total_percentage = 0;
            float total_weight = 0;
            float weight_bucket[200];
            for(int j = 0; j < task_num; j++)
            {
                if(population[i][j] != 0)
                {
                    Domain start = domain_list[task_list[j].born_domain];
                    Domain end = domain_list[population[i][j] - 1];
                    float weight = weight_compute(start, end, task_list[j]);
                    if(weight != 0)
                    {
                        value_sum += 1/weight;
                    }
                    else
                        value_sum = 0;
                    total_weight += weight;
                    weight_bucket[population[i][j] - 1] += weight;
                }
            }

            for(int j = 0; j < domain_num; j++)
            {
                total_percentage += weight_bucket[j] / domain_list[j].capacity;
                if(max_percentage < weight_bucket[j] / domain_list[j].capacity)
                    max_percentage  = weight_bucket[j] / domain_list[j].capacity;
            }
                
            value_fitness[i] = value_sum;
            balance_fitness[i] = total_percentage / (max_percentage * domain_num);
        }
        else
        {
            value_fitness[i] = 0;
            balance_fitness[i] = 0;
        }

    }
    float fitness_max = 0;
    for(int i = 0; i < 2 * Population_size; i++)
    {
        if(fitness_max < balance_fitness[i])
            fitness_max = balance_fitness[i];
    }
    balance_fitness_max = fitness_max;

    int kill[6500];
    for(int i = 0; i < 2 * Population_size; i++)
        kill[i] = 0;
        /*
    cout << endl;
    cout << "before: " << endl;
    cout<< "value: ";
    for(int i = 0; i < 2*Population_size; i++)
        cout  << value_fitness[i] << " ";
    cout << endl;
    cout<< "balance: ";
    for(int i = 0; i < 2*Population_size; i++)
        cout << balance_fitness[i] << " ";
    cout << endl;
    */
    for(int i = 0; i < Population_size / 2; i++)//挑选种群中价值较大的个体
    {
        float min = 999999;
        int min_num = 0;
        for(int j = 0; j < 2 * Population_size; j++)
        {
            if(min > value_fitness[j] && kill[j] == 0)
            {
                min = value_fitness[j];
                min_num = j;
            }
        }
        kill[min_num] = 1;
    }
    for(int i = 0; i < Population_size / 2; i++)//挑选种群中较为平衡的个体
    {
        float min = 999999;
        int min_num = 0;
        for(int j = 0; j < 2 * Population_size; j++)
        {
            if(min >= balance_fitness[j] && kill[j] == 0)
            {
                min = balance_fitness[j];
                min_num = j;
            }
        }
        kill[min_num] = 1;
    }
    int cc = 0;
    for(int i = 0; i < 2 * Population_size; i++)
    {
        if(kill[i] == 0)
        {
            for(int j = 0; j < task_num; j++)
            {
                live[cc][j] = population[i][j];
            }
            cc++;
        }
    }
    /*
    cout << endl;
    cout << "after: " << endl;
    for(int i = 0; i < 2 * Population_size; i++)
    {
        if(kill[i] == 0)
        {
            cout << "value:" << value_fitness[i] << " ";
            cout << "balance:"<< balance_fitness[i] << " ";
        }
    }*/



    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < task_num; j++)
            population[i][j] = live[i][j];

}
void domain_selection(Domain d, Task T_list[6500], int T_list_len)//将父种群和子种群进行筛选
{
    clock_t start, end;
    double dur;
    start = clock();
    //cout <<"T_list_len"<< T_list_len << endl;
    float value_fitness[200];
    for(int i = 0; i < 2 * Population_size; i++)//计算种群适应值
    {
        int flag = 1;
        float bucket[200];
        for(int j = 0; j < node_num; j++)
            bucket[j] = 0;
            end = clock();
    dur = (double)(end - start) * 1000;
    //cout << "   time0.5: " << dur/CLOCKS_PER_SEC;
        for(int j = 0; j < T_list_len; j++)//判断是否为可行解
        {
            if(population[i][j] != 0)
            {
                Node start = node_list[T_list[j].born_node];
                Node end = node_list[population[i][j] - 1];
                float weight = domain_weight_compute(start, end, T_list[j]);
                bucket[population[i][j] - 1] += weight;
            }
        }
        end = clock();
    dur = (double)(end - start) * 1000;
    //cout << "   time1: " << dur/CLOCKS_PER_SEC;
        for(int j = 0; j < d.member_num; j++)
            if(bucket[d.member_id[j]] > node_list[d.member_id[j]].capacity)
                flag = 0;
        if(flag == 1)//计算基因适应值
        {
            float value_sum = 0;
            float total_weight = 0;
            float weight_bucket[200];
            for(int j = 0; j < node_num; j++)
                weight_bucket[j] = 0;
            for(int j = 0; j < T_list_len; j++)
            {
                if(population[i][j] != 0)
                {
                    Node start = node_list[T_list[j].born_node];
                    Node end = node_list[population[i][j] - 1];
                    float weight = domain_weight_compute(start, end, T_list[j]);
                    weight_bucket[population[i][j] - 1] += weight;
                    if(weight != 0)
                    {
                        value_sum += 1/weight;
                    }
                    else
                        value_sum = 0;
                    total_weight += weight;
                }
            }                
            value_fitness[i] = value_sum;
        }
        else
            value_fitness[i] = 0;
        end = clock();
    dur = (double)(end - start) * 1000;
    //cout << "   time2: " << dur/CLOCKS_PER_SEC;
    }
    end = clock();
    dur = (double)(end - start) * 1000;
    //cout << "   total_time: " << dur/CLOCKS_PER_SEC;
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
void duplicate(int gen_len)//将种群复制一份到矩阵下面
{
    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < gen_len; j++)
           population[i + Population_size][j] = population[i][j];
}
void print_population(int T_num)//输出种群
{
    for(int i = 0; i < Population_size; i++)
    {
        for(int j = 0; j < T_num; j++)
        {
            cout << population[i][j] << " ";
        }        
        cout << endl;
    }
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
    int count = 0;
    //int c = 10000;
    Task *Tlist = (Task*) malloc(sizeof(Task) * 6000);
    int round = 0;
    int index[1000];
    while(count < task_num && round < node_num)
    {
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
                c = knapsack(node_list[i], Tlist, node_list[i].current_task_num, index);
            }
                
            count += c;
            for(int j = 0; j < node_list[i].current_task_num; j++)//清空节点的current task
                node_list[i].current_task[j] = 0;
            node_list[i].current_task_num = 0;
        }
        //cout << "ok!";
        if (count >= task_num)
            break;
        //cout << "  2.2 ";
        for(int i = 0; i < task_num; i++)//向其他节点传输
        {
            if(task_list[i].done == 0)
            {
                int current = task_list[i].current_node;
                float min = 1000;
                int des = -1;
                for(int j = 0; j < node_num; j++)
                {
                    if(min > node_graph[current][j] && task_list[i].route_bucket[j] != 1 && task_list[i].left_time > node_graph[current][j] && current != j)
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
    //cout << "aa";
    
    std::vector<vector<float>> domain_input0;//读入domain_input
    ifstream fp3(o_route); //定义声明一个ifstream对象，指定文件路径
    string line3;
    //getline(fp,line); //跳过列名，第一行不做处理
    while (getline(fp3,line3))
    { //循环读取每行数据
        vector<float> data_line;
        string number;
        istringstream readstr(line3); //string数据流化
        //将一行数据按'，'分割
        for(int j = 0;j < 110;j++)
        { //可根据数据的实际情况取循环获取
            getline(readstr,number,','); //循环读取数据
            data_line.push_back(atof(number.c_str())); //字符串传float
        }
        domain_input0.push_back(data_line); //插入到vector中
    }
    
    //cout << "bb";
    //float work[200010][10];
    float **work=(float ** )malloc(sizeof(float *)*(work0[0][0] + 1)); 
    for(int i = 0;i <= work0[0][0];++i) 
        work[i] = (float*)malloc(sizeof(float)*10);
    
    //cout << work0[0][0];

    for(int i = 0; i <= work0[0][0];++i)
        for(int j = 0; j < 4;++j)
            work[i][j] = work0[i][j];
    
    //cout << "dd";
    float domain_input[100][100];
    for(int i = 0; i <= domain_input0[0][0];i++)
        for(int j = 0; j < 15;j++)
            domain_input[i][j] = domain_input0[i][j];
    //cout << "ee";
   

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
    init(work, domain_input, route);
    centralized_init();
    clock_t start, end;
    double dur;
    double div_time;
    //div_time = domainDiv();
    
    ofstream ou(rd_route);
    //cout << "ss";
    int count = 0;
    int t_flag = 0;
    int done_flag = 0;
    int jump_ms = 100;
    int total_dead = 0;


    
    Task *Tlist = (Task*) malloc(sizeof(Task) * 200);
    
    int **population_copy=NULL;//复制population数组
    int ii; 
    
    population_copy=(int ** )malloc(sizeof(int *)*10); 
    for(ii = 0;ii < 20;ii++)
    {
        
        population_copy[ii] = (int*)malloc(sizeof(int)*100);
    } 
        
    
    
    while(done_flag == 0 && count < 10000)
    {
        cout << "gooood";
        //cout << endl;
        //cout << "1" << " ";
        //div_time = domainDiv() * 1000;//kmeans
         cout << "good";
        //cout << div_time <<" ";
        kmeans_t = div_time;
        //cout << "kmeans_t:"<< kmeans_t << " ";
        system_t += kmeans_t;
        done_flag = 1;
        

        int dead = task_update();
        
        update(work, domain_input, route);
        
        for(int i = 0; i < total_task_num; i++)
        {
            if(total_task_list[i].finished == 0)
            {
                done_flag = 0;
                break;
            }
        }
        //if(done_flag == 1)
            //continue;
        if(task_num == 0)
        {
            system_t += jump_ms;
            if(total_task_list[total_task_num].done == 1)
                t_flag = 1;
        }
            
        //distribute




        //cout << "2" << " ";
        int gene_class[150];//我们的方案
        for(int i = 0; i < domain_num; i++)
            gene_class[i] = i;
        start = clock();
        population_factory();//初始化种群
        end = clock();
        dur = (double)(end - start) * 1000;
        our_t += dur/CLOCKS_PER_SEC;
            
        for(int i = 0; i < iteration_num; i++)//进行遗传
        {  
        start = clock();
        duplicate(task_num);
        crossover(task_num);
        mutation(task_num, gene_class, node_num);
        selection();
        end = clock();
        dur = (double)(end - start) * 1000;
        our_t += dur/CLOCKS_PER_SEC;
        //max_fitness_num++;

        }


    //cout << "3" << " ";
    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < task_num; j++)
            population_copy[i][j] = population[i][j];
    
    int Tlist_num;
    int index[10010];//索引，Tlist中序号->task_list中序号
    for(int i = 1; i <= domain_num; i++)//域内遗传算法
    {
        Domain d = domain_list[i - 1];
        Tlist_num = 0;
        for(int j = 0; j < task_num; j++)
        {
            if(population[0][j] == i)//进行域数据的更新和写入Tlist
            {
                int weight = weight_compute(domain_list[task_list[j].born_domain],domain_list[i - 1], task_list[j]);
                //domain_list[i - 1].remain -= weight;
                Tlist[Tlist_num] = task_list[j];
                index[Tlist_num] = j;
                Tlist_num++;
            } 
        }
        //cout << "4" << " ";
        if(Tlist_num != 0)
        {
        start = clock();      
        domain_population_factory(d,Tlist,Tlist_num);//新建域内种群
        end = clock();
        dur = (double)(end - start) * 1000;
        d.compute_time += dur/CLOCKS_PER_SEC;
        for(int j = 0; j < iteration_num; j++)//域内遗传算法
        {
            start = clock();
            duplicate(Tlist_num);
            end = clock();
            dur = (double)(end - start) * 1000;
            d.duplicate_time += dur/CLOCKS_PER_SEC;
            double a = dur/CLOCKS_PER_SEC;

            start = clock();
            crossover(Tlist_num);
            end = clock();
            dur = (double)(end - start) * 1000;
            d.crossover_time += dur/CLOCKS_PER_SEC;
            double b = dur/CLOCKS_PER_SEC;

            start = clock();
            mutation(Tlist_num, d.member_id, d.member_num);
            end = clock();
            dur = (double)(end - start) * 1000;
            d.mutation_time += dur/CLOCKS_PER_SEC;
            double c = dur/CLOCKS_PER_SEC;

            start = clock();
            domain_selection(d,Tlist,Tlist_num);
            end = clock();
            dur = (double)(end - start) * 1000;
            d.selection_time += dur/CLOCKS_PER_SEC;
            double dd = dur/CLOCKS_PER_SEC;
            //print_population(Tlist_num);
            // dur = (double)(end - start) * 1000;
            d.compute_time = d.compute_time + a + b + c + dd;
            //cout << "iteration " << j << " compute time: " << d.compute_time << " duplicate_time: " << a <<" crossover time: " << b <<" mutation time: " << c <<" selection time: " << dd <<" ";
            //cout << endl;
            
        }
        //cout << "5" << " ";
        for(int j = 0; j < Tlist_num; j++)
        {
            //cout << population[0][j] << " ";
        }
        for(int j = 0; j < Tlist_num; j++)//进行节点和任务数据的更新
        {
            if(population[0][j] != 0)
            {
                float weight = domain_weight_compute(node_list[Tlist[j].born_node], node_list[population[0][j] - 1], Tlist[j]);
                //cout <<"weight: " <<weight << " ";
                node_list[population[0][j] - 1].remain -= weight;
                task_list[index[j]].weight = weight;
                task_list[index[j]].done = 1;
                task_list[index[j]].end_node = population[0][j] - 1;
                if(weight != 0)
                    total_value += 1/weight;
                
            }
        }
        domain_list[i - 1].compute_time = d.compute_time;
        for(int i = 0; i < Population_size; i++)//还原population数组
            for(int j = 0; j < task_num; j++)
                population[i][j] = population_copy[i][j];
        }
    }
    
    //cout << "6" << endl;
    float t_max = 0;
    //cout << "total_compute time: ";
    for(int i = 0; i < domain_num; i++)
    {
        //cout << " t: "<<domain_list[i].compute_time << " ";
        //cout << domain_list[i].duplicate_time << " ";
        //cout << domain_list[i].crossover_time << " ";
        //cout << domain_list[i].mutation_time << " ";
        //cout << domain_list[i].selection_time << " ";
        if(t_max < domain_list[i].compute_time)
            t_max = domain_list[i].compute_time;
    }
    our_t += t_max;
    schedule_t = our_t;

        for(int i = 0; i < task_num; i++)//将task_num信息同步到总表上
            total_task_list[task_list[i].id] = task_list[i];
        for(int i = 0; i < node_num;i++)
            //cout << node_list[i].remain << " ";
        for(int i = 0; i < domain_num;i++)
        {
            domain_list[i].remain = 0;
            for(int j = 0; j < node_num;j++)
            {
                if(node_list[j].domain_num == i)
                {
                    domain_list[i].remain += node_list[j].remain;
                }
            }
        }
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

     //cout <<" count: " << count;//当前时隙id 0
    //cout << " time: "<< schedule_t + kmeans_t <<" ";//当前时隙长短1
    //cout << " task_num: " << task_num;//当前时隙需要处理任务量2
    //cout <<" failed task num: " << dead<< " ";//本时隙超时任务数3

    cout << count << "," << schedule_t + kmeans_t << "," << finish_task_num << "," <<  dead << endl;

    //cout << " total_value: " << total_value << " ";
    for(int i = 0; i < current_task_id; i++)
        if(total_task_list[i].failed == 1)
            total_dead ++;
    //cout << " current_dead:" << total_dead << " ";
    //cout << " current_task_id: " << current_task_id << " ";
    //cout << " time: " << system_t;

    float mmax = 0;
    float msum = 0;
    for(int i = 0; i < domain_num; i++)
    {
        if(domain_list[i].remain / domain_list[i].capacity > mmax)
        {
            mmax = domain_list[i].remain / domain_list[i].capacity;
        }
        msum += domain_list[i].remain / domain_list[i].capacity;
    }

        for(int i = 0; i < domain_num; i++)
        {
            cout << domain_list[i].remain / domain_list[i].capacity;
            if(i != domain_num - 1)
                cout << ",";
        }
        

    cout << endl;
    cout << total_value << "," << total_dead << "," << current_task_id << "," << system_t << endl << endl;
    schedule_t = 0;
    kmeans_t = 0;
    our_t = 0;
    count ++; 
    total_dead = 0;
    finish_task_num = 0;
    
    //cout << task_num << endl;    
    if(system_t > 60000)
        break;
    }
    
    



/*
   
    while(done_flag == 0 && count < 10000)//集中式
    {
        //cout << endl;
        //cout << "1" << " ";
        done_flag = 1;
        int dead = task_update();
        update(work, domain_input, route);
        for(int i = 0; i < total_task_num; i++)
        {
            if(total_task_list[i].finished == 0)
            {
                done_flag = 0;
                break;
            }
        }
        //if(done_flag == 1)
            //continue;
        if(task_num == 0)
        {
            system_t += jump_ms;
            if(total_task_list[total_task_num].done == 1)
                t_flag = 1;
        }
        //cout << "2" << " ";

    //cout << "3" << " ";
    for(int i = 0; i < Population_size; i++)
        for(int j = 0; j < task_num; j++)
            population_copy[i][j] = population[i][j];
        Domain d = domain_list[0];
        //cout << "4" << " ";
        int gene_class[150];
        for(int i = 0; i < node_num; i++)
            gene_class[i] = i;
        if(task_num != 0)
        {
            start = clock();      
            domain_population_factory(d,task_list,task_num);//新建域内种群
            end = clock();
            dur = (double)(end - start) * 1000;
            d.compute_time += dur/CLOCKS_PER_SEC;
            for(int j = 0; j < iteration_num; j++)//域内遗传算法
            {
                start = clock();
                duplicate(task_num);
                crossover(task_num);
                mutation(task_num, gene_class, node_num);
                domain_selection(d,task_list,task_num);
                end = clock();
            //print_population(Tlist_num);
                dur = (double)(end - start) * 1000;
                d.compute_time += dur/CLOCKS_PER_SEC;
            //cout << endl;
            
            }
            //cout << "5" << " ";
        for(int j = 0; j < task_num; j++)//进行节点和任务数据的更新
        {
            if(population[0][j] != 0)
            {
                float weight = domain_weight_compute(node_list[task_list[j].born_node], node_list[population[0][j] - 1], task_list[j]);
                //cout <<"weight: " <<weight << " ";
                node_list[population[0][j] - 1].remain -= weight;
                task_list[j].weight = weight;
                task_list[j].done = 1;
                task_list[j].end_node = population[0][j] - 1;
                if(weight != 0)
                    total_value += 1/weight;
                
            }
        }
        domain_list[0].compute_time = d.compute_time;
        for(int i = 0; i < Population_size; i++)//还原population数组
            for(int j = 0; j < task_num; j++)
                population[i][j] = population_copy[i][j];
        }
    
    
    //cout << "6" << " ";
    float t_max = domain_list[0].compute_time;
    our_t += t_max;
    schedule_t = our_t;



        for(int i = 0; i < task_num; i++)//将task_num信息同步到总表上
            total_task_list[task_list[i].id] = task_list[i];
        //cout << endl;
        for(int i = 0; i < node_num;i++)
            //cout << node_list[i].remain << " ";
        //更改node csv文件
        
        
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
        count ++; 
    //cout << "7" << endl;
    //cout <<" count: " << count;//当前时隙id 0
    //cout << " time: "<< schedule_t + kmeans_t <<" ";//当前时隙长短1
    //cout << " task_num: " << task_num;//当前时隙需要处理任务量2
    //cout <<" failed task num: " << dead<< " ";//本时隙超时任务数3

    cout << count << "," << schedule_t + kmeans_t << "," << finish_task_num << "," <<  dead << endl;
    finish_task_num = 0;
    //cout << " total_value: " << total_value << " ";
    for(int i = 0; i < current_task_id; i++)
        if(total_task_list[i].failed == 1)
            total_dead ++;
    //cout << " current_dead:" << total_dead << " ";
    //cout << " current_task_id: " << current_task_id << " ";
    //cout << " time: " << system_t;

    cout << total_value << "," << total_dead << "," << current_task_id << "," << system_t << endl << endl;
    schedule_t = 0;
    kmeans_t = 0;
    our_t = 0;
    total_dead = 0;
    if(system_t > 60000)
        break;
    
    
   

    //cout << task_num << endl;    
    }
    

    //cout << "total_dead:" << dead << " ";

    //cout << "total time:"<< system_t << endl;
    //cout << "good!" << endl;
    //cout<< total_task_list[total_task_num].born_time;
    //cout << count;
    
   */



    


    



    

        /*
        jump_ms = 1;
        //system_t += 100;
        while(done_flag == 0 && count < 1000000)
        {
            
        //cout <<endl <<  "1" << " ";
        done_flag = 1;
        int dead = task_update();
        
        update(work, domain_input, route);
        

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
        }
            
        //distribute
        //cout << "2" << " ";

        start = clock();//分布式
        distributed();
        end = clock();
        dur = (double)(end - start) * 1000;
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
    schedule_t = 0;
    kmeans_t = 0;
    our_t = 0;

    count ++; 

    //cout <<" count: " << count;//当前时隙id 0
    //cout << " time: "<< schedule_t + kmeans_t <<" ";//当前时隙长短1
    //cout << " task_num: " << task_num;//当前时隙需要处理任务量2
    //cout <<" failed task num: " << dead<< " ";//本时隙超时任务数3

    cout << count << "," << schedule_t + kmeans_t << "," << finish_task_num << "," <<  dead << endl;
    finish_task_num = 0;
    //cout << " total_value: " << total_value << " ";
    for(int i = 0; i < current_task_id; i++)
        if(total_task_list[i].failed == 1)
            total_dead ++;
    //cout << " current_dead:" << total_dead << " ";
    //cout << " current_task_id: " << current_task_id << " ";
    //cout << " time: " << system_t;

    cout << total_value << "," << total_dead << "," << current_task_id << "," << system_t << endl << endl;
    total_dead = 0;
    
    //cout << task_num << endl; 
    if(system_t > 60000)
        break;
    }
    */
    




    
    //cout << " total_dead:" << total_dead << " ";
    if(t_flag == 1)
        system_t -= jump_ms;
    //cout << " total time:"<< system_t << endl;
    //cout << "good!" << endl;
    //free(work);
    //free(population_copy);
    //free(route);
    //cout<< total_task_list[total_task_num].born_time;
    //cout << count;   
    

}



/*
    start = clock();//分布式
    distributed();
    end = clock();
    dur = (double)(end - start) * 1000;
    distributed_t += dur/CLOCKS_PER_SEC;
    
    cout << "distributed_t:" << distributed_t << endl;
    int dead = 0;
    for(int i = 0; i < task_num; i++)
        if(task_list[i].done == 0)
            dead++;
    cout << "dead:" << dead;
    */

/*
    centralized_init();
    start = clock();//集中式
    int dead = 0;    
    //int population_copy[100][6500];
    int **population_copy=NULL;
    int i; 

    start = clock();      
    domain_population_factory(domain_list[0],task_list,task_num);
    end = clock();
    dur = (double)(end - start) * 1000;
    centralized_t += dur/CLOCKS_PER_SEC;
    int gene_class[150];
    for(int i = 0; i < node_num; i++)
        gene_class[i] = i;
    for(int i = 0; i < iteration_num; i++)
    {
    //cout << "domain" << " "<< i - 1 << " " << j + 1 <<" generation" << endl;
        start = clock();
        duplicate(task_num);
        crossover(task_num);
        mutation(task_num, gene_class, node_num);
        domain_selection(domain_list[0],task_list,task_num);
        end = clock();
        //print_population(Tlist_num);
        dur = (double)(end - start) * 1000;
        //cout << endl;
    }
    centralized_t += dur/CLOCKS_PER_SEC;
    for(int i = 0; i < task_num; i++)
    {
        if(population[0][i] != 0)
        {
            task_list[i].done = 1;
            task_list[i].end_node = population[0][i] - 1;
            int weight = domain_weight_compute(node_list[task_list[i].born_node],node_list[population[0][i] - 1],task_list[i]);
            node_list[population[0][i] - 1].remain -= weight;

        }
    }
    


    for(int i = 0; i < task_num; i++)
        if(population[0][i] == 0)
        {
            task_list[i].done == 0;
            dead++;
        }
    cout << "time: " << centralized_t << endl;
    cout << "dead: " << dead << endl;
    cout << "goooooood!";
    */

double domainDiv() 
{
    clock_t start, finish;
    double Total_time;
    
    loadCsvData();
    start = clock();
    
    kmeans();
    
    getController();
    
    finish = clock();
    storeCsvData();
    
    Total_time = (double)(finish - start) / CLOCKS_PER_SEC;
    return Total_time;
    /*
    int n[3] = { 30, 50, 100 };
    int i, j;

    for (int k1 = 60; k1 <= 60; k1++) {
        for (int k2 = 50; k2 <= 50; k2++) {
            string folderPath = "D:/NetDataOut/" + to_string(k1) + "_" + to_string(k2);
            if(0 != _access(folderPath.c_str(), 0)) _mkdir(folderPath.c_str());  // 创建输出文件夹
            for (i = 0; i < 3; i++) {
                for (j = 0; j < 10; j++) {
                    K1 = k1 / 100.0;
                    K2 = k2 / 100.0;

                    cout << K1 << '_' << K2 << " ";
                    cout << i << " " << j << "\n";
                    d_route = "D:/NetData/cov_data_" + to_string(n[i]) + "_v" + to_string(j) + ".csv"; //映射数据文件
                    rd_route = "D:/NetData/data_" + to_string(n[i]) + "_v" + to_string(j) + ".csv"; //数据文件
                    o_route = folderPath + "/out_" + to_string(n[i]) + "_v" + to_string(j) + ".csv"; //数据文件

                    loadCsvData();

                    kmeans();
                    getController();

                    storeCsvData();
                }
            }
        }
    }
    */
}

void kmeans()
{
    int i, j, count = 0;
    float temp1, temp2;
    
    incrementalGreedy();
    temp1 = getDifference();  //第一次中心点和所属数据点的距离之和
    count++;
    //printf("The difference between data and center is: %.2f\n\n", temp1);

    getCenter(in_cluster);
    cluster();  //用新的k个中心点进行第二次聚类
    temp2 = getDifference();
    count++;
    //printf("The difference between data and center is: %.2f\n\n", temp2);

    while (fabs(temp2 - temp1) != 0) {   //比较前后两次迭代，若不相等继续迭代
        temp1 = temp2;
        getCenter(in_cluster);
        cluster();
        temp2 = getDifference();
        count++;
        //printf("The %dth difference between data and center is: %.2f\n\n", count, temp2);
    }

    //printf("\nThe total number of iterations is: %d\n", count);  //统计迭代次数
}

//计算欧几里得距离
float getDistance(float avector[], float bvector[], int n)
{
    int i;
    float sum = 0.0;
    for (i = 0; i < n; i++)
        sum += pow(avector[i] - bvector[i], 2);
    // sum += (N * pow(bvector[i] - avector[i], 2));
    return sqrt(sum);
}

//把N个数据点聚类，标出每个点属于哪个聚类
void cluster()
{
    int i, j;
    float min;
    float** distance = createarray(N, K);  //存放每个数据点到每个中心点的距离
    //float distance[N][K];  //也可使用C99变长数组
    for (i = 0; i < N; ++i) {
        min = 9999.0;
        for (j = 0; j < K; ++j) {
            distance[i][j] = getDistance(net_data[i], cluster_center[j], D);
            //printf("%f\n", distance[i][j]);
            if (distance[i][j] < min) {
                min = distance[i][j];
                in_cluster[i] = j;
            }
        }
        //printf("data[%d] in cluster-%d\n", i, in_cluster[i] + 1);
    }
    //printf("-----------------------------\n");
    freearray(distance);
}

//计算所有聚类的中心点与其数据点的距离之和
float getDifference()
{
    int i, j;
    float sum = 0.0;
    for (i = 0; i < K; ++i) {
        for (j = 0; j < N; ++j) {
            if (i == in_cluster[j]) {//这里的i不是路由器的编号QAQ
                sum += getDistance(net_data[j], cluster_center[i], D);
            }
        }
    }
    return sum;
}

//计算每个聚类的中心点
void getCenter(int in_cluster[])
{
    float** sum = createarray(K, D);  //存放每个聚类中心点
    //float sum[K][D];  //也可使用C99变长数组
    int i, j, q, count;
    for (i = 0; i < K; i++)
        for (j = 0; j < D; j++)
            sum[i][j] = 0.0;
    for (i = 0; i < K; i++) {
        count = 0;  //统计属于某个聚类内的所有数据点
        for (j = 0; j < N; j++) {
            if (i == in_cluster[j]) {
                for (q = 0; q < D; q++)
                    sum[i][q] += net_data[j][q];  //计算所属聚类的所有数据点的相应维数之和
                count++;
            }
        }
        for (q = 0; q < D; q++)
            cluster_center[i][q] = sum[i][q] / count;
    }
    /*
    printf("The new center of cluster is:\n");
    for (i = 0; i < K; i++)
        for (q = 0; q < D; q++) {
            printf("%-8.2f", cluster_center[i][q]);
            if ((q + 1) % D == 0)    putchar('\n');
        }
    */
    freearray(sum);
}

void incrementalGreedy()
{
    K = 0;
    set<int>::iterator it1, it2;
    set<int> R, C;
    int i, j;
    int best_controller;
    int* is_center = (int*)malloc(N * sizeof(int));//存放第i个中心点的路由器编号

    for (i = 0; i < N; i++) {
        R.insert(i);
        C.insert(i);
    }
    while (!R.empty()) {
        set<int> max_s, cur_s;
        best_controller = -1;
        for (it1 = C.begin(); it1 != C.end(); it1++) {
            cur_s.clear();
            //将所有符合时延的点全加入Sj，衡量标准未定之前先用75约束交换机的加入
            for (it2 = R.begin(); it2 != R.end(); it2++) {
                float commnication_time = net_data[*it2][*it1];
                float cp_difference = cp_diff[*it2][*it1];
                if (commnication_time < T1 && cp_difference < T2) {
                    cur_s.insert(*it2);
                }
            }
            if (cur_s.size() > max_s.size()) {
                max_s = cur_s;
                best_controller = *it1;
                //printf("the current best controller is: %d\n", best_controller);
            }
        }
        //printf("the final best controller is: %d\n", best_controller);
        is_center[K] = best_controller;
        C.erase(best_controller);

        for (it1 = max_s.begin(); it1 != max_s.end(); it1++) {
            in_cluster[*it1] = K;
            R.erase(*it1);
        }
        K++;
    }

    //printf("The number of cluster is: %d\n", K);
    cluster_center = createarray(K, D);
    for (i = 0; i < K; ++i) {
        int curcenter = is_center[i];
        copy(net_data[curcenter], net_data[curcenter] + D, cluster_center[i]);
    }

    free(is_center);
}

// 域的划分完成后获取每个域的控制节点
void getController()
{
    
    int i, j, temp_K = K;
    float min, distance;
    int* controller = (int*)malloc(K * sizeof(int));
    points_list = {};
    
    //cout << K << "\n";
    for (i = 0; i < temp_K; ++i) {
        vector<int> p;
        min = 9999.0;
        for (j = 0; j < N; ++j) {  // 查找每个控制域内的节点
            if (i == in_cluster[j]) {
                
                p.push_back(j);
                //cout << "good";
                distance = getDistance(net_data[j], cluster_center[i], D);
                if (distance < min) {
                    min = distance;
                    controller[i] = j;
                }
            }
        }
        
        if (p.size() == 0) {
            K--;
        }
        else {
            p.push_back(controller[i]);  // 把controller放在队列最后
            points_list.push_back(p);
        }
        //cout << "The " << i << "st controller is " << controller[i] << "\n";
    }
    
    free(controller);
}

void loadCsvData()
{
    int i, j;
    int max_delay = 0;
    string line, field;

    //ifstream fp1(d_route, ios::in), fp2(rd_route, ios::in);
    ifstream fp2(rd_route, ios::in);
    //if (!fp1) { cout << "打开文件失败：" << d_route << "\n"; exit(1); }
    if (!fp2) { cout << "打开文件失败：" << rd_route << "\n"; exit(1); }
    getline(fp2, line);  stringstream ss(line);  getline(ss, field, ',');  D = stof(field);  getline(ss, field, ',');  N = stof(field);
    //getline(fp1, line);  // 给D和N赋值
    
    net_data = createarray(N, D);
    cp_diff = createarray(N, D);
    r_net_data = createarray(N, D);
    r_cp_diff = createarray(N, D);

    in_cluster = (int*)malloc(N * sizeof(int));  //每个数据点所属聚类的标志数组
    memset(in_cluster, -1, N * sizeof(int));

    
    for (i = 0; i < N; i++) {  // 读取N行数据
        getline(fp2, line);  stringstream ss2(line);
        for (j = 0; j < D - 1; j++) {
            getline(ss2, field, ',');  r_net_data[i][j] = stof(field);
            if (r_net_data[i][j] > max_delay) max_delay = r_net_data[i][j];
            //cout << r_net_data[i][j] << " ";
        }
        getline(ss2, field, ',');  r_net_data[i][j] = stof(field);
        //cout << r_net_data[i][j] << "\n";
    }

    T1 = 0;
    T2 = 0;
    for (i = 0; i < N; i++) {
        for (j = 0; j < D - 1; j++) {
            net_data[i][j] = r_net_data[i][j] / max_delay;
            T1 += net_data[i][j];
            //cout << r_net_data[i][j] << " ";
        }
        net_data[i][j] = (r_net_data[i][j] / 1152) * sqrt(N);
        //cout << r_net_data[i][j] << "\n";
    }

    for (i = 0; i < N; i++) {
        for (j = i; j < N; j++) {
            float diff = abs(net_data[i][D - 1] - net_data[j][D - 1]);
            cp_diff[i][j] = diff;
            cp_diff[j][i] = diff;
            T2 += diff;

            diff = abs(r_net_data[i][D - 1] - r_net_data[j][D - 1]);
            r_cp_diff[i][j] = diff;
            r_cp_diff[j][i] = diff;
        }
    }

    T1 /= (N * (N - 1));
    T2 /= (N * (N - 1) / 2);
    T1 *= K1;
    T2 *= K2;

    fp2.close();

}

void storeCsvData()
{
    int i, j;
    float avg_delay = 0.0, avg_cpdiff = 0.0;
    ofstream outFile(o_route, ios::out);  if (!outFile) { cout << "打开文件失败！" << endl; exit(1); }
    outFile << K << endl;
    for (i = 0; i < K; ++i) {
        float max_delay = 0.0, min_cp = 99999.0, max_cp = 0.0;  // 每个域内的最大时延，最大算力，最小算力
        vector<int> points = points_list[i];
        int size = points.size() - 1;
        int cur_controller = points[size];  // 域内节点计数器数量N
        for (j = 0; j < size; ++j) {  // 查找每个控制域内的节点
            int cur_p = points[j];
            float cur_delay = r_net_data[cur_p][cur_controller], cur_cp = r_net_data[cur_p][D - 1];
            if (cur_delay > max_delay) max_delay = cur_delay;
            if (cur_cp < min_cp) min_cp = cur_cp;
            if (cur_cp > max_cp) max_cp = cur_cp;
        }
        avg_delay += max_delay;
        avg_cpdiff += (max_cp - min_cp);
        //cout << max_cp << "," << min_cp << endl;

        outFile << cur_controller << ',' << size << ',' << min_cp << ',' << max_delay;
        for (j = 0; j < size; ++j) outFile << ',' << points[j];
        outFile << endl;  
    }

    avg_delay /= K;
    avg_cpdiff /= K;

    outFile << "\n\n" << K << ',' << avg_cpdiff << ',' << avg_delay << endl;

    outFile.close();

    freearray(net_data);
    freearray(cp_diff);
    freearray(r_net_data);
    freearray(r_cp_diff);
    freearray(cluster_center);
    free(in_cluster);
}

//动态创建二维数组
float** createarray(int m, int n)
{
    int i;
    float** p;
    p = (float**)malloc(m * sizeof(float*));
    p[0] = (float*)malloc(m * n * sizeof(float));
    for (i = 1; i < m; i++)    p[i] = p[i - 1] + n;
    return p;
}

//释放二维数组所占用的内存
void freearray(float** p)
{
    free(*p);
    free(p);
}

/*
//从data.txt导入数据，要求首行格式：D=数据维度,N=数据量
void loadData(int* d, int* n)
{
    int i, j;
    //float** arraydata;
    FILE* fp, * fp1;
    if ((fp = fopen(d_route.c_str(), "r")) == NULL)    fprintf(stderr, "cannot open data.txt!\n");
    if (fscanf(fp, "D=%d,N=%d\n", d, n) != 2)        fprintf(stderr, "load error!\n");

    if ((fp1 = fopen(rd_route.c_str(), "r")) == NULL)    fprintf(stderr, "cannot open data.txt!\n");
    if (fscanf(fp1, "D=%d,N=%d\n", d, n) != 2)        fprintf(stderr, "load error!\n");

    net_data = createarray(*n, *d);
    cp_diff = createarray(*n, *n);

    r_net_data = createarray(*n, *d);
    r_cp_diff = createarray(*n, *n);

    in_cluster = (int*)malloc(*n * sizeof(int));  //每个数据点所属聚类的标志数组
    memset(in_cluster, -1, *n * sizeof(int));

    T1 = 0;
    T2 = 0;
    for (i = 0; i < *n; i++) {
        for (j = 0; j < *d - 1; j++) {
            fscanf(fp, "%f", &net_data[i][j]);  //读取数据点
            T1 += net_data[i][j];

            fscanf(fp1, "%f", &r_net_data[i][j]);
        }
        fscanf(fp, "%f", &net_data[i][j]);

        fscanf(fp1, "%f", &r_net_data[i][j]);
    }

    for (i = 0; i < *n; i++) {
        for (j = i; j < *n; j++) {
            float diff = abs(net_data[i][*d - 1] - net_data[j][*d - 1]);
            cp_diff[i][j] = diff;
            cp_diff[j][i] = diff;
            T2 += diff;

            diff = abs(r_net_data[i][*d - 1] - r_net_data[j][*d - 1]);
            r_cp_diff[i][j] = diff;
            r_cp_diff[j][i] = diff;
        }
    }

    T1 /= (N * (N - 1));
    T2 /= (N * (N - 1) / 2);
    T1 *= K1;
    T2 *= K2;

    fclose(fp);
    fclose(fp1);
}

void storeData()
{
    int i, j;
    float avg_delay = 0.0, avg_cpdiff = 0.0;
    FILE* fp = fopen(o_route.c_str(), "w");
    fprintf(fp, "K=%d\n\n", K);
    for (i = 0; i < K; ++i) {
        float max_delay = 0.0, min_cp = 999.0, max_cp = 0.0;  // 每个域内的最大时延，最大算力，最小算力
        vector<int> points = points_list[i];
        int size = points.size() - 1;
        int cur_controller = points[size];  // 域内节点计数器数量N
        //cout << "[ ";
        fprintf(fp, "[ ");
        for (j = 0; j < size; ++j) {  // 查找每个控制域内的节点
            int cur_p = points[j];
            float cur_delay = r_net_data[cur_p][cur_controller], cur_cp = r_net_data[cur_p][D - 1];
            if (cur_delay > max_delay) max_delay = cur_delay;
            if (cur_cp < min_cp) min_cp = cur_cp;
            if (cur_cp > max_cp) max_cp = cur_cp;
            //cout << cur_p << " ";
            fprintf(fp, "%d ", cur_p);
        }
        //cout << "]\n";
        fprintf(fp, "]\n");
        fprintf(fp, "C=%d N=%d R=%f D=%f\n", cur_controller, size, min_cp, max_delay);

        avg_delay += max_delay;
        avg_cpdiff += (max_cp - min_cp);
    }

    avg_delay /= K;
    avg_cpdiff /= K;

    fprintf(fp, "\nK=%d, average_delay=%.4f, average_cpdiff=%.4f\n", K, avg_delay, avg_cpdiff);

    fclose(fp);
}
*/