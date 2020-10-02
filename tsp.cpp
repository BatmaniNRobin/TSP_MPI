#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <string.h>
#include <limits.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <stdint.h>
#include <float.h>

using namespace std;

// struct for every city, tuple of x, y joined with ID, pretty much just a triplet
typedef struct
{
    int id;
    float x;
    float y;
} City;


// read cities from file and put into cities triplet
vector<City> getCities(char* filePath)
{
    vector<City> cities;
    fstream file(filePath);
    int id = 0;
    double x, y;

    if(file.is_open())
    {
        while(file >> x >> y)
        {
            City city;
            city.id = id;
            city.x = x;
            city.y = y;
            cities.push_back(city);
            id++;
            // cout << x << " " << y << " hi mani how are you today" << endl;
        }
        file.close();
    }

    return cities;
}

// calculate the euclidean distance between every city
double** computeEuclideanDistanceMatrix(vector<City> cities, double** distances)
{
    for(int i = 0; i < cities.size(); i++)
    {
        distances[i] = (double*)malloc(cities.size() * sizeof(double *));
        for(int j = 0; j < cities.size(); j++)
        {
            City city1 = cities[i];
            City city2 = cities[j];
            distances[i][j] = sqrt(pow(city1.x - city2.x, 2) + pow(city1.y - city2.y, 2));
        }
    }

    return distances;
}

// A utility function to print the 
// constructed MST stored in parent[] 
void printMST(double parent[], vector<City> cities, double** distances) 
{ 
    cout<<"Edge \tWeight\n"; 
    for (int i = 1; i < cities.size(); i++)
    {
        cout<<parent[i]<<" - "<<i<<" \t"<<distances[i][int(parent[i])]<<" \n"; 
    }
} 

// A utility function to find the vertex with 
// minimum key value, from the set of vertices 
// not yet included in MST 
int minKey(double key[], bool mstSet[], vector<City> cities) 
{ 
    // Initialize min value 
    int min = INT_MAX, min_index; 
 
    for (int v = 0; v < cities.size(); v++) 
        if (mstSet[v] == false && key[v] < min) 
            min = key[v], min_index = v; 
 
    return min_index;
}

// call gamestop to do preorder traversal
void gamestop(vector<City> cities, double* parent, bool* node_visited, double** distances, int root, vector<int>* path, double* cost)
{
    if(node_visited[root] == false)
    {
        if(root != 0)
        {
            (*cost) = (*cost) + distances[(*path)[path->size() - 1]][root];
        }
        node_visited[root] = true;
        path->push_back(root);
    }

    for(int i = 0; i < cities.size(); i++)
    {
        if(parent[i] == root)
        {
            gamestop(cities, parent, node_visited, distances, i, path, cost);
        }
    }
}

void primMST(vector<City> cities, double** distances, vector<int>* path, double* cost)
{
    double parent[cities.size()];
    double key[cities.size()];
    bool mstSet[cities.size()];
    bool visited[cities.size()];
    
    // init all keys as INF
    for(int i = 0; i < cities.size(); i++)
    {
        key[i] = INT_MAX;
        mstSet[i] = false;
    }

    key[0] = 0;
    parent[0] = -1; // first node is always root of MST

    for(int i = 0; i < cities.size(); i++)
    {
        int u = minKey(key, mstSet, cities);
        mstSet[u] = true;

        for(int j = 0; j < cities.size(); j++)
        {
            if (distances[u][j] && mstSet[j] == false && distances[u][j] < key[j])
            {
                parent[j] = u, key[j] = distances[u][j];
            }
        }
    }

    for(int i = 0; i < cities.size(); i++)
    {
        visited[i] = false;
    }
    gamestop(cities, parent, visited, distances, 0, path, cost);
    // printMST(parent, cities, distances);
}


int get_lowest_weights_city_number(double** distances, int c, bool* visited, int length, double* cost)
{
    double kmin;
    int nc = 999;
    double min = 999;

    for(int i = 0; i < length; i++)
    {
        if((distances[c][i] != 0) && (visited[i] == false))
        {
            if(distances[c][i] + distances[i][c] < min)
            {
                min = distances[i][0] + distances[c][i];
                kmin = distances[c][i];
                nc = i;
            }
        }
    }
    if(min != 999)
    {
        (*cost) += kmin;
    }
    return nc;
}


void TSP_Serial_DP(double** distances, int length, vector<int>* path, double* cost, int city, bool* visited)
{  
    int ncity;
    visited[city] = true;
    path->push_back(city+1);
    ncity = get_lowest_weights_city_number(distances, city, visited, length, cost);
    if(ncity == 999)
    {
        ncity = 0;
        (*cost) += distances[city][ncity];
        return;
    }
    // cout << "ncity: "<< ncity << endl;
    TSP_Serial_DP(distances, length, path, cost, ncity, visited);
}

// TODO CHECK ALL OPENMP FOR ACTUAL ACCURACY

void TSP_MP_EMST(vector<City> cities, double** distances, vector<int>* path, double* cost)
{
    double parent[cities.size()];
    double key[cities.size()];
    bool mstSet[cities.size()];
    bool visited[cities.size()];
    
    // init all keys as INF
    #pragma omp parallel for
    for(int i = 0; i < cities.size(); i++)
    {
        key[i] = INT_MAX;
        mstSet[i] = false;
    }

    key[0] = 0;
    parent[0] = -1; // first node is always root of MST

    #pragma omp parallel for
    for(int i = 0; i < cities.size(); i++)
    {
        int u;
        
        int min = INT_MAX, min_index; 
 
        for (int v = 0; v < cities.size(); v++)
        {
            if (mstSet[v] == false && key[v] < min) 
            {
                min = key[v], min_index = v; 
            }
        }

        u = min_index;

        mstSet[u] = true;

        for(int j = 0; j < cities.size(); j++)
        {
            if (distances[u][j] && mstSet[j] == false && distances[u][j] < key[j])
            {
                parent[j] = u, key[j] = distances[u][j];
            }
        }
    }

    #pragma omp parallel for
    for(int i = 0; i < cities.size(); i++)
    {
        visited[i] = false;
    }

    gamestop(cities, parent, visited, distances, 0, path, cost);
    // printMST(parent, cities, distances);
}

void TSP_MP_DP(double** distances, int length, vector<int>* path, double* cost, bool* visited)
{

}

void TSP_Parallel_EMST(vector<City> cities, double** distances, vector<int>* path, double* cost)
{

}

void TSP_Parallel_DP(vector<City> cities, double** distances, vector<int>* path, double* cost)
{

}

int main(int argc, char* argv[])
{
    double** distances;
    
    if(argc != 2)
    {
        printf("Usage: ./tsp [citiesFile].txt\n");
        exit(1);
    }

    vector<City> cities = getCities(argv[1]);

    if(cities.size() == 0)
    {
        printf("use nonempty dataset\n");
        exit(2);
    }

    int length = cities.size();
    distances = (double**)malloc(cities.size() * sizeof(double*));

    vector<int> bestPath;
    bool visited[length];
    int location = 0;
    double cost = 0.00;

    computeEuclideanDistanceMatrix(cities, distances);

    struct timespec start, end;

    //--------------------------- Euclidean Min Span Tree ------------------------------------------

    clock_gettime(CLOCK_MONOTONIC_RAW, &start);    

    primMST(cities, distances, &bestPath, &cost);

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    uint64_t diff = (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec) / 1e6;

    // cout << "TSP Serial EMST Cost: " << cost << endl;
    // cout << "TSP Serial EMST Path:";
    // for(int i = 0; i < bestPath.size(); i++)
    // {
    //     cout << " " << bestPath[i];
    // }
    // cout << endl;

    printf("TSP Serial EMST ran in %llu ms for %i cities\n\n", (long long unsigned int)diff, cities.size());

    bestPath.clear();
    cost = 0.00;

    //----------------------------- Dynamic Programming ----------------------------------------------

    clock_gettime(CLOCK_MONOTONIC_RAW, &start); 

    TSP_Serial_DP(distances, length, &bestPath, &cost, location, visited);

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    diff = (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec) / 1e6;

    // cout << "TSP Serial DP Cost: " << cost << endl;
    // cout << "TSP Serial DP Path:";
    // for(int i = 0; i < bestPath.size(); i++)
    // {
    //     cout << " " << bestPath[i];
    // }
    // cout << endl;

    printf("TSP Serial DP ran in %llu ms for %i cities\n\n", (long long unsigned int)diff, cities.size());

    bestPath.clear();
    cost = 0.00;

    //--------------------------- MP EMST -----------------------------------------------------------

    clock_gettime(CLOCK_MONOTONIC_RAW, &start);    

    TSP_MP_EMST(cities, distances, &bestPath, &cost);

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    diff = (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec) / 1e6;

    // cout << "TSP Parallel EMST Cost: " << cost << endl;
    // cout << "TSP Parallel EMST Path:";
    // for(int i = 0; i < bestPath.size(); i++)
    // {
    //     cout << " " << bestPath[i];
    // }
    // cout << endl;

    printf("TSP OpenMP EMST ran in %llu ms for %i cities\n\n", (long long unsigned int)diff, cities.size());

    bestPath.clear();
    cost = 0.00;

    //----------------------------- MP DP -----------------------------------------------------------

    clock_gettime(CLOCK_MONOTONIC_RAW, &start); 

    // TSP_MP_DP(distances, length, &bestPath, &cost, location, visited);

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    diff = (1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec) / 1e6;

    // cout << "TSP Serial DP Cost: " << cost << endl;
    // cout << "TSP Serial DP Path:";
    // for(int i = 0; i < bestPath.size(); i++)
    // {
    //     cout << " " << bestPath[i];
    // }
    // cout << endl;

    printf("TSP OpenMP DP ran in %llu ms for %i cities\n\n", (long long unsigned int)diff, cities.size());

    bestPath.clear();
    cost = 0.00;

    return 0;
}