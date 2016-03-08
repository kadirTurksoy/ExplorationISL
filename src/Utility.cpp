#include "Utility.h"

Graph::Graph(int v = 0):v(v){
    vertexList.resize(v);
    scanList.resize(v);

    current_place = 0;
    isPlaceFinished.push_back(false);

    for(int i= 0; i<v; i++){
        isFinished.push_back(false);

        isCritical.push_back(true);

        placeIDList.push_back(0);

        poseList.push_back(Position(0,0,0));
    }

}

int Graph::getSize(){return v;}

vector< map<int,double> > Graph::getGraph(){ return vertexList; }

bool Graph::is_connected(bool *graph[], int size){
    int old_size = 0, c_size = 0;
    bool* close = new bool[size];
    bool* open = new bool[size];
    for(int i = 0; i < size; ++i)
        open[i] = close[i] = false;
    open[0] = true;
    while(c_size < size){
        for(int i = 0; i < size; ++i){
            old_size = c_size;
            if (open[i] && (close[i] == false)){
                close[i] = true;
                c_size++;
            }
            for(int j = 0; j < size; ++j){
                open[j]=open[j]||graph[i][j];
            }
            if(c_size == size) return true;
            if(old_size == c_size) return false;
        }
    }
}

bool Graph::isAdj(int x, int y){
    if(x >= v) return false;
    if(y >= v) return false;

    if(vertexList[x].find(y)!=vertexList[x].end())
        return true;
    return false;
}

void Graph::addVertex(){
    v++;
    vertexList.resize(v);
}

void Graph::addEdge(int a, int b, double distance){
    if(a < v && b < v){
        vertexList[a][b] = distance;
        vertexList[b][a] = distance;
    }
}

double Graph::shortestPath(int source, int destination){
    vector<double> dist(v);
    vector<bool> cancel(v);
    int proc = source;
    int oldProc = source;
    double minDistance = INFINITY;
    bool flag = false;

    if(source > v || destination > v)
        return 0;

    for(int i = 0; i < v; i++){
        cancel[i] = false;
        if(i == source)
            dist[i] = 0.0;
        else
            dist[i] = INFINITY;
    }

    while (flag == false){
        for(map<int,double>::iterator itr = vertexList[proc].begin();
            itr != vertexList[proc].end(); itr++){
            if(cancel[itr->first] == false && isAdj(proc, itr->first)){
                if(dist[itr->first] > (dist[proc] + vertexList[proc][itr->first]))
                    dist[itr->first] = dist[proc] + vertexList[proc][itr->first];
            }
        }
        cancel[proc] = true;
        oldProc = proc;
        for(int i = 0; i < v; i++){
            if(!cancel[i] && dist[i] != INFINITY && dist[i] < minDistance){
                proc = i;
                minDistance = dist[i];
            }
        }
        if(proc == destination)
            return dist[destination];
        if(proc == oldProc)
            return 0;
        minDistance = INFINITY;
    }

}

int Graph::findBackTrace(int n, int id){
    double min = INFINITY;
        double back_trace = n;
        int index = n;

        for(int i=0; i<v; i++){
            double d = shortestPath(n,i);
            if(i != n && d < min && isFinished[i] == false && placeIDList[i] == current_place){
                min = d;
                index = i;
            }
        }

        for(map<int,double>::iterator itr = vertexList[n].begin();
            itr != vertexList[n].end(); itr++){

            double d = shortestPath(index,itr->first);
            if (index != n){
                if((d + vertexList[n][itr->first] - min) < 0.01){
                    back_trace = itr->first;
                }
            }
        }

        if(back_trace == n){
            if (id == current_place){
                back_trace = -1;
            }
            else{
                current_place = id;
            }
        }

        return back_trace;
}

Graph::~Graph(){
    //cout<<"Destructor used..."<<endl;
}

int Conncomp1d::findcompNumber() {
    compNumber=0;
    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[i]==1)
                compNumber = compNumber + 1;
        }
        if(i!=0) {
            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                compNumber = compNumber + 1;
            }
        }
    }
    return compNumber;
}

void Conncomp1d::findcompwidth() {
    cx = -1;

    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[0]==1) {
                cx = cx + 1;
                compwidth[cx] = 1;
            }
        }

        else {

            if (binaryImage[i]==1 && binaryImage[i-1]==1) {
                compwidth[cx] = compwidth[cx] + 1;
            }

            if (binaryImage[i]==0 && binaryImage[i-1]==1) {
                cx = cx + 1;
                compwidth[cx] = 0;
            }

            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                if (cx==-1) {
                    cx = 0;
                }

                compwidth[cx] = 1;
            }
        }
    }
}

void Conncomp1d::findcentroid() {
    cx=-1;

    for (i = 0; i < count; i++){
        if(i==0){
            if (binaryImage[0]==1) {
                cx = cx + 1;
                centroid[cx] = 0;
            }
        }

        else {
            if (binaryImage[i]==1 && binaryImage[i-1]==1) {
                centroid[cx] = centroid[cx] + 0.5;
            }

            if (binaryImage[i]==1 && binaryImage[i-1]==0) {
                cx = cx + 1;
                centroid[cx]=i;
            }
        }
    }
}

int Conncomp1d::SetValues1(int *p1, int count)
{
    if(count > 2000) return -1;

    this->count = count;

    for(int i = 0; i < count; i++)
        binaryImage[i] = p1[i];
    return 0;
}


double StdDeviation::CalculateMean()
{
    double sum = 0;
    for(int i = 0; i < count; i++)
        sum += value[i];
    return (sum / count);
}

double StdDeviation::CalculateVariane()
{
    mean = CalculateMean();

    double temp = 0;
    for(int i = 0; i < count; i++)
    {
        temp += (value[i] - mean) * (value[i] - mean) ;
    }
    return temp / count;
}

double StdDeviation::CalculateSampleVariane()
{
    mean = CalculateMean();

    double temp = 0;
    for(int i = 0; i < count; i++)
    {
        temp += (value[i] - mean) * (value[i] - mean) ;
    }
    return temp / (count - 1);
}

int StdDeviation::SetValues(double *p, int count)
{
    if(count > 10000)
        return -1;
    this->count = count;
    for(int i = 0; i < count; i++)
        value[i] = p[i];
    return 0;
}

double StdDeviation::GetStandardDeviation()
{
    return sqrt(CalculateVariane());
}

double StdDeviation::GetSampleStandardDeviation()
{
    return sqrt(CalculateSampleVariane());
}
