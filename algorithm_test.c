#include "core.h"
#include <time.h>
#include <string.h>
#include <math.h>

static int integer(const char *label, int lo, int hi) {
    int x;
    for (;;) {
        printf("%s", label);
        int status = scanf("%d", &x);
        if (status == EOF) exit(0);
        if (status == 1 && x >= lo && x <= hi) return x;
        printf("Invalid input (expected %d..%d).\n", lo, hi);
        int c; while ((c = getchar()) != '\n' && c != EOF) {}
    }
}
static double number(const char *label, double lo, double hi) {
    double x;
    for (;;) {
        printf("%s", label);
        int status = scanf("%lf", &x);
        if (status == EOF) exit(0);
        if (status == 1 && isfinite(x) && x >= lo && x <= hi) return x;
        printf("Invalid numeric input.\n");
        int c; while ((c = getchar()) != '\n' && c != EOF) {}
    }
}
static void graph(CityMap *m) {
    int n = integer("Vertices (2..25): ", 2, MAX_NODES);
    init_city_map(m, n);
    int e = integer("Number of undirected edges: ", 0, n*(n-1)/2);
    puts("Enter each edge (endpoints are 0-based):");
    for (int i=0; i<e; i++) {
        int u=integer("  From: ",0,n-1), v=integer("  To: ",0,n-1);
        if (u==v) {puts("  Self-loop ignored; enter another edge."); i--; continue;}
        double w=number("  Weight (nonnegative): ",0,1000000);
        if (w<m->adj_matrix[u][v]) m->adj_matrix[u][v]=m->adj_matrix[v][u]=w;
    }
}
static void distances(const double *d, int n) {
    for (int i=0;i<n;i++) {
        printf("  Vertex %d: ",i);
        if (d[i]>=INF_TIME) puts("unreachable"); else printf("%.3f\n",d[i]);
    }
}
static void bellman(const CityMap *m, int source, double *d) {
    int n=m->num_nodes;
    for (int i=0;i<n;i++) d[i]=INF_TIME;
    d[source]=0;
    for (int k=1;k<n;k++) {
        int changed=0;
        for(int u=0;u<n;u++) if(d[u]<INF_TIME)
            for(int v=0;v<n;v++) if(m->adj_matrix[u][v]<INF_TIME && d[u]+m->adj_matrix[u][v]<d[v]) {
                d[v]=d[u]+m->adj_matrix[u][v]; changed=1;
            }
        if(!changed) break;
    }
}
static void mst(const CityMap *m) {
    int n=m->num_nodes, used[MAX_NODES]={0}, parent[MAX_NODES];
    double key[MAX_NODES], total=0;
    for(int i=0;i<n;i++){key[i]=INF_TIME;parent[i]=-1;}
    key[0]=0;
    puts("Prim minimum spanning forest (reachable component from vertex 0):");
    int edges=0;
    for(int step=0;step<n;step++) {
        int u=-1;
        for(int i=0;i<n;i++)if(!used[i]&&(u<0||key[i]<key[u]))u=i;
        if(u<0||key[u]>=INF_TIME)break;
        used[u]=1;
        if(parent[u]>=0){printf("  %d -- %d (%.3f)\n",parent[u],u,key[u]);total+=key[u];edges++;}
        for(int v=0;v<n;v++)if(!used[v]&&m->adj_matrix[u][v]<key[v]){key[v]=m->adj_matrix[u][v];parent[v]=u;}
    }
    printf("Prim cost: %.3f; edges: %d\n",total,edges);
    if(edges!=n-1)puts("Graph disconnected: no spanning tree covering every vertex.");
    /* Kruskal, independently computed from the same input. */
    typedef struct {int u,v;double w;} Edge;
    Edge es[MAX_NODES*MAX_NODES/2]; int ne=0,dsu[MAX_NODES];
    for(int i=0;i<n;i++){dsu[i]=i;for(int j=i+1;j<n;j++)if(m->adj_matrix[i][j]<INF_TIME)es[ne++]=(Edge){i,j,m->adj_matrix[i][j]};}
    for(int i=1;i<ne;i++){Edge x=es[i];int j=i-1;while(j>=0&&es[j].w>x.w){es[j+1]=es[j];j--;}es[j+1]=x;}
    total=0;edges=0;
    puts("Kruskal edges:");
    for(int i=0;i<ne;i++){
        int a=es[i].u,b=es[i].v;
        while(dsu[a]!=a) a=dsu[a];
        while(dsu[b]!=b) b=dsu[b];
        if(a!=b){dsu[a]=b;total+=es[i].w;edges++;printf("  %d -- %d (%.3f)\n",es[i].u,es[i].v,es[i].w);}
    }
    printf("Kruskal cost: %.3f; edges: %d\n",total,edges);
}
static void sort_demo(void) {
    int n=integer("Number of values (1..200): ",1,200);
    Order *a=calloc((size_t)n,sizeof(*a));if(!a){puts("Out of memory");return;}
    for(int i=0;i<n;i++){char label[60];sprintf(label,"Value %d: ",i+1);a[i].ready_time=number(label,-1e7,1e7);a[i].order_id=i+1;}
    daa_merge_sort_orders(a,0,n-1,1);
    puts("Sorted values:");for(int i=0;i<n;i++)printf(" %.3f",a[i].ready_time);puts("");
    free(a);
}
static void binary_demo(void) {
    int n=integer("Number of sorted values (1..200): ",1,200);
    double a[200];
    for(int i=0;i<n;i++){char label[60];sprintf(label,"Sorted value %d: ",i+1);a[i]=number(label,-1e7,1e7);if(i&&a[i]<a[i-1]){puts("Values must be nondecreasing. Try again.");i--;}}
    double x=number("Search value: ",-1e7,1e7);
    int lo=0,hi=n-1,found=-1;
    while(lo<=hi){int mid=lo+(hi-lo)/2;if(a[mid]==x){found=mid;break;}if(a[mid]<x)lo=mid+1;else hi=mid-1;}
    if(found<0)puts("Not found.");else printf("Found at index %d (0-based).\n",found);
}
static void knapsack(void) {
    int n=integer("Number of items (1..30): ",1,30);
    int cap=integer("Bag capacity (0..500): ",0,500);
    int w[30],v[30];
    for(int i=0;i<n;i++){char label[60];sprintf(label,"Item %d weight: ",i+1);w[i]=integer(label,1,500);sprintf(label,"Item %d value: ",i+1);v[i]=integer(label,0,100000);}
    int *dp=calloc((size_t)(n+1)*(cap+1),sizeof(int));if(!dp){puts("Out of memory");return;}
    #define DP(i,c) dp[(size_t)(i)*(cap+1)+(c)]
    for(int i=1;i<=n;i++)for(int c=0;c<=cap;c++){
        DP(i,c)=DP(i-1,c);
        if(w[i-1]<=c && DP(i-1,c-w[i-1])+v[i-1]>DP(i,c))DP(i,c)=DP(i-1,c-w[i-1])+v[i-1];
    }
    printf("Optimal 0/1 knapsack value: %d\nSelected item indices (1-based):",DP(n,cap));
    int c=cap;for(int i=n;i>0;i--)if(DP(i,c)!=DP(i-1,c)){printf(" %d",i);c-=w[i-1];}puts("");
    #undef DP
    free(dp);
}
static void dispatch_demo(void) {
    CityMap m;generate_sample_city(&m);daa_floyd_warshall(&m);
    int n=integer("Number of orders (5..100): ",5,100);
    int stations=integer("Kitchen stations (1..25): ",1,25);
    int riders=integer("Riders (1..35): ",1,35);
    int bag=integer("Bag capacity kg (1..24): ",1,24);
    int batch=integer("Maximum drops per trip (1..5): ",1,5);
    Order *o=allocate_and_generate_orders(n,m.num_nodes);if(!o){puts("Allocation failed");return;}
    daa_schedule_kitchen(o,n,&m,0,stations);
    int pruned=daa_dispatch_knapsack_bnb(o,n,&m,0,riders,bag,batch);
    int served=0,ontime=0;
    for(int i=0;i<n;i++)if(o[i].service_status==1){served++;ontime+=(o[i].delivered_time<=o[i].due_time);}
    printf("Served: %d/%d; on-time: %.1f%%; branch-and-bound pruned: %d\n",served,n,100.0*ontime/n,pruned);
    puts("Order -> rider / trip / delivery minute:");
    for(int i=0;i<n;i++)printf("  #%d -> rider %d, trip %d, %.2f min (%s)\n",o[i].order_id,o[i].assigned_rider,o[i].batch_id,o[i].delivered_time,o[i].service_status==1?"served":"unserved");
    free(o);
}
int main(void) {
    for(;;){
        puts("\n========== DOONEATS INTERACTIVE ALGORITHM TESTER ==========");
        puts("1. BFS connectivity (custom graph)");
        puts("2. Dijkstra shortest paths (custom graph)");
        puts("3. Floyd-Warshall all-pairs (custom graph)");
        puts("4. Merge sort (custom values)");
        puts("5. Binary search (custom sorted values)");
        puts("6. 0/1 knapsack (custom items)");
        puts("7. Branch-and-bound delivery routing (custom scenario)");
        puts("8. Prim and Kruskal MST (custom graph)");
        puts("9. Bellman-Ford shortest paths (custom graph)");
        puts("0. Exit");
        int choice=integer("Select: ",0,9);
        if(choice==0)break;
        clock_t start=clock();
        if(choice==4)sort_demo();
        else if(choice==5)binary_demo();
        else if(choice==6)knapsack();
        else if(choice==7)dispatch_demo();
        else {
            CityMap m;graph(&m);
            if(choice==1){int source=integer("Start vertex: ",0,m.num_nodes-1);daa_bfs_connectivity(&m,source);for(int i=0;i<m.num_nodes;i++)printf("  Vertex %d: %s\n",i,m.bfs_reachable[i]?"reachable":"unreachable");}
            else if(choice==2||choice==9){int source=integer("Source vertex: ",0,m.num_nodes-1);double d[MAX_NODES];if(choice==2)daa_dijkstra(&m,source,d);else bellman(&m,source,d);distances(d,m.num_nodes);}
            else if(choice==3){daa_floyd_warshall(&m);puts("All-pairs shortest path matrix (INF=unreachable):");for(int i=0;i<m.num_nodes;i++){for(int j=0;j<m.num_nodes;j++)if(m.shortest_dist[i][j]>=INF_TIME)printf(" %8s","INF");else printf(" %8.2f",m.shortest_dist[i][j]);puts("");}}
            else if(choice==8)mst(&m);
        }
        printf("Elapsed CPU time: %.3f ms\n",1000.0*(clock()-start)/CLOCKS_PER_SEC);
    }
    puts("Goodbye.");return 0;
}
