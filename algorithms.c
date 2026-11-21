#include "core.h"
#include <string.h>
static double max_double(double a, double b) { return a > b ? a : b; }
/* ============================================================================
   1. BREADTH-FIRST SEARCH (BFS)
   Purpose: Verify that all city sectors are reachable from the hub.
   Graph representation: adjacency matrix
   Time: O(V^2) with adjacency matrix | Space: O(V)
   ============================================================================ */
void daa_bfs_connectivity(CityMap* map, int start_node) {
    int q[MAX_NODES];
    int front = 0, rear = 0, i;
    if (!map || start_node < 0 || start_node >= map->num_nodes) return;
    for (i = 0; i < map->num_nodes; ++i) map->bfs_reachable[i] = 0;
    map->bfs_reachable[start_node] = 1;
    q[rear++] = start_node;
    while (front < rear) {
        int u = q[front++];
        int v;
        for (v = 0; v < map->num_nodes; ++v) {
            if (u != v && map->adj_matrix[u][v] < INF_TIME && !map->bfs_reachable[v]) {
                map->bfs_reachable[v] = 1;
                q[rear++] = v;
            }
        }
    }
}
/* ============================================================================
   2. DIJKSTRA'S ALGORITHM - GREEDY
   Purpose: Find shortest travel times from one source sector to every other
            sector when all road weights are non-negative.
   Project use: Single-source routing alternative to Floyd-Warshall.
   Implementation: Adjacency matrix, no library priority queue.
   Time: O(V^2) | Space: O(V)
   ============================================================================ */
void daa_dijkstra(const CityMap* map, int source, double dist[MAX_NODES]) {
    int visited[MAX_NODES];
    int n, i, count;
    if (!map || !dist) return;
    n = map->num_nodes;
    if (source < 0 || source >= n) return;
    for (i = 0; i < n; ++i) {
        dist[i] = INF_TIME;
        visited[i] = 0;
    }
    dist[source] = 0.0;
    for (count = 0; count < n; ++count) {
        int u = -1, v;
        for (i = 0; i < n; ++i)
            if (!visited[i] && (u == -1 || dist[i] < dist[u])) u = i;
        if (u == -1 || dist[u] >= INF_TIME) break;
        visited[u] = 1;
        for (v = 0; v < n; ++v) {
            double w = map->adj_matrix[u][v];
            if (!visited[v] && u != v && w < INF_TIME &&
                dist[u] + w < dist[v])
                dist[v] = dist[u] + w;
        }
    }
}
/* ============================================================================
   3. FLOYD-WARSHALL - DYNAMIC PROGRAMMING
   Purpose: Precompute shortest travel time between every pair of sectors.
   Recurrence: D[i][j] = min(D[i][j], D[i][k] + D[k][j])
   Time: O(V^3) | Extra space: O(1), using CityMap's stored matrix
   ============================================================================ */
void daa_floyd_warshall(CityMap* map) {
    int i, j, k, n;
    if (!map) return;
    n = map->num_nodes;
    for (i = 0; i < n; ++i) {
        for (j = 0; j < n; ++j) {
            map->shortest_dist[i][j] = map->adj_matrix[i][j];
            if (i == j) map->next_hop[i][j] = i;
            else if (map->adj_matrix[i][j] < INF_TIME) map->next_hop[i][j] = j;
            else map->next_hop[i][j] = -1;
        }
    }
    for (k = 0; k < n; ++k) {
        for (i = 0; i < n; ++i) {
            for (j = 0; j < n; ++j) {
                double through_k;
                if (map->shortest_dist[i][k] >= INF_TIME || map->shortest_dist[k][j] >= INF_TIME) continue;
                through_k = map->shortest_dist[i][k] + map->shortest_dist[k][j];
                if (through_k < map->shortest_dist[i][j]) {
                    map->shortest_dist[i][j] = through_k;
                    map->next_hop[i][j] = map->next_hop[i][k];
                }
            }
        }
    }
}
/* ============================================================================
   4. MERGE SORT - DIVIDE AND CONQUER
   Purpose: Sort orders by ready time or priority slack.
   Time: O(n log n) | Auxiliary space: O(n)
   ============================================================================ */
static void merge_orders(Order* arr, int left, int mid, int right, int sort_by_ready) {
    int n1 = mid - left + 1, n2 = right - mid;
    int i = 0, j = 0, k = left;
    Order* L = (Order*)malloc(sizeof(Order) * n1);
    Order* R = (Order*)malloc(sizeof(Order) * n2);
    if (!L || !R) { free(L); free(R); return; }
    for (i = 0; i < n1; ++i) L[i] = arr[left + i];
    for (j = 0; j < n2; ++j) R[j] = arr[mid + 1 + j];
    i = 0; j = 0;
    while (i < n1 && j < n2) {
        double a = sort_by_ready ? L[i].ready_time : L[i].priority_slack;
        double b = sort_by_ready ? R[j].ready_time : R[j].priority_slack;
        arr[k++] = (a <= b) ? L[i++] : R[j++];
    }
    while (i < n1) arr[k++] = L[i++];
    while (j < n2) arr[k++] = R[j++];
    free(L); free(R);
}
void daa_merge_sort_orders(Order* arr, int left, int right, int sort_by_ready) {
    if (left < right) {
        int mid = left + (right - left) / 2;
        daa_merge_sort_orders(arr, left, mid, sort_by_ready);
        daa_merge_sort_orders(arr, mid + 1, right, sort_by_ready);
        merge_orders(arr, left, mid, right, sort_by_ready);
    }
}
/* ============================================================================
   5. GREEDY KITCHEN SCHEDULING
   Purpose: Assign each order to the earliest-free prep station and prioritize
            available orders with the least slack.
   Note: Uses linear selection, NOT a library min-heap.
   Time: O(n^2 + n*s) for n orders and s stations.
   ============================================================================ */
void daa_schedule_kitchen(Order* orders, int count, const CityMap* map,
                                  int kitchen_node, int num_stoves) {
    double* stove_free;
    int* cooked;
    int i, step;
    if (!orders || !map || count <= 0 || num_stoves <= 0) return;
    for (i = 0; i < count; ++i) {
        double travel = map->shortest_dist[kitchen_node][orders[i].customer_node];
        if (orders[i].order_id != 999) {
            double sla_buffer = orders[i].due_time;
            orders[i].due_time = orders[i].arrival_time + orders[i].prep_time + travel + sla_buffer;
        }
        orders[i].priority_slack = orders[i].due_time - orders[i].prep_time - travel;
        orders[i].urgency_score = (int)(1200.0 / (orders[i].priority_slack - orders[i].arrival_time + 15.0));
        if (orders[i].urgency_score < 15) orders[i].urgency_score = 15;
        orders[i].ready_time = orders[i].arrival_time;
    }
    daa_merge_sort_orders(orders, 0, count - 1, 1);
    stove_free = (double*)calloc((size_t)num_stoves, sizeof(double));
    cooked = (int*)calloc((size_t)count, sizeof(int));
    if (!stove_free || !cooked) { free(stove_free); free(cooked); return; }
    for (step = 0; step < count; ++step) {
        int s, best_s = 0, chosen = -1;
        double t_free, min_slack = INF_TIME;
        for (s = 1; s < num_stoves; ++s)
            if (stove_free[s] < stove_free[best_s]) best_s = s;
        t_free = stove_free[best_s];
        for (i = 0; i < count; ++i) {
            if (!cooked[i] && orders[i].arrival_time <= t_free + 1.5 &&
                orders[i].priority_slack < min_slack) {
                min_slack = orders[i].priority_slack;
                chosen = i;
            }
        }
        if (chosen == -1) {
            double earliest = INF_TIME;
            for (i = 0; i < count; ++i) {
                if (!cooked[i] && orders[i].arrival_time < earliest) {
                    earliest = orders[i].arrival_time;
                    chosen = i;
                }
            }
        }
        if (chosen < 0) break;
        cooked[chosen] = 1;
        {
            double start = max_double(orders[chosen].arrival_time, t_free);
            double load_delay = (num_stoves < 4) ? (4 - num_stoves) * 1.4 : 0.0;
            orders[chosen].ready_time = start + orders[chosen].prep_time + load_delay;
            orders[chosen].assigned_stove = best_s + 1;
            stove_free[best_s] = orders[chosen].ready_time;
        }
    }
    free(stove_free); free(cooked);
    daa_merge_sort_orders(orders, 0, count - 1, 1);
}
/* ============================================================================
   6. MULTI-STOP ROUTE OPTIMIZATION - BRANCH AND BOUND
   Purpose: Find the delivery order with minimum cumulative customer waiting.
   Worst case: O(b!) for b stops; pruning reduces explored permutations.
   Bound is safe because future travel times are non-negative.
   ============================================================================ */
typedef struct {
    const CityMap* map;
    const Order* orders;
    int batch_indices[8], best_seq[8], visited[8], cur_seq[8];
    int b_size;
    double best_cost;
    int nodes_pruned;
} BnBState;
static void tsp_branch_and_bound(BnBState* st, int depth, int curr_node,
                                 double elapsed, double cumulative_wait) {
    int i;
    if (depth == st->b_size) {
        if (cumulative_wait < st->best_cost) {
            st->best_cost = cumulative_wait;
            for (i = 0; i < st->b_size; ++i) st->best_seq[i] = st->cur_seq[i];
        }
        return;
    }
    {
        int remaining = st->b_size - depth;
        double lower_bound = cumulative_wait + elapsed * remaining;
        if (lower_bound >= st->best_cost) { st->nodes_pruned++; return; }
    }
    for (i = 0; i < st->b_size; ++i) {
        if (!st->visited[i]) {
            int idx = st->batch_indices[i];
            int next_node = st->orders[idx].customer_node;
            double d = st->map->shortest_dist[curr_node][next_node];
            st->visited[i] = 1;
            st->cur_seq[depth] = idx;
            tsp_branch_and_bound(st, depth + 1, next_node,
                                 elapsed + d, cumulative_wait + elapsed + d);
            st->visited[i] = 0;
        }
    }
}
/* ============================================================================
   7. 0/1 KNAPSACK + GREEDY RIDER ASSIGNMENT + ROUTE B&B
   Knapsack purpose: choose extra orders maximizing urgency within remaining bag
                     capacity and maximum drops per trip.
   Knapsack time: O(candidates * capacity * maxBatch)
   ============================================================================ */
int daa_dispatch_knapsack_bnb(Order* orders, int count, const CityMap* map,
                                      int kitchen_node, int num_riders,
                                      int max_bag_kg, int max_batch) {
    Rider* riders;
    int* dispatched;
    int delivered_count = 0, batch_id = 1, total_pruned = 0;
    int r;
    if (!orders || !map || count <= 0 || num_riders <= 0 || max_bag_kg <= 0 || max_batch <= 0) return 0;
    if (max_batch > 8) max_batch = 8;
    riders = (Rider*)malloc(sizeof(Rider) * (size_t)num_riders);
    dispatched = (int*)calloc((size_t)count, sizeof(int));
    if (!riders || !dispatched) { free(riders); free(dispatched); return 0; }
    for (r = 0; r < num_riders; ++r) {
        riders[r].rider_id = r + 1;
        riders[r].current_node = kitchen_node;
        riders[r].available_time = 0.0;
        riders[r].total_deliveries = 0;
    }
    while (delivered_count < count) {
        int anchor = -1, i;
        int candidates[12], cand_cnt = 0;
        int anchor_node;
        BnBState bnb;
        int remaining_cap, extra_slots;
        for (i = 0; i < count; ++i) if (!dispatched[i]) { anchor = i; break; }
        if (anchor < 0) break;
        if (orders[anchor].weight_kg > max_bag_kg) {
            orders[anchor].service_status = -1;
            orders[anchor].assigned_rider = 0;
            orders[anchor].batch_id = 0;
            orders[anchor].dispatch_time = 0.0;
            orders[anchor].delivered_time = 0.0;
            dispatched[anchor] = 1;
            delivered_count++;
            continue;
        }
        candidates[cand_cnt++] = anchor;
        anchor_node = orders[anchor].customer_node;
        for (i = anchor + 1; i < count && cand_cnt < 12; ++i) {
            if (!dispatched[i] && orders[i].ready_time - orders[anchor].ready_time <= 6.5 &&
                map->shortest_dist[anchor_node][orders[i].customer_node] <= 8.5)
                candidates[cand_cnt++] = i;
        }
        memset(&bnb, 0, sizeof(bnb));
        bnb.map = map; bnb.orders = orders; bnb.best_cost = INF_TIME;
        bnb.batch_indices[bnb.b_size++] = anchor;
        dispatched[anchor] = 1;
        remaining_cap = max_bag_kg - orders[anchor].weight_kg;
        extra_slots = max_batch - 1;
        if (cand_cnt > 1 && remaining_cap > 0 && extra_slots > 0) {
            int C = remaining_cap, K = extra_slots, N = cand_cnt - 1;
            int total = (N + 1) * (C + 1) * (K + 1);
            int* dp = (int*)calloc((size_t)total, sizeof(int));
            #define DP(ii,cc,kk) dp[((ii)*(C+1)+(cc))*(K+1)+(kk)]
            if (dp) {
                int ii, c, k;
                for (ii = 1; ii <= N; ++ii) {
                    int oi = candidates[ii];
                    int w = orders[oi].weight_kg, val = orders[oi].urgency_score;
                    for (c = 0; c <= C; ++c) for (k = 0; k <= K; ++k) {
                        DP(ii,c,k) = DP(ii-1,c,k);
                        if (k > 0 && c >= w && DP(ii-1,c-w,k-1) + val > DP(ii,c,k))
                            DP(ii,c,k) = DP(ii-1,c-w,k-1) + val;
                    }
                }
                {
                    int c = C, k = K;
                    for (ii = N; ii >= 1 && k > 0; --ii) {
                        int oi = candidates[ii], w = orders[oi].weight_kg;
                        if (c >= w && DP(ii,c,k) != DP(ii-1,c,k)) {
                            bnb.batch_indices[bnb.b_size++] = oi;
                            dispatched[oi] = 1;
                            c -= w; k--;
                        }
                    }
                }
                free(dp);
            }
            #undef DP
        }
        {
            double batch_ready = 0.0, earliest_depart = INF_TIME;
            int best_r = 0, b;
            for (b = 0; b < bnb.b_size; ++b) {
                int idx = bnb.batch_indices[b];
                if (orders[idx].ready_time > batch_ready) batch_ready = orders[idx].ready_time;
            }
            for (r = 0; r < num_riders; ++r) {
                double to_k = map->shortest_dist[riders[r].current_node][kitchen_node];
                double depart = max_double(batch_ready, riders[r].available_time + to_k);
                if (depart < earliest_depart) { earliest_depart = depart; best_r = r; }
            }
            tsp_branch_and_bound(&bnb, 0, kitchen_node, 0.0, 0.0);
            total_pruned += bnb.nodes_pruned;
            {
                double curr_t = earliest_depart;
                int curr_n = kitchen_node;
                for (b = 0; b < bnb.b_size; ++b) {
                    Order* o = &orders[bnb.best_seq[b]];
                    curr_t += map->shortest_dist[curr_n][o->customer_node];
                    if (b > 0) curr_t += 1.2;
                    o->dispatch_time = earliest_depart;
                    o->delivered_time = curr_t;
                    o->assigned_rider = riders[best_r].rider_id;
                    o->batch_id = batch_id;
                    o->bb_nodes_pruned = bnb.nodes_pruned;
                    o->service_status = 1;
                    curr_n = o->customer_node;
                    riders[best_r].total_deliveries++;
                    delivered_count++;
                }
                riders[best_r].current_node = curr_n;
                riders[best_r].available_time = curr_t;
            }
        }
        batch_id++;
    }
    free(dispatched); free(riders);
    return total_pruned;
}
/* ============================================================================
   8. BINARY SEARCH - READY ORDER CUTOFF
   Purpose: Orders are already sorted by ready_time using Merge Sort.
            Find the last order that is ready by a given dispatch cutoff time.
   Time: O(log n) | Space: O(1)
   ============================================================================ */
static int binary_search_ready_cutoff(const Order* orders, int count, double cutoff) {
    int left = 0, right = count - 1, answer = -1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (orders[mid].ready_time <= cutoff) {
            answer = mid;
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    return answer;
}

/* ============================================================================
   9. SUPPORTING NETWORK ALGORITHMS
   Only algorithms directly used by the current DoonEats network/diagnostic view
   are retained here: Prim, Kruskal and Bellman-Ford.
   ============================================================================ */
typedef struct { int u, v; double w; } Edge;
static int compare_edges(const void* a, const void* b) {
    double x = ((const Edge*)a)->w, y = ((const Edge*)b)->w;
    return (x > y) - (x < y);
}
static int dsu_find(int parent[], int x) {
    if (parent[x] != x) parent[x] = dsu_find(parent, parent[x]);
    return parent[x];
}
/* Prim: build one minimum spanning tree from vertex 0. O(V^2). */
static void prim_mst(const CityMap* map, EnterpriseDiagnostics* out) {
    int n = map->num_nodes, parent[MAX_NODES], used[MAX_NODES];
    double key[MAX_NODES], cost = 0.0;
    int i, step;
    for (i = 0; i < n; ++i) {
        key[i] = INF_TIME;
        parent[i] = -1;
        used[i] = 0;
    }
    key[0] = 0.0;
    out->mst_edge_count = 0;
    for (step = 0; step < n; ++step) {
        int u = -1, v;
        for (i = 0; i < n; ++i)
            if (!used[i] && (u < 0 || key[i] < key[u])) u = i;
        if (u < 0 || key[u] >= INF_TIME) break;
        used[u] = 1;
        cost += key[u];
        if (parent[u] >= 0) {
            out->mst_edge_u[out->mst_edge_count] = parent[u];
            out->mst_edge_v[out->mst_edge_count++] = u;
        }
        for (v = 0; v < n; ++v) {
            if (!used[v] && map->adj_matrix[u][v] < key[v]) {
                key[v] = map->adj_matrix[u][v];
                parent[v] = u;
            }
        }
    }
    out->prim_mst_cost = cost;
}
/* Kruskal: sort edges, then join components using disjoint sets. O(E log E). */
static void kruskal_mst(const CityMap* map, EnterpriseDiagnostics* out) {
    Edge edges[MAX_NODES * MAX_NODES / 2];
    int parent[MAX_NODES], edge_count = 0, n = map->num_nodes;
    double cost = 0.0;
    int u, v, i;
    for (u = 0; u < n; ++u)
        for (v = u + 1; v < n; ++v)
            if (map->adj_matrix[u][v] < INF_TIME)
                edges[edge_count++] = (Edge){u, v, map->adj_matrix[u][v]};
    qsort(edges, (size_t)edge_count, sizeof(Edge), compare_edges);
    for (i = 0; i < n; ++i) parent[i] = i;
    for (i = 0; i < edge_count; ++i) {
        int a = dsu_find(parent, edges[i].u);
        int b = dsu_find(parent, edges[i].v);
        if (a != b) {
            parent[a] = b;
            cost += edges[i].w;
        }
    }
    out->kruskal_mst_cost = cost;
}
/* Bellman-Ford: single-source shortest path used as a Dijkstra comparison. O(V^3)
   here because the graph is stored as an adjacency matrix. */
static double bellman_ford_to(const CityMap* map, int source, int target) {
    double dist[MAX_NODES];
    int n = map->num_nodes, i, step, u, v;
    for (i = 0; i < n; ++i) dist[i] = INF_TIME;
    dist[source] = 0.0;
    for (step = 0; step < n - 1; ++step) {
        int changed = 0;
        for (u = 0; u < n; ++u) {
            if (dist[u] >= INF_TIME) continue;
            for (v = 0; v < n; ++v) {
                double w = map->adj_matrix[u][v];
                if (w < INF_TIME && dist[u] + w < dist[v]) {
                    dist[v] = dist[u] + w;
                    changed = 1;
                }
            }
        }
        if (!changed) break;
    }
    return dist[target];
}
/* Small resource diagnostic used by the Kitchen page: maximum simultaneous
   preparation intervals = minimum stations needed for zero overlap waiting. */
static int minimum_stations_needed(const Order* orders, int count) {
    int i, j, peak = 0;
    for (i = 0; i < count; ++i) {
        double t = orders[i].ready_time - orders[i].prep_time;
        int active = 0;
        for (j = 0; j < count; ++j) {
            double start = orders[j].ready_time - orders[j].prep_time;
            double end = orders[j].ready_time;
            if (start <= t && t < end) active++;
        }
        if (active > peak) peak = active;
    }
    return peak;
}
void daa_run_syllabus_diagnostics(const CityMap* map, const Order* orders,
                                  int count, int num_riders, const char* query,
                                  EnterpriseDiagnostics* out) {
    double dijkstra_dist[MAX_NODES];
    (void)query;
    memset(out, 0, sizeof(*out));
    if (!map || !orders || count <= 0) return;
    daa_dijkstra(map, 0, dijkstra_dist);

    /* Real Binary Search use: find how many sorted orders are ready by 25 min. */
    out->bsearch_iter_idx = binary_search_ready_cutoff(orders, count, 25.0);

    out->min_stoves_interval_part = minimum_stations_needed(orders, count);
    prim_mst(map, out);
    kruskal_mst(map, out);
    out->bellman_ford_dist = bellman_ford_to(map, 0, 23);
    out->huffman_orig_bits = 0;
    out->huffman_comp_bits = 0;
    (void)num_riders;
}


/* =============================================================
   SUPPLEMENTARY ALGORITHMS
   ============================================================= */
#include "core.h"
#include <string.h>
#include <math.h>
#include <limits.h>
#include <stdint.h>
static int min_i(int a,int b) {
    return a<b?a:b;
}
static int max_i(int a,int b) {
    return a>b?a:b;
}
static double max_d(double a,double b) {
    return a>b?a:b;
}
static int bs_rec(const double *a,int l,int r,double x) {
    if(l>r)return r;
    int m=l+(r-l)/2;
    return a[m]<=x?bs_rec(a,m+1,r,x):bs_rec(a,l,m-1,x);
}
static void quick(double *a,int l,int r) {
    if(l>=r)return;
    double p=a[r];
    int k=l;
    for(int j=l;j<r;j++)if(a[j]<=p) {
        double t=a[j];
        a[j]=a[k];
        a[k++]=t;
    }
    a[r]=a[k];
    a[k]=p;
    quick(a,l,k-1);
    quick(a,k+1,r);
}
static double cross(const double *a,int l,int m,int r) {
    double s=0,b=-1e30,c=-1e30;
    for(int i=m;i>=l;i--) {
        s+=a[i];
        b=max_d(b,s);
    }
    s=0;
    for(int i=m+1;i<=r;i++) {
        s+=a[i];
        c=max_d(c,s);
    }
    return b+c;
}
static double maxsub(const double *a,int l,int r) {
    if(l==r)return a[l];
    int m=(l+r)/2;
    return max_d(max_d(maxsub(a,l,m),maxsub(a,m+1,r)),cross(a,l,m,r));
}
static long long power_mod(long long a,long long n,long long mod) {
    long long v=1;
    for(;n;n>>=1,a=a*a%mod)if(n&1)v=v*a%mod;
    return v;
}
static void mat_classic(const double a[2][2],const double b[2][2],double c[2][2]) {
    for(int i=0;i<2;i++)for(int j=0;j<2;j++) {
        c[i][j]=0;
        for(int k=0;k<2;k++)c[i][j]+=a[i][k]*b[k][j];
    }
}
static void mat_strassen(const double a[2][2],const double b[2][2],double c[2][2]) {
    double p1=(a[0][0]+a[1][1])*(b[0][0]+b[1][1]),p2=(a[1][0]+a[1][1])*b[0][0],p3=a[0][0]*(b[0][1]-b[1][1]),p4=a[1][1]*(b[1][0]-b[0][0]),p5=(a[0][0]+a[0][1])*b[1][1],p6=(a[1][0]-a[0][0])*(b[0][0]+b[0][1]),p7=(a[0][1]-a[1][1])*(b[1][0]+b[1][1]);
    c[0][0]=p1+p4-p5+p7;
    c[0][1]=p3+p5;
    c[1][0]=p2+p4;
    c[1][1]=p1-p2+p3+p6;
}
static int celebrity(const unsigned char *knows,int n) {
    int x=0;
    for(int i=1;i<n;i++)if(knows[x*n+i])x=i;
    for(int i=0;i<n;i++)if(i!=x&&(knows[x*n+i]||!knows[i*n+x]))return -1;
    return x;
}
static int activity(const Order *o,int n) {
    double ends[512],starts[512];
    if(n>512)n=512;
    for(int i=0;i<n;i++) {
        ends[i]=o[i].ready_time;
        starts[i]=o[i].ready_time-o[i].prep_time;
    }
    for(int i=1;i<n;i++) {
        double e=ends[i],s=starts[i];
        int j=i-1;
        while(j>=0&&ends[j]>e) {
            ends[j+1]=ends[j];
            starts[j+1]=starts[j];
            j--;
        }
        ends[j+1]=e;
        starts[j+1]=s;
    }
    int cnt=0;
    double last=-1e30;
    for(int i=0;i<n;i++)if(starts[i]>=last) {
        cnt++;
        last=ends[i];
    }
    return cnt;
}
static double fractional(const Order *o,int n,int cap) {
    double value=0;
    unsigned char *used=calloc((size_t)n,1);
    if(!used)return -1;
    while(cap>0) {
        int b=-1;
        double ratio=-1;
        for(int i=0;i<n;i++)if(!used[i]&&o[i].weight_kg>0) {
            double r=(double)o[i].urgency_score/o[i].weight_kg;
            if(r>ratio) {
                ratio=r;
                b=i;
            }
        }
        if(b<0)break;
        used[b]=1;
        int take=min_i(cap,o[b].weight_kg);
        value+=take*ratio;
        cap-=take;
    }
    free(used);
    return value;
}
static int huffman(const Order *o,int n,int *original) {
    int f[8]= {
        0
    },nodes[16],len=0,weighted=0;
    for(int i=0;i<n;i++) {
        int symbol=(o[i].service_status==1?0:4)+(o[i].weight_kg%4);
        f[symbol]++;
    }
    for(int i=0;i<8;i++)if(f[i])nodes[len++]=f[i];
    *original=n*8;
    if(len==1)return n;
    while(len>1) {
        int a=0,b=1;
        if(nodes[b]<nodes[a]) {
            int t=a;
            a=b;
            b=t;
        }
        for(int i=2;i<len;i++)if(nodes[i]<nodes[a]) {
            b=a;
            a=i;
        }else if(nodes[i]<nodes[b])b=i;
        int sum=nodes[a]+nodes[b];
        weighted+=sum;
        if(a>b) {
            int t=a;
            a=b;
            b=t;
        }
        nodes[b]=nodes[len-1];
        nodes[a]=sum;
        len--;
    }
    return weighted;
}
static int lcs(const char *a,const char *b) {
    int n=(int)strlen(a),m=(int)strlen(b);
    int *dp=calloc((size_t)(n+1)*(m+1),sizeof(int));
    if(!dp)return -1;
    for(int i=1;i<=n;i++)for(int j=1;j<=m;j++)dp[i*(m+1)+j]=a[i-1]==b[j-1]?dp[(i-1)*(m+1)+j-1]+1:max_i(dp[(i-1)*(m+1)+j],dp[i*(m+1)+j-1]);
    int v=dp[n*(m+1)+m];
    free(dp);
    return v;
}
static int mcm(const int *p,int n) {
    int dp[8][8]= {
        {
            0
        }
    };
    for(int len=2;len<=n;len++)for(int i=0;i+len<=n;i++) {
        int j=i+len-1;
        dp[i][j]=INT_MAX;
        for(int k=i;k<j;k++) {
            int v=dp[i][k]+dp[k+1][j]+p[i]*p[k+1]*p[j+1];
            if(v<dp[i][j])dp[i][j]=v;
        }
    }
    return dp[0][n-1];
}
static double obst(const double *p,int n) {
    double dp[8][8]= {
        {
            0
        }
    };
    for(int len=1;len<=n;len++)for(int i=0;i+len<=n;i++) {
        int j=i+len-1;
        double sum=0,b=1e30;
        for(int k=i;k<=j;k++)sum+=p[k];
        for(int k=i;k<=j;k++) {
            double v=(k>i?dp[i][k-1]:0)+(k<j?dp[k+1][j]:0)+sum;
            if(v<b)b=v;
        }
        dp[i][j]=b;
    }
    return dp[0][n-1];
}
static void queens(int row,int n,unsigned cols,unsigned d1,unsigned d2,int *cnt) {
    if(row==n) {
        (*cnt)++;
        return;
    }
    unsigned avail=((1u<<n)-1)&~(cols|d1|d2);
    while(avail) {
        unsigned bit=avail&(~avail+1u);
        avail-=bit;
        queens(row+1,n,cols|bit,(d1|bit)<<1,(d2|bit)>>1,cnt);
    }
}
static void subsets(const int *a,int n,int i,int sum,int target,int *cnt) {
    if(sum==target) {
        (*cnt)++;
        return;
    }
    if(i==n||sum>target)return;
    subsets(a,n,i+1,sum+a[i],target,cnt);
    subsets(a,n,i+1,sum,target,cnt);
}
static int naive(const char *s,const char *p) {
    int n=(int)strlen(s),m=(int)strlen(p),c=0;
    for(int i=0;i+m<=n;i++)if(memcmp(s+i,p,(size_t)m)==0)c++;
    return c;
}
static int rabin(const char *s,const char *p) {
    int n=(int)strlen(s),m=(int)strlen(p),cnt=0;
    if(!m||m>n)return 0;
    const long long mod=1000000007LL,base=257;
    long long pow=1,h=0,t=0;
    for(int i=0;i<m;i++) {
        h=(h*base+(unsigned char)p[i])%mod;
        t=(t*base+(unsigned char)s[i])%mod;
        if(i<m-1)pow=pow*base%mod;
    }
    for(int i=0;i+m<=n;i++) {
        if(h==t&&!memcmp(s+i,p,(size_t)m))cnt++;
        if(i+m<n) {
            t=((t-(unsigned char)s[i]*pow%mod+mod)*base+(unsigned char)s[i+m])%mod;
        }
    }
    return cnt;
}
static int kmp(const char *s,const char *p) {
    int m=(int)strlen(p),cnt=0;
    if(!m)return 0;
    int *pi=calloc((size_t)m,sizeof(int));
    if(!pi)return -1;
    for(int i=1,j=0;i<m;i++) {
        while(j&&p[i]!=p[j])j=pi[j-1];
        if(p[i]==p[j])j++;
        pi[i]=j;
    }
    for(int i=0,j=0;s[i];i++) {
        while(j&&s[i]!=p[j])j=pi[j-1];
        if(s[i]==p[j])j++;
        if(j==m) {
            cnt++;
            j=pi[j-1];
        }
    }
    free(pi);
    return cnt;
}
/* DFA string matching with alphabet of 256 bytes and explicit transition table. */
static int automaton(const char *s,const char *p) {
    int m=(int)strlen(p),cnt=0;
    if(m==0||m>100)return -1;
    int (*d)[256]=calloc((size_t)(m+1),sizeof(*d));
    if(!d)return -1;
    for(int state=0;state<=m;state++)for(int c=0;c<256;c++) {
        int k=min_i(m,state+1);
        for(;k>0;k--) {
            int ok=1;
            for(int j=0;j<k;j++) {
                int pos=state+1-k+j;
                int v=pos==state?c:(pos<state?(unsigned char)p[pos]:-1);
                if(v!=(unsigned char)p[j]) {
                    ok=0;
                    break;
                }
            }
            if(ok)break;
        }
        d[state][c]=k;
    }
    int state=0;
    for(int i=0;s[i];i++) {
        state=d[state][(unsigned char)s[i]];
        if(state==m)cnt++;
    }
    free(d);
    return cnt;
}
static long long crt(long long a,long long m,long long b,long long n) {
    for(long long x=a;x<m*n;x+=m)if(x%n==b%n)return x;
    return -1;
}
/* Edmonds-Karp max flow, derived from road travel-time capacity. */
static double flow(const CityMap *map) {
    int n=map->num_nodes;
    double r[MAX_NODES][MAX_NODES]= {
        {
            0
        }
    },ans=0;
    for(int i=0;i<n;i++)for(int j=0;j<n;j++)if(i!=j&&map->adj_matrix[i][j]<INF_TIME)r[i][j]=30.0/map->adj_matrix[i][j];
    int sink=n-1;
    for(;;) {
        int parent[MAX_NODES],q[MAX_NODES],h=0,t=0;
        for(int i=0;i<n;i++)parent[i]=-1;
        parent[0]=-2;
        q[t++]=0;
        while(h<t&&parent[sink]==-1) {
            int u=q[h++];
            for(int v=0;v<n;v++)if(parent[v]==-1&&r[u][v]>1e-9) {
                parent[v]=u;
                q[t++]=v;
            }
        }
        if(parent[sink]==-1)break;
        double add=1e30;
        for(int v=sink;v!=0;v=parent[v])if(r[parent[v]][v]<add)add=r[parent[v]][v];
        for(int v=sink;v!=0;v=parent[v]) {
            r[parent[v]][v]-=add;
            r[v][parent[v]]+=add;
        }
        ans+=add;
    }
    return ans;
}
static int augment(int rider,int n,const unsigned char *edge,int *match,unsigned char *seen) {
    for(int j=0;j<n;j++)if(edge[rider*n+j]&&!seen[j]) {
        seen[j]=1;
        if(match[j]<0||augment(match[j],n,edge,match,seen)) {
            match[j]=rider;
            return 1;
        }
    }
    return 0;
}
static int matching(const Order *o,int n,int riders) {
    int jobs=min_i(n,32),r=min_i(riders,32),ans=0;
    if(jobs<=0||r<=0)return 0;
    unsigned char edge[32*32]= {
        0
    };
    int match[32];
    for(int j=0;j<jobs;j++) {
        match[j]=-1;
        for(int i=0;i<r;i++)edge[i*jobs+j]=(o[j].service_status==1 && o[j].assigned_rider==i+1);
    }
    for(int i=0;i<r;i++) {
        unsigned char seen[32]= {
            0
        };
        ans+=augment(i,jobs,edge,match,seen);
    }
    return ans;
}
/* Held-Karp open tour over kitchen and up to four customer sectors. */
static double held_karp(const CityMap *map) {
    int n=min_i(map->num_nodes-1,4),size=1<<n;
    double dp[16][4];
    for(int s=0;s<size;s++)for(int j=0;j<n;j++)dp[s][j]=INF_TIME;
    for(int j=0;j<n;j++)dp[1<<j][j]=map->shortest_dist[0][j+1];
    for(int s=1;s<size;s++)for(int j=0;j<n;j++)if(s&(1<<j))for(int k=0;k<n;k++)if(!(s&(1<<k))) {
        double v=dp[s][j]+map->shortest_dist[j+1][k+1];
        if(v<dp[s|(1<<k)][k])dp[s|(1<<k)][k]=v;
    }
    double best=INF_TIME;
    for(int j=0;j<n;j++)if(dp[size-1][j]<best)best=dp[size-1][j];
    return best;
}
/* Backtracking exact knapsack and three distinct branch-and-bound frontier policies. */
static void knapsack_bt(const int *w,const int *v,int n,int i,int cap,int score,int *best) {
    if(i==n) {
        if(score>*best)*best=score;
        return;
    }
    knapsack_bt(w,v,n,i+1,cap,score,best);
    if(w[i]<=cap)knapsack_bt(w,v,n,i+1,cap-w[i],score+v[i],best);
}
typedef struct {
    int i,cap,value,bound;
}
KNode;
static int knap_bound(const int *v,int n,int i,int score) {
    int b=score;
    for(int j=i;j<n;j++)b+=v[j];
    return b;
}
static int knapsack_bnb(const int *w,const int *v,int n,int cap,int policy) {
    KNode frontier[2048];
    int len=1,best=0;
    frontier[0]=(KNode) {
        0,cap,0,knap_bound(v,n,0,0)
    };
    while(len) {
        int ix=policy==0?0:len-1;
        if(policy==2)for(int j=1;j<len;j++)if(frontier[j].bound>frontier[ix].bound)ix=j;
        KNode cur=frontier[ix];
        for(int j=ix;j<len-1;j++)frontier[j]=frontier[j+1];
        len--;
        if(cur.bound<=best)continue;
        if(cur.i==n) {
            if(cur.value>best)best=cur.value;
            continue;
        }
        int k=cur.i;
        KNode exclude= {
            k+1,cur.cap,cur.value,knap_bound(v,n,k+1,cur.value)
        };
        if(exclude.bound>best&&len<2048)frontier[len++]=exclude;
        if(w[k]<=cur.cap) {
            KNode include= {
                k+1,cur.cap-w[k],cur.value+v[k],knap_bound(v,n,k+1,cur.value+v[k])
            };
            if(include.value>best)best=include.value;
            if(include.bound>best&&len<2048)frontier[len++]=include;
        }
    }
    return best;
}
void daa_extra_diagnostics(const CityMap *map,const Order *orders,int n,int riders,SyllabusExtra *out) {
    memset(out,0,sizeof(*out));
    if(!map||!orders||n<=0)return;
    double *a=malloc((size_t)n*sizeof(double));
    if(!a)return;
    for(int i=0;i<n;i++)a[i]=orders[i].ready_time;
    out->binary_recursive=bs_rec(a,0,n-1,25);
    quick(a,0,n-1);
    out->quicksort_ok=1;
    for(int i=1;i<n;i++)if(a[i]<a[i-1])out->quicksort_ok=0;
    free(a);
    double changes[64];
    int count=min_i(n,64);
    for(int i=0;i<count;i++)changes[i]=orders[i].due_time-orders[i].delivered_time;
    out->max_subarray=maxsub(changes,0,count-1);
    out->fast_power=power_mod(7,31,1000000007LL);
    unsigned char knows[5][5]= {
        {
            0
        }
    };
    for(int i=0;i<5;i++)if(i!=2)knows[i][2]=1;
    out->celebrity=celebrity(&knows[0][0],5);
    double A[2][2]= {
        {
            1,2
        }, {
            3,4
        }
    },B[2][2]= {
        {
            5,6
        }, {
            7,8
        }
    },C[2][2],D[2][2];
    mat_classic(A,B,C);
    mat_strassen(A,B,D);
    for(int i=0;i<2;i++)for(int j=0;j<2;j++)out->strassen_error+=fabs(C[i][j]-D[i][j]);
    out->activity_count=activity(orders,n);
    out->fractional_value=fractional(orders,n,12);
    out->huffman_bits=huffman(orders,n,&out->huffman_original_bits);
    out->lcs=lcs("DELIVERY","VERY");
    int dims[]= {
        10,20,5,15,10
    };
    out->mcm=mcm(dims,4);
    double probs[]= {
        .4,.3,.2,.1
    };
    out->obst_cost=obst(probs,4);
    queens(0,8,0,0,0,&out->nqueens);
    int weights[10];
    int k=min_i(n,10);
    for(int i=0;i<k;i++)weights[i]=orders[i].weight_kg;
    subsets(weights,k,0,0,12,&out->subset_matches);
    const char *events="DOON DELIVERY DOON DELIVERY DOON";
    out->naive_hits=naive(events,"DOON");
    out->rabin_hits=rabin(events,"DOON");
    out->kmp_hits=kmp(events,"DOON");
    out->automaton_hits=automaton(events,"DOON");
    out->crt_result=crt(2,3,3,5);
    int kw[8],kv[8],kn=min_i(n,8);
    for(int i=0;i<kn;i++) {
        kw[i]=orders[i].weight_kg;
        kv[i]=orders[i].urgency_score;
    }
    knapsack_bt(kw,kv,kn,0,12,0,&out->bt_knapsack_best);
    out->bnb_fifo_best=knapsack_bnb(kw,kv,kn,12,0);
    out->bnb_lifo_best=knapsack_bnb(kw,kv,kn,12,1);
    out->bnb_lc_best=knapsack_bnb(kw,kv,kn,12,2);
    out->max_flow=flow(map);
    out->matched_riders=matching(orders,n,riders);
    out->tsp_cost=held_karp(map);
    out->tsp_cost_valid=out->tsp_cost<INF_TIME;
}