#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NODES 25
#define INF_TIME 1e9

typedef struct {
    int order_id;
    int customer_node;
    int weight_kg;
    int urgency_score;
    double arrival_time;
    double prep_time;
    double due_time;
    double priority_slack;
    double ready_time;
    double dispatch_time;
    double delivered_time;
    int assigned_stove;
    int assigned_rider;
    int batch_id;
    int bb_nodes_pruned;
    int service_status;
} Order;

typedef struct {
    int rider_id;
    int current_node;
    double available_time;
    int total_deliveries;
} Rider;

typedef struct {
    int num_nodes;
    double adj_matrix[MAX_NODES][MAX_NODES];
    double shortest_dist[MAX_NODES][MAX_NODES];
    int next_hop[MAX_NODES][MAX_NODES];
    int bfs_reachable[MAX_NODES];
} CityMap;

typedef struct {
    int bsearch_iter_idx;
    int bsearch_rec_idx;
    double max_subarray_surge;
    long long fast_pow_hash;
    int celebrity_hub_id;
    double strassen_checksum;

    double fractional_knapsack_val;
    int activity_selection_count;
    int min_stoves_interval_part;
    int huffman_orig_bits;
    int huffman_comp_bits;
    double prim_mst_cost;
    double kruskal_mst_cost;
    int mst_edge_u[MAX_NODES];
    int mst_edge_v[MAX_NODES];
    int mst_edge_count;

    int mcm_min_ops;
    int lcs_similarity_len;
    double obst_min_cost;
    double bellman_ford_dist;
    double held_karp_tsp_cost;

    int nqueens_solutions;
    int subset_sum_matches;

    double ford_fulkerson_max_flow;
    int bipartite_matched_riders;
    long long crt_sync_timestamp;
    int str_naive_hits;
    int str_rabin_karp_hits;
    int str_automaton_hits;
    int str_kmp_hits;
} EnterpriseDiagnostics;

void init_city_map(CityMap* map, int nodes);
void generate_sample_city(CityMap* map);
Order* allocate_and_generate_orders(int count, int num_nodes);
void export_csv_telemetry(const Order* orders, int count, const char* filename);

void daa_bfs_connectivity(CityMap* map, int start_node);
void daa_dijkstra(const CityMap* map, int source, double dist[MAX_NODES]);
void daa_floyd_warshall(CityMap* map);
void daa_merge_sort_orders(Order* arr, int left, int right, int sort_by_ready);
void daa_schedule_kitchen(Order* orders, int count, const CityMap* map, int kitchen_node, int num_stoves);
int  daa_dispatch_knapsack_bnb(Order* orders, int count, const CityMap* map, int kitchen_node, int num_riders, int max_bag_kg, int max_batch);

void daa_run_syllabus_diagnostics(const CityMap* map, const Order* orders, int count, int num_riders, const char* search_query, EnterpriseDiagnostics* out);

#ifdef __cplusplus
}
#endif

typedef struct {
 int binary_recursive, quicksort_ok, celebrity, lcs, mcm, nqueens, subset_matches;
 int kmp_hits, naive_hits, rabin_hits, automaton_hits, matched_riders;
 int bt_knapsack_best, bnb_fifo_best, bnb_lifo_best, bnb_lc_best;
 int activity_count, huffman_bits, huffman_original_bits, tsp_cost_valid;
 long long fast_power, crt_result;
 double max_subarray, strassen_error, fractional_value, max_flow, tsp_cost, obst_cost;
} SyllabusExtra;
void daa_extra_diagnostics(const CityMap *map,const Order *orders,int n,int riders,SyllabusExtra *out);

#endif
