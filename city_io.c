#include "core.h"

void init_city_map(CityMap* map, int nodes) {
    int i, j;
    map->num_nodes = nodes;
    for (i = 0; i < nodes; ++i) {
        map->bfs_reachable[i] = 0;
        for (j = 0; j < nodes; ++j) {
            map->adj_matrix[i][j] = (i == j) ? 0.0 : INF_TIME;
            map->shortest_dist[i][j] = (i == j) ? 0.0 : INF_TIME;
            map->next_hop[i][j] = (i == j) ? i : -1;
        }
    }
}

static void add_road_c(CityMap* map, int u, int v, double minutes) {
    if (u >= 0 && u < map->num_nodes && v >= 0 && v < map->num_nodes) {
        map->adj_matrix[u][v] = minutes;
        map->adj_matrix[v][u] = minutes;
        map->next_hop[u][v] = v;
        map->next_hop[v][u] = u;
    }
}

void generate_sample_city(CityMap* map) {
    int i, u, v;
    srand(42);
    init_city_map(map, MAX_NODES);

    for (i = 1; i < map->num_nodes; ++i) {
        if (i <= 8) {
            add_road_c(map, 0, i, 2.5 + (rand() % 25) / 10.0);
        } else {
            int hub_link = (i % 8) + 1;
            add_road_c(map, hub_link, i, 3.0 + (rand() % 30) / 10.0);
        }
        if (i < map->num_nodes - 1) {
            add_road_c(map, i, i + 1, 2.0 + (rand() % 20) / 10.0);
        }
    }

    for (i = 0; i < map->num_nodes; ++i) {
        u = (rand() % (map->num_nodes - 1)) + 1;
        v = (rand() % (map->num_nodes - 1)) + 1;
        if (u != v) {
            add_road_c(map, u, v, 2.8 + (rand() % 35) / 10.0);
        }
    }
}

Order* allocate_and_generate_orders(int count, int num_nodes) {
    int i;
    Order* orders = (Order*)malloc(sizeof(Order) * count);
    if (!orders) {
        printf("[FATAL] Out of memory!\n");
        exit(1);
    }

    srand(101);
    for (i = 0; i < count; ++i) {
        double base_arr = i * 2.25;
        double jitter = ((rand() % 140) - 70) / 10.0; 
        double arr = base_arr + jitter;
        if (arr < 0.0) arr = (rand() % 30) / 10.0;

        double prep = 5.5 + (rand() % 65) / 10.0; 

        int tier_roll = rand() % 100;
        double sla_buffer;
        if (tier_roll < 72) {
            sla_buffer = 13.0 + (rand() % 75) / 10.0; 
        } else if (tier_roll < 90) {
            sla_buffer = 7.0 + (rand() % 45) / 10.0; 
        } else {
            sla_buffer = 3.0 + (rand() % 30) / 10.0; 
        }

        orders[i].order_id = i + 1;
        orders[i].customer_node = (rand() % (num_nodes - 1)) + 1;
        orders[i].weight_kg = 2 + (rand() % 4);
        orders[i].urgency_score = 50 + (rand() % 50);
        orders[i].arrival_time = arr;
        orders[i].prep_time = prep;
        orders[i].due_time = sla_buffer;
        orders[i].priority_slack = 0.0;
        orders[i].ready_time = 0.0;
        orders[i].dispatch_time = 0.0;
        orders[i].delivered_time = 0.0;
        orders[i].assigned_stove = 0;
        orders[i].assigned_rider = 0;
        orders[i].batch_id = 0;
        orders[i].bb_nodes_pruned = 0;
        orders[i].service_status = 0;
    }
    return orders;
}

void export_csv_telemetry(const Order* orders, int count, const char* filename) {
    int i;
    FILE* fp = fopen(filename, "w");
    if (!fp) return;
    fprintf(fp, "OrderID,Zone,WeightKG,Arrival,Prep,Ready,Dispatch,Delivered,PromisedSLA,Station,Rider,TripID,RoutesFiltered,Status\n");
    for (i = 0; i < count; ++i) {
        const char* st = (orders[i].service_status < 0) ? "UNSERVED_CAPACITY" : ((orders[i].delivered_time <= orders[i].due_time) ? "ON_TIME" : "DELAYED");
        fprintf(fp, "%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%s\n",
                orders[i].order_id, orders[i].customer_node, orders[i].weight_kg,
                orders[i].arrival_time, orders[i].prep_time, orders[i].ready_time,
                orders[i].dispatch_time, orders[i].delivered_time, orders[i].due_time,
                orders[i].assigned_stove, orders[i].assigned_rider, orders[i].batch_id,
                orders[i].bb_nodes_pruned, st);
    }
    fclose(fp);
}
