#ifdef _WIN32
    #define _WIN32_WINNT 0x0601
    #include <winsock2.h>
    #include <ws2tcpip.h>
    typedef SOCKET sock_t;
    #define CLOSE_SOCK closesocket
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    typedef int sock_t;
    #define CLOSE_SOCK close
#endif

#include "core.h"

#include <string.h>
#include <time.h>

int build_simulation_json(char* buf, int max_len, const CityMap* map, const Order* orders, int count, double exec_ms, int stoves, int riders, int total_pruned, const EnterpriseDiagnostics* diag) {
    int pos = 0, i, u, v, on_time = 0, served = 0, unserved = 0, reachable_cnt = 0;
    double total_wait = 0.0, total_late = 0.0;

    for (i = 0; i < map->num_nodes; ++i) if (map->bfs_reachable[i]) reachable_cnt++;
    for (i = 0; i < count; ++i) {
        if (orders[i].service_status < 0) {
            unserved++;
            continue;
        }
        if (orders[i].service_status == 1) {
            served++;
            total_wait += (orders[i].delivered_time - orders[i].arrival_time);
            if (orders[i].delivered_time <= orders[i].due_time) on_time++;
            else total_late += (orders[i].delivered_time - orders[i].due_time);
        }
    }

    pos += snprintf(buf + pos, max_len - pos,
        "{\"metrics\":{\"orders\":%d,\"served\":%d,\"unserved\":%d,\"exec_ms\":%.3f,\"avg_wait\":%.2f,\"on_time_pct\":%.1f,\"tardiness\":%.2f,\"stoves\":%d,\"riders\":%d,\"pruned\":%d,\"bfs_nodes\":%d},"
        "\"diag\":{\"surge_score\":%.1f,\"min_stoves_opt\":%d,\"prim_cost\":%.1f,\"kruskal_cost\":%.1f,\"huff_saved_pct\":%.1f,\"max_flow\":%.1f,\"bipartite\":%d,\"bellman_bidholi\":%.1f,\"nqueens\":%d,\"subset_matches\":%d,\"kmp_hits\":%d,\"crt_sync\":%lld,\"obst_cost\":%.2f,\"mcm_ops\":%d},\"mst_edges\":[",
        count, served, unserved, exec_ms, served ? total_wait / served : 0.0,
        count ? (100.0 * on_time) / count : 0.0, total_late, stoves, riders,
        total_pruned, reachable_cnt, diag->max_subarray_surge,
        diag->min_stoves_interval_part, diag->prim_mst_cost, diag->kruskal_mst_cost,
        diag->huffman_orig_bits > 0 ? 100.0 * (1.0 - (double)diag->huffman_comp_bits / diag->huffman_orig_bits) : -1.0,
        diag->ford_fulkerson_max_flow, diag->bipartite_matched_riders,
        diag->bellman_ford_dist, diag->nqueens_solutions, diag->subset_sum_matches,
        diag->str_kmp_hits, diag->crt_sync_timestamp, diag->obst_min_cost, diag->mcm_min_ops);

    for (i = 0; i < diag->mst_edge_count; ++i) {
        pos += snprintf(buf + pos, max_len - pos, "%s{\"u\":%d,\"v\":%d}",
            (i == 0) ? "" : ",", diag->mst_edge_u[i], diag->mst_edge_v[i]);
    }

    pos += snprintf(buf + pos, max_len - pos, "],\"edges\":[");
    int first_edge = 1;
    for (u = 0; u < map->num_nodes; ++u) {
        for (v = u + 1; v < map->num_nodes; ++v) {
            if (map->adj_matrix[u][v] < INF_TIME) {
                pos += snprintf(buf + pos, max_len - pos, "%s{\"u\":%d,\"v\":%d,\"w\":%.1f}",
                    first_edge ? "" : ",", u, v, map->adj_matrix[u][v]);
                first_edge = 0;
            }
        }
    }

    pos += snprintf(buf + pos, max_len - pos, "],\"orders\":[");
    for (i = 0; i < count; ++i) {
        pos += snprintf(buf + pos, max_len - pos,
            "%s{\"id\":%d,\"node\":%d,\"kg\":%d,\"val\":%d,\"arr\":%.1f,\"prep\":%.1f,\"ready\":%.1f,\"disp\":%.1f,\"deliv\":%.1f,\"due\":%.1f,\"stove\":%d,\"rider\":%d,\"batch\":%d,\"pruned\":%d,\"served\":%d,\"on_time\":%d}",
            (i == 0) ? "" : ",",
            orders[i].order_id, orders[i].customer_node, orders[i].weight_kg, orders[i].urgency_score,
            orders[i].arrival_time, orders[i].prep_time, orders[i].ready_time, orders[i].dispatch_time,
            orders[i].delivered_time, orders[i].due_time, orders[i].assigned_stove,
            orders[i].assigned_rider, orders[i].batch_id, orders[i].bb_nodes_pruned,
            (orders[i].service_status == 1) ? 1 : 0,
            (orders[i].service_status == 1 && orders[i].delivered_time <= orders[i].due_time) ? 1 : 0);
    }
    pos += snprintf(buf + pos, max_len - pos, "]}");
    return pos;
}

void serve_html_file(sock_t client) {
    FILE* fp = fopen("dashboard.html", "rb");
    if (!fp) return;
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char header[256];
    snprintf(header, sizeof(header), "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\nContent-Length: %ld\r\nConnection: close\r\n\r\n", fsize);
    send(client, header, (int)strlen(header), 0);
    char* file_buf = (char*)malloc(fsize);
    fread(file_buf, 1, fsize, fp);
    { long sent = 0; while (sent < fsize) { int k = send(client, file_buf + sent, (int)(fsize - sent), 0); if (k <= 0) break; sent += k; } }
    free(file_buf);
    fclose(fp);
}

static void serve_offline_charts(sock_t client) {
    FILE *fp=fopen("offline_charts.js","rb");
    if(!fp){const char *e="HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";send(client,e,(int)strlen(e),0);return;}
    fseek(fp,0,SEEK_END);long size=ftell(fp);rewind(fp);
    char hdr[256];snprintf(hdr,sizeof(hdr),"HTTP/1.1 200 OK\r\nContent-Type: application/javascript\r\nContent-Length: %ld\r\nConnection: close\r\n\r\n",size);
    send(client,hdr,(int)strlen(hdr),0);
    char b[4096];size_t nr;while((nr=fread(b,1,sizeof(b),fp))>0){size_t sent=0;while(sent<nr){int k=send(client,b+sent,(int)(nr-sent),0);if(k<=0){fclose(fp);return;}sent+=(size_t)k;}}
    fclose(fp);
}
int main(void) {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    CityMap city;
    generate_sample_city(&city);
    daa_bfs_connectivity(&city, 0);
    daa_floyd_warshall(&city);

    sock_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("[ERROR] Port 8080 busy! Close previous webapp.exe first.\n");
        return 1;
    }
    listen(server_fd, 10);
    printf("====================================================================\n");
    printf("  DOONEATS EXPRESS (DEHRADUN) LIVE AT: http://localhost:8080\n");
    printf("  100%% Syllabus Modules Linked & Active!\n");
    printf("====================================================================\n");

    char* json_buf = (char*)malloc(1024 * 1024);
    if (!json_buf) return 1;
    while (1) {
        sock_t client = accept(server_fd, NULL, NULL);
        if (client < 0) continue;
        char req[2048] = {0};
        recv(client, req, sizeof(req) - 1, 0);

        if (strncmp(req, "GET /api/simulate", 17) == 0) {
            int n = 30, stoves = 5, riders = 6, batch = 4, bag_kg = 12, urg_node = -1;
            char* q = strchr(req, '?');
            if (q) {
                char query[1024];
                char* token;
                snprintf(query, sizeof(query), "%s", q + 1);
                token = strtok(query, "&");
                while (token) {
                    int val;
                    if (sscanf(token, "n=%d", &val) == 1) n = val;
                    else if (sscanf(token, "stoves=%d", &val) == 1) stoves = val;
                    else if (sscanf(token, "riders=%d", &val) == 1) riders = val;
                    else if (sscanf(token, "batch=%d", &val) == 1) batch = val;
                    else if (sscanf(token, "bag=%d", &val) == 1) bag_kg = val;
                    else if (sscanf(token, "unode=%d", &val) == 1) urg_node = val;
                    token = strtok(NULL, "&");
                }
            }
            if (n < 5) n = 5;
            if (n > 500) n = 500;
            if (stoves < 1) stoves = 1;
            if (stoves > 25) stoves = 25;
            if (riders < 1) riders = 1;
            if (riders > 35) riders = 35;
            if (batch < 1) batch = 1;
            if (batch > 5) batch = 5;
            if (bag_kg < 1) bag_kg = 1;
            if (bag_kg > 24) bag_kg = 24;

            int total_n = (urg_node > 0) ? n + 1 : n;
            Order* orders = allocate_and_generate_orders(total_n, city.num_nodes);
            if (urg_node > 0 && urg_node < city.num_nodes) {
                orders[total_n - 1].order_id = 999;
                orders[total_n - 1].customer_node = urg_node;
                orders[total_n - 1].weight_kg = 2;
                orders[total_n - 1].arrival_time = 4.0;
                orders[total_n - 1].prep_time = 6.0;
                orders[total_n - 1].due_time = 15.0;
            }

            clock_t t0 = clock();
            daa_schedule_kitchen(orders, total_n, &city, 0, stoves);
            int pruned = daa_dispatch_knapsack_bnb(orders, total_n, &city, 0, riders, bag_kg, batch);
            EnterpriseDiagnostics diag;
            daa_run_syllabus_diagnostics(&city, orders, total_n, riders, "Doon", &diag);
            SyllabusExtra extra;
            daa_extra_diagnostics(&city, orders, total_n, riders, &extra);
            diag.max_subarray_surge = extra.max_subarray;
            diag.huffman_orig_bits = extra.huffman_original_bits;
            diag.huffman_comp_bits = extra.huffman_bits;
            diag.ford_fulkerson_max_flow = extra.max_flow;
            diag.bipartite_matched_riders = extra.matched_riders;
            diag.nqueens_solutions = extra.nqueens;
            diag.subset_sum_matches = extra.subset_matches;
            diag.str_kmp_hits = extra.kmp_hits;
            diag.crt_sync_timestamp = extra.crt_result;
            diag.obst_min_cost = extra.obst_cost;
            diag.mcm_min_ops = extra.mcm;
            clock_t t1 = clock();

            double ms = ((double)(t1 - t0) / CLOCKS_PER_SEC) * 1000.0;
            export_csv_telemetry(orders, total_n, "dispatch_telemetry.csv");

            int len = build_simulation_json(json_buf, 1024 * 1024, &city, orders, total_n, ms, stoves, riders, pruned, &diag);
            char hdr[256];
            snprintf(hdr, sizeof(hdr), "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nConnection: close\r\n\r\n", len);
            send(client, hdr, (int)strlen(hdr), 0);
            { int sent = 0; while (sent < len) { int k = send(client, json_buf + sent, len - sent, 0); if (k <= 0) break; sent += k; } }
            free(orders);
        } else if (strncmp(req,"GET /offline_charts.js",22)==0) {
            serve_offline_charts(client);
        } else {
            serve_html_file(client);
        }
        CLOSE_SOCK(client);
    }
    return 0;
}
