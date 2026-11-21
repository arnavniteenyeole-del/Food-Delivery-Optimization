
#include "core.h"

#include <time.h>
#include <math.h>
#include <string.h>

static int failures = 0;

#define CHECK(x, msg)                                      \
    do {                                                   \
        if (!(x)) {                                        \
            fprintf(stderr, "FAIL: %s\n", msg);            \
            failures++;                                    \
        }                                                  \
    } while (0)

static void run(
    CityMap *city,
    int n,
    int stations,
    int riders,
    int batch,
    int bag,
    int verbose
) {
    Order *o = allocate_and_generate_orders(n, city->num_nodes);
    clock_t t = clock();

    daa_schedule_kitchen(o, n, city, 0, stations);

    int pruned = daa_dispatch_knapsack_bnb(
        o, n, city, 0, riders, bag, batch
    );

    SyllabusExtra d;
    daa_extra_diagnostics(city, o, n, riders, &d);

    int served = 0;
    int ontime = 0;
    int unserved = 0;
    double wait = 0;

    for (int i = 0; i < n; i++) {
        if (o[i].service_status == 1) {
            served++;

            wait += o[i].delivered_time - o[i].arrival_time;
            ontime += (o[i].delivered_time <= o[i].due_time);

            CHECK(
                o[i].assigned_rider >= 1 &&
                o[i].assigned_rider <= riders,
                "rider id in bounds"
            );

            CHECK(o[i].batch_id > 0, "valid trip");

            CHECK(
                o[i].delivered_time >= o[i].ready_time,
                "delivery after preparation"
            );
        } else {
            unserved++;

            CHECK(
                o[i].weight_kg > bag,
                "only overweight orders are unserved"
            );
        }
    }

    CHECK(served + unserved == n, "accounting");
    CHECK(d.nqueens == 92, "8 queens count");
    CHECK(d.mcm == 2250, "matrix chain result");
    CHECK(d.lcs == 4, "LCS result");
    CHECK(d.crt_result == 8, "CRT result");

    CHECK(
        d.strassen_error < 1e-9,
        "Strassen equals classic"
    );

    CHECK(d.quicksort_ok, "quicksort sorted");

    CHECK(
        d.naive_hits == d.rabin_hits &&
        d.rabin_hits == d.kmp_hits &&
        d.kmp_hits == d.automaton_hits,
        "four string matchers agree"
    );

    CHECK(
        d.bt_knapsack_best == d.bnb_fifo_best &&
        d.bnb_fifo_best == d.bnb_lifo_best &&
        d.bnb_lifo_best == d.bnb_lc_best,
        "backtracking and FIFO/LIFO/LC branch-and-bound agree"
    );

    CHECK(d.max_flow >= 0, "nonnegative maxflow");
    CHECK(d.tsp_cost_valid, "finite TSP");

    if (verbose) {
        printf(
            "%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.3f,%d\n",
            n,
            stations,
            riders,
            batch,
            bag,
            served,
            100.0 * ontime / n,
            served ? wait / served : 0,
            1000.0 * (clock() - t) / CLOCKS_PER_SEC,
            pruned
        );
    }

    free(o);
}

int main(void) {
    CityMap city;

    generate_sample_city(&city);
    daa_bfs_connectivity(&city, 0);
    daa_floyd_warshall(&city);

    for (int i = 0; i < city.num_nodes; i++) {
        CHECK(city.bfs_reachable[i], "graph connected");
    }

    printf(
        "orders,stations,riders,max_drops,bag_kg,"
        "served,on_time_pct,avg_wait_min,elapsed_ms,pruned\n"
    );

    run(&city, 30, 5, 6, 4, 12, 1);
    run(&city, 30, 5, 6, 1, 12, 1);
    run(&city, 60, 2, 2, 4, 12, 1);
    run(&city, 150, 5, 6, 4, 12, 1);
    run(&city, 400, 5, 6, 4, 12, 1);
    run(&city, 30, 5, 6, 4, 1, 1);
    run(&city, 30, 1, 1, 1, 5, 1);

    fprintf(
        stderr,
        "TEST RESULT: %s (%d failures)\n",
        failures ? "FAIL" : "PASS",
        failures
    );

    return failures ? 1 : 0;
}
