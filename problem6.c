#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_NODES 10000
#define INF 1e15
#define PI 3.14159265358979323846

typedef struct {
    double lat, lon;
} Coord;

typedef struct {
    int to;
    double dist;
    double cost_rate;
    double speed;
    double interval;
    int start_h, end_h;
    char mode[50];
} Edge;

Coord nodes[MAX_NODES];
Edge *adj[MAX_NODES];
int adj_size[MAX_NODES], adj_cap[MAX_NODES];
int node_count = 0;

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
    return 6371.0 * 2 * atan2(sqrt(a), sqrt(1 - a));
}

int get_node_id(double lat, double lon) {
    for (int i = 0; i < node_count; i++)
        if (fabs(nodes[i].lat - lat) < 1e-7 && fabs(nodes[i].lon - lon) < 1e-7) return i;
    nodes[node_count].lat = lat; nodes[node_count].lon = lon;
    adj_size[node_count] = 0; adj_cap[node_count] = 20;
    adj[node_count] = (Edge *)malloc(20 * sizeof(Edge));
    return node_count++;
}

void add_edge(int u, int v, double d, double rate, double speed, double interval, int sh, int eh, char *mode) {
    if (adj_size[u] >= adj_cap[u]) {
        adj_cap[u] *= 2;
        adj[u] = (Edge *)realloc(adj[u], adj_cap[u] * sizeof(Edge));
    }
    Edge *e = &adj[u][adj_size[u]++];
    e->to = v; e->dist = d; e->cost_rate = rate; e->speed = speed;
    e->interval = interval; e->start_h = sh; e->end_h = eh; strcpy(e->mode, mode);
}

void format_time(double mins, char *buf) {
    int h = ((int)(mins / 60)) % 24;
    int m = (int)fmod(mins, 60);
    char *period = (h >= 12) ? "PM" : "AM";
    int h12 = (h % 12 == 0) ? 12 : h % 12;
    sprintf(buf, "%02d:%02d %s", h12, m, period);
}

void load_data() {
    FILE *fp; char line[5000], *token;
    // 1. Car: 20 tk/km, 20 km/h, Instant
    fp = fopen("Roadmap-Dhaka.csv", "r");
    if(fp) {
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double c_vals[150]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) c_vals[c++] = atof(token);
            for (int i = 0; i < c - 4; i += 2) {
                int u = get_node_id(c_vals[i+1], c_vals[i]); int v = get_node_id(c_vals[i+3], c_vals[i+2]);
                add_edge(u, v, haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon), 20.0, 20.0, 0, 0, 24, "Car");
                add_edge(v, u, haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon), 20.0, 20.0, 0, 0, 24, "Car");
            }
        } fclose(fp);
    }
    // 2. Metro: 5 tk/km, 15 km/h, 5 min interval, 1 AM - 11 PM
    // 3. Bikalpa Bus: 7 tk/km, 10 km/h, 20 min interval, 7 AM - 10 PM
    // 4. Uttara Bus: 10 tk/km, 12 km/h, 10 min interval, 6 AM - 11 PM
    char *f[] = {"Routemap-DhakaMetroRail.csv", "Routemap-BikolpoBus.csv", "Routemap-UttaraBus.csv"};
    char *m[] = {"Metro", "Bikalpa Bus", "Uttara Bus"};
    double cr[] = {5.0, 7.0, 10.0}, sp[] = {15.0, 10.0, 12.0}, inv[] = {5.0, 20.0, 10.0};
    int sh[] = {1, 7, 6}, eh[] = {23, 22, 23};
    for(int i=0; i<3; i++) {
        fp = fopen(f[i], "r"); if(!fp) continue;
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double c_vals[1000]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) { if (atof(token) == 0 && c > 2) break; c_vals[c++] = atof(token); }
            for (int j = 0; j < c - 3; j += 2) {
                int u = get_node_id(c_vals[j+1], c_vals[j]); int v = get_node_id(c_vals[j+3], c_vals[j+2]);
                add_edge(u, v, haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon), cr[i], sp[i], inv[i], sh[i], eh[i], m[i]);
            }
        } fclose(fp);
    }
}

double calculate_wait(double curr, double interval, int sh, int eh) {
    if (interval == 0) return 0; // Car
    if (curr < sh * 60) return (sh * 60) - curr;
    if (curr > eh * 60) return INF;
    return fmod(interval - fmod(curr, interval), interval);
}

void solve_problem6(double sLat, double sLon, double dLat, double dLon, int sh, int sm, int dh, int dm) {
    load_data();
    double start_time = sh * 60.0 + sm, deadline = dh * 60.0 + dm;
    int start_node = 0, end_node = 0; double min_s = INF, min_e = INF;
    for(int i=0; i<node_count; i++) {
        double d1 = haversine(sLat, sLon, nodes[i].lat, nodes[i].lon); if(d1 < min_s) { min_s = d1; start_node = i; }
        double d2 = haversine(dLat, dLon, nodes[i].lat, nodes[i].lon); if(d2 < min_e) { min_e = d2; end_node = i; }
    }

    double min_cost[MAX_NODES], time_at[MAX_NODES]; int prev[MAX_NODES], visited[MAX_NODES] = {0};
    for(int i=0; i<MAX_NODES; i++) { min_cost[i] = INF; prev[i] = -1; }
    
    // Case C: Walk to nearest node (2km/h, 0 cost)
    double initial_walk_time = (min_s / 2.0) * 60.0;
    min_cost[start_node] = 0;
    time_at[start_node] = start_time + initial_walk_time;

    for(int i=0; i<node_count; i++) {
        int u = -1;
        for(int j=0; j<node_count; j++) if(!visited[j] && (u == -1 || min_cost[j] < min_cost[u])) u = j;
        if(u == -1 || min_cost[u] == INF) break; visited[u] = 1;

        for(int k=0; k<adj_size[u]; k++) {
            Edge e = adj[u][k];
            double wait = calculate_wait(time_at[u], e.interval, e.start_h, e.end_h);
            double travel = (e.dist / e.speed) * 60.0;
            double arrival = time_at[u] + wait + travel;
            if(arrival <= deadline && min_cost[u] + (e.dist * e.cost_rate) < min_cost[e.to]) {
                min_cost[e.to] = min_cost[u] + (e.dist * e.cost_rate);
                time_at[e.to] = arrival;
                prev[e.to] = u;
            }
        }
    }

    if(min_cost[end_node] == INF) { printf("No route found within deadline!\n"); return; }

    FILE *txt = fopen("problem6_directions.txt", "w");
    FILE *kml = fopen("problem6.kml", "w");
    fprintf(kml, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document><Placemark><LineString><coordinates>%f,%f,0\n", sLon, sLat);
    
    char t1[20], t2[20];
    format_time(start_time, t1); format_time(time_at[start_node], t2);
    fprintf(txt, "%s - %s, Cost: BDT 0.00: Walk from Source (%f, %f) to (%f, %f).\n\n", t1, t2, sLon, sLat, nodes[start_node].lon, nodes[start_node].lat);

    int path[MAX_NODES], p_count = 0, curr = end_node;
    while(curr != -1) { path[p_count++] = curr; curr = prev[curr]; }

    double cur_t = time_at[start_node];
    for(int i = p_count - 1; i > 0; i--) {
        int u = path[i], v = path[i-1]; double d, cr, wait, speed; char *m; int sh_e, eh_e; double inv;
        for(int k=0; k<adj_size[u]; k++) if(adj[u][k].to == v) { 
            d=adj[u][k].dist; cr=adj[u][k].cost_rate; m=adj[u][k].mode; 
            inv=adj[u][k].interval; sh_e=adj[u][k].start_h; eh_e=adj[u][k].end_h; speed=adj[u][k].speed; break; 
        }
        wait = calculate_wait(cur_t, inv, sh_e, eh_e);
        format_time(cur_t + wait, t1); format_time(cur_t + wait + (d/speed)*60.0, t2);
        fprintf(txt, "%s - %s, Cost: BDT %.2f: Ride %s from (%f, %f) to (%f, %f).\n\n", t1, t2, d*cr, m, nodes[u].lon, nodes[u].lat, nodes[v].lon, nodes[v].lat);
        fprintf(kml, "%f,%f,0\n", nodes[v].lon, nodes[v].lat);
        cur_t += wait + (d/speed)*60.0;
    }
    double final_walk_time = (min_e / 2.0) * 60.0;
    format_time(cur_t, t1); format_time(cur_t + final_walk_time, t2);
    fprintf(txt, "%s - %s, Cost: BDT 0.00: Walk from (%f, %f) to Destination (%f, %f).\n", t1, t2, nodes[end_node].lon, nodes[end_node].lat, dLon, dLat);
    fprintf(kml, "%f,%f,0\n</coordinates></LineString></Placemark></Document></kml>", dLon, dLat);
    fclose(txt); fclose(kml);
    printf("Problem 6 solved. Files: problem6.kml, problem6_directions.txt\n");
}

int main() {
    double sLat, sLon, dLat, dLon; int sh, sm, dh, dm;
    printf("--- Problem 6: Cheapest within Deadline ---\n");
    printf("Source Lat Lon: "); scanf("%lf %lf", &sLat, &sLon);
    printf("Destination Lat Lon: "); scanf("%lf %lf", &dLat, &dLon);
    printf("Start Time (HH MM): "); scanf("%d %d", &sh, &sm);
    printf("Deadline Time (HH MM): "); scanf("%d %d", &dh, &dm);
    solve_problem6(sLat, sLon, dLat, dLon, sh, sm, dh, dm);
    return 0;
}