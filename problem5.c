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

void add_edge(int u, int v, double d, double rate, char *mode) {
    if (adj_size[u] >= adj_cap[u]) {
        adj_cap[u] *= 2;
        adj[u] = (Edge *)realloc(adj[u], adj_cap[u] * sizeof(Edge));
    }
    adj[u][adj_size[u]].to = v; adj[u][adj_size[u]].dist = d;
    adj[u][adj_size[u]].cost_rate = rate; strcpy(adj[u][adj_size[u]].mode, mode);
    adj_size[u]++;
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
    // Car data
    fp = fopen("Roadmap-Dhaka.csv", "r");
    if(fp) {
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double c_vals[150]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) c_vals[c++] = atof(token);
            for (int i = 0; i < c - 4; i += 2) {
                int u = get_node_id(c_vals[i+1], c_vals[i]); int v = get_node_id(c_vals[i+3], c_vals[i+2]);
                double d = haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon);
                add_edge(u, v, d, 20.0, "Car"); add_edge(v, u, d, 20.0, "Car");
            }
        } fclose(fp);
    }
    // Metro and Bus data
    char *files[] = {"Routemap-DhakaMetroRail.csv", "Routemap-BikolpoBus.csv", "Routemap-UttaraBus.csv"};
    char *modes[] = {"Metro", "Bikolpo Bus", "Uttara Bus"};
    double rates[] = {5.0, 7.0, 7.0};
    for(int i=0; i<3; i++) {
        fp = fopen(files[i], "r"); if(!fp) continue;
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double c_vals[1000]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) { if (atof(token) == 0 && c > 2) break; c_vals[c++] = atof(token); }
            for (int j = 0; j < c - 3; j += 2) {
                int u = get_node_id(c_vals[j+1], c_vals[j]); int v = get_node_id(c_vals[j+3], c_vals[j+2]);
                add_edge(u, v, haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon), rates[i], modes[i]);
            }
        } fclose(fp);
    }
}

double get_wait(double curr, char *mode) {
    if (strcmp(mode, "Car") == 0) return 0;
    if (curr < 360) return 360 - curr; // Service starts at 6 AM
    if (curr > 1320) return INF;      // Service ends at 11 PM
    return fmod(15.0 - fmod(curr, 15.0), 15.0);
}

void solve_problem5(double sLat, double sLon, double dLat, double dLon, int sh, int sm) {
    load_data();
    double start_time = sh * 60.0 + sm;
    int start_node = 0, end_node = 0; double min_s = INF, min_e = INF;
    for(int i=0; i<node_count; i++) {
        double d1 = haversine(sLat, sLon, nodes[i].lat, nodes[i].lon); if(d1 < min_s) { min_s = d1; start_node = i; }
        double d2 = haversine(dLat, dLon, nodes[i].lat, nodes[i].lon); if(d2 < min_e) { min_e = d2; end_node = i; }
    }

    double time_at[MAX_NODES], total_cost[MAX_NODES]; int prev[MAX_NODES], visited[MAX_NODES] = {0};
    for(int i=0; i<MAX_NODES; i++) { time_at[i] = INF; total_cost[i] = 0; prev[i] = -1; }
    
    // Initial walking to road
    time_at[start_node] = start_time + (min_s / 2.0) * 60.0;

    for(int i=0; i<node_count; i++) {
        int u = -1;
        for(int j=0; j<node_count; j++) if(!visited[j] && (u == -1 || time_at[j] < time_at[u])) u = j;
        if(u == -1 || time_at[u] == INF) break; visited[u] = 1;

        for(int k=0; k<adj_size[u]; k++) {
            Edge e = adj[u][k]; double wait = get_wait(time_at[u], e.mode);
            double travel = (e.dist / 10.0) * 60.0; // 10 km/h speed
            if(wait != INF && time_at[u] + wait + travel < time_at[e.to]) {
                time_at[e.to] = time_at[u] + wait + travel;
                total_cost[e.to] = total_cost[u] + (e.dist * e.cost_rate);
                prev[e.to] = u;
            }
        }
    }

    if(time_at[end_node] == INF) { printf("No fastest route found.\n"); return; }

    FILE *txt = fopen("problem5_directions.txt", "w");
    FILE *kml = fopen("problem5.kml", "w");
    fprintf(kml, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document><Placemark><LineString><coordinates>%f,%f,0\n", sLon, sLat);
    
    char t1[20], t2[20];
    format_time(start_time, t1); format_time(time_at[start_node], t2);
    fprintf(txt, "%s - %s, Cost: BDT 0.00: Walk from Source (%f, %f) to (%f, %f).\n\n", t1, t2, sLon, sLat, nodes[start_node].lon, nodes[start_node].lat);

    int path[MAX_NODES], p_count = 0, curr = end_node;
    while(curr != -1) { path[p_count++] = curr; curr = prev[curr]; }

    double cur_t = time_at[start_node];
    for(int i = p_count - 1; i > 0; i--) {
        int u = path[i], v = path[i-1]; double d, cr, wait; char *m;
        for(int k=0; k<adj_size[u]; k++) if(adj[u][k].to == v) { d=adj[u][k].dist; cr=adj[u][k].cost_rate; m=adj[u][k].mode; break; }
        wait = get_wait(cur_t, m);
        format_time(cur_t + wait, t1); format_time(cur_t + wait + (d/10.0)*60.0, t2);
        fprintf(txt, "%s - %s, Cost: BDT %.2f: Ride %s from (%f, %f) to (%f, %f).\n\n", t1, t2, d*cr, m, nodes[u].lon, nodes[u].lat, nodes[v].lon, nodes[v].lat);
        fprintf(kml, "%f,%f,0\n", nodes[v].lon, nodes[v].lat);
        cur_t += wait + (d/10.0)*60.0;
    }
    format_time(cur_t, t1); format_time(cur_t + (min_e/2.0)*60.0, t2);
    fprintf(txt, "%s - %s, Cost: BDT 0.00: Walk from (%f, %f) to Destination (%f, %f).\n", t1, t2, nodes[end_node].lon, nodes[end_node].lat, dLon, dLat);
    fprintf(kml, "%f,%f,0\n</coordinates></LineString></Placemark></Document></kml>", dLon, dLat);
    fclose(txt); fclose(kml);
    printf("Problem 5 solved. Files: problem5.kml, problem5_directions.txt\n");
}

int main() {
    double sLat, sLon, dLat, dLon; int h, m;
    printf("--- Problem 5: Fastest Route (Time Based) ---\n");
    printf("Enter Source Latitude and Longitude: "); scanf("%lf %lf", &sLat, &sLon);
    printf("Enter Destination Latitude and Longitude: "); scanf("%lf %lf", &dLat, &dLon);
    printf("Enter Starting Time (HH MM): "); scanf("%d %d", &h, &m);
    solve_problem5(sLat, sLon, dLat, dLon, h, m);
    return 0;
}