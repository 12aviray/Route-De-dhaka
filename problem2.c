#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_NODES 8000
#define INF 1e15
#define PI 3.14159265358979323846

typedef struct {
    double lat, lon;
} Coord;

typedef struct {
    int to;
    double dist;
    double cost;
    char mode[30];
} Edge;

Coord nodes[MAX_NODES];
Edge adj[MAX_NODES][200];
int adj_size[MAX_NODES];
int node_count = 0;

// Distance calculation
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
    adj_size[node_count] = 0;
    return node_count++;
}

void add_edge(int u, int v, double d, double cost, char *mode) {
    adj[u][adj_size[u]].to = v;
    adj[u][adj_size[u]].dist = d;
    adj[u][adj_size[u]].cost = cost;
    strcpy(adj[u][adj_size[u]].mode, mode);
    adj_size[u]++;
}

void load_data() {
    FILE *fp; char line[5000], *token;
    
    // Load Car Data (Cost: 20 tk/km)
    fp = fopen("Roadmap-Dhaka.csv", "r");
    if(fp) {
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double coords[150]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) coords[c++] = atof(token);
            for (int i = 0; i < c - 4; i += 2) {
                int u = get_node_id(coords[i+1], coords[i]);
                int v = get_node_id(coords[i+3], coords[i+2]);
                double d = haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon);
                add_edge(u, v, d, d * 20.0, "Car"); add_edge(v, u, d, d * 20.0, "Car");
            }
        } fclose(fp);
    }

    // Load Metro Data (Cost: 5 tk/km)
    fp = fopen("Routemap-DhakaMetroRail.csv", "r");
    if(fp) {
        while (fgets(line, sizeof(line), fp)) {
            strtok(line, ","); double coords[500]; int c = 0;
            while ((token = strtok(NULL, ",")) != NULL) {
                if (atof(token) == 0 && c > 2) break; 
                coords[c++] = atof(token);
            }
            for (int i = 0; i < c - 3; i += 2) {
                int u = get_node_id(coords[i+1], coords[i]);
                int v = get_node_id(coords[i+3], coords[i+2]);
                double d = haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon);
                add_edge(u, v, d, d * 5.0, "Metro");
            }
        } fclose(fp);
    }
}

void solve_problem2(double sLat, double sLon, double dLat, double dLon) {
    load_data();
    int start_node = 0, end_node = 0;
    double min_s = INF, min_e = INF;
    for(int i=0; i<node_count; i++) {
        double d1 = haversine(sLat, sLon, nodes[i].lat, nodes[i].lon);
        if(d1 < min_s) { min_s = d1; start_node = i; }
        double d2 = haversine(dLat, dLon, nodes[i].lat, nodes[i].lon);
        if(d2 < min_e) { min_e = d2; end_node = i; }
    }

    double cost[MAX_NODES], time_at[MAX_NODES];
    int prev[MAX_NODES], visited[MAX_NODES] = {0};
    for(int i=0; i<MAX_NODES; i++) { cost[i] = INF; prev[i] = -1; }
    
    // Case C: Start by walking
    cost[start_node] = 0;
    time_at[start_node] = (min_s / 2.0) * 60.0; // 2 km/h

    for(int i=0; i<node_count; i++) {
        int u = -1;
        for(int j=0; j<node_count; j++)
            if(!visited[j] && (u == -1 || cost[j] < cost[u])) u = j;
        if(u == -1 || cost[u] == INF) break;
        visited[u] = 1;

        for(int k=0; k<adj_size[u]; k++) {
            int v = adj[u][k].to;
            if(cost[u] + adj[u][k].cost < cost[v]) {
                cost[v] = cost[u] + adj[u][k].cost;
                time_at[v] = time_at[u] + (adj[u][k].dist / 30.0) * 60.0;
                prev[v] = u;
            }
        }
    }

    if(cost[end_node] == INF) { printf("No path!\n"); return; }

    FILE *txt = fopen("problem2_directions.txt", "w");
    FILE *kml = fopen("problem2.kml", "w");
    fprintf(kml, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document><Placemark><LineString><coordinates>\n%f,%f,0\n", sLon, sLat);

    double current_mins = 8 * 60.0; // Starting at 8:00 AM
    fprintf(txt, "08:00 AM - %02d:%02d AM, Cost: BDT 0.00: Walk from Source (%f, %f) to (%f, %f).\n\n",
            (int)((current_mins + (min_s/2.0)*60)/60), (int)fmod(current_mins + (min_s/2.0)*60, 60), sLon, sLat, nodes[start_node].lon, nodes[start_node].lat);
    current_mins += (min_s / 2.0) * 60.0;

    int path[MAX_NODES], p_count = 0, curr = end_node;
    while(curr != -1) { path[p_count++] = curr; curr = prev[curr]; }

    for(int i = p_count - 1; i > 0; i--) {
        int u = path[i], v = path[i-1];
        double d, c; char *m;
        for(int k=0; k<adj_size[u]; k++) if(adj[u][k].to == v) { d = adj[u][k].dist; c = adj[u][k].cost; m = adj[u][k].mode; break; }
        double t = (d / 30.0) * 60.0;
        fprintf(txt, "%02d:%02d AM - %02d:%02d AM, Cost: BDT %.2f: Ride %s from (%f, %f) to (%f, %f).\n\n",
                (int)(current_mins/60), (int)fmod(current_mins, 60), (int)((current_mins+t)/60), (int)fmod(current_mins+t, 60), c, m, nodes[u].lon, nodes[u].lat, nodes[v].lon, nodes[v].lat);
        fprintf(kml, "%f,%f,0\n", nodes[v].lon, nodes[v].lat);
        current_mins += t;
    }

    double final_walk = (min_e / 2.0) * 60.0;
    fprintf(txt, "%02d:%02d AM - %02d:%02d AM, Cost: BDT 0.00: Walk from (%f, %f) to Destination (%f, %f).\n",
            (int)(current_mins/60), (int)fmod(current_mins, 60), (int)((current_mins+final_walk)/60), (int)fmod(current_mins+final_walk, 60), nodes[end_node].lon, nodes[end_node].lat, dLon, dLat);
    fprintf(kml, "%f,%f,0\n", dLon, dLat);
    fprintf(kml, "</coordinates></LineString></Placemark></Document></kml>");
    
    fclose(txt); fclose(kml);
    printf("\nProblem 2 Finished. Cheapest Cost: BDT %.2f\nFiles: problem2.kml, problem2_directions.txt\n", cost[end_node]);
}

int main() {
    double sLat, sLon, dLat, dLon;
    printf("--- Problem 2: Cheapest Route (Car & Metro) ---\n");
    printf("Enter Source Latitude and Longitude: ");
    scanf("%lf %lf", &sLat, &sLon);
    printf("Enter Destination Latitude and Longitude: ");
    scanf("%lf %lf", &dLat, &dLon);
    solve_problem2(sLat, sLon, dLat, dLon);
    return 0;
}