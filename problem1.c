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
} Edge;

Coord nodes[MAX_NODES];
Edge adj[MAX_NODES][150];
int adj_size[MAX_NODES];
int node_count = 0;

// Haversine formula to calculate distance in KM
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

void load_roadmap() {
    FILE *fp = fopen("Roadmap-Dhaka.csv", "r");
    if (!fp) { printf("Error: Roadmap-Dhaka.csv not found!\n"); exit(1); }
    char line[4096];
    while (fgets(line, sizeof(line), fp)) {
        char *token = strtok(line, ","); // Skip "DhakaStreet"
        double coords[100]; int c = 0;
        while ((token = strtok(NULL, ",")) != NULL) coords[c++] = atof(token);
        // Process segments in the line
        for (int i = 0; i < c - 4; i += 2) {
            int u = get_node_id(coords[i+1], coords[i]);
            int v = get_node_id(coords[i+3], coords[i+2]);
            double d = haversine(nodes[u].lat, nodes[u].lon, nodes[v].lat, nodes[v].lon);
            adj[u][adj_size[u]].to = v; adj[u][adj_size[u]++].dist = d;
            adj[v][adj_size[v]].to = u; adj[v][adj_size[v]++].dist = d;
        }
    }
    fclose(fp);
}

void solve_problem1(double sLat, double sLon, double dLat, double dLon) {
    load_roadmap();
    int start_node = 0, end_node = 0;
    double min_s = INF, min_e = INF;
    for(int i=0; i<node_count; i++) {
        double d1 = haversine(sLat, sLon, nodes[i].lat, nodes[i].lon);
        if(d1 < min_s) { min_s = d1; start_node = i; }
        double d2 = haversine(dLat, dLon, nodes[i].lat, nodes[i].lon);
        if(d2 < min_e) { min_e = d2; end_node = i; }
    }

    double dist[MAX_NODES]; int prev[MAX_NODES], visited[MAX_NODES] = {0};
    for(int i=0; i<MAX_NODES; i++) { dist[i] = INF; prev[i] = -1; }
    dist[start_node] = 0;

    for(int i=0; i<node_count; i++) {
        int u = -1;
        for(int j=0; j<node_count; j++) if(!visited[j] && (u == -1 || dist[j] < dist[u])) u = j;
        if(u == -1 || dist[u] == INF) break;
        visited[u] = 1;
        for(int k=0; k<adj_size[u]; k++) {
            int v = adj[u][k].to;
            if(dist[u] + adj[u][k].dist < dist[v]) { dist[v] = dist[u] + adj[u][k].dist; prev[v] = u; }
        }
    }

    if(dist[end_node] == INF) { printf("No path found!\n"); return; }

    // Start generating direction output (Default start time: 09:00 AM)
    double current_mins = 9 * 60.0;
    FILE *txt = fopen("problem1_directions.txt", "w");
    FILE *kml = fopen("problem1.kml", "w");

    fprintf(kml, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<Placemark><LineString><coordinates>\n");
    fprintf(kml, "%f,%f,0\n", sLon, sLat);

    // Step 1: Walk to nearest road point (Case C)
    double walk_time = (min_s / 2.0) * 60.0; // 2 km/h walking speed
    fprintf(txt, "09:00 AM - %02d:%02d AM, Cost: BDT 0.00: Walk from Source (%f, %f) to (%f, %f).\n\n",
            (int)((current_mins+walk_time)/60), (int)fmod(current_mins+walk_time, 60),
            sLon, sLat, nodes[start_node].lon, nodes[start_node].lat);
    current_mins += walk_time;

    // Path Reconstruction
    int path[MAX_NODES], p_count = 0, curr = end_node;
    while(curr != -1) { path[p_count++] = curr; curr = prev[curr]; }

    for(int i = p_count - 1; i > 0; i--) {
        int u = path[i], v = path[i-1];
        double d = 0;
        for(int k=0; k<adj_size[u]; k++) if(adj[u][k].to == v) { d = adj[u][k].dist; break; }
        double travel_time = (d / 30.0) * 60.0; // Assume 30 km/h car speed
        double cost = d * 20.0; // Car cost 20 tk/km
        fprintf(txt, "%02d:%02d AM - %02d:%02d AM, Cost: BDT %.2f: Ride Car from (%f, %f) to (%f, %f).\n\n",
                (int)(current_mins/60), (int)fmod(current_mins, 60),
                (int)((current_mins+travel_time)/60), (int)fmod(current_mins+travel_time, 60),
                cost, nodes[u].lon, nodes[u].lat, nodes[v].lon, nodes[v].lat);
        fprintf(kml, "%f,%f,0\n", nodes[v].lon, nodes[v].lat);
        current_mins += travel_time;
    }

    // Step 3: Final Walk to destination
    double final_walk_time = (min_e / 2.0) * 60.0;
    fprintf(txt, "%02d:%02d AM - %02d:%02d AM, Cost: BDT 0.00: Walk from (%f, %f) to Destination (%f, %f).\n",
            (int)(current_mins/60), (int)fmod(current_mins, 60),
            (int)((current_mins+final_walk_time)/60), (int)fmod(current_mins+final_walk_time, 60),
            nodes[end_node].lon, nodes[end_node].lat, dLon, dLat);
    fprintf(kml, "%f,%f,0\n", dLon, dLat);

    fprintf(kml, "</coordinates></LineString></Placemark></Document></kml>");
    fclose(txt); fclose(kml);
    printf("\nProblem 1 Finished.\nDistance: %.2f km\nFiles created: problem1.kml, problem1_directions.txt\n", dist[end_node]);
}

int main() {
    double sLat, sLon, dLat, dLon;
    printf("--- Problem 1: Shortest Car Path ---\n");
    printf("Enter Source Latitude and Longitude: ");
    scanf("%lf %lf", &sLat, &sLon);
    printf("Enter Destination Latitude and Longitude: ");
    scanf("%lf %lf", &dLat, &dLon);
    solve_problem1(sLat, sLon, dLat, dLon);
    return 0;
}