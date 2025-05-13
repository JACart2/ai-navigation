import folium
import numpy as np
import transformer

jmu_origin = (38.433702, -78.861496)

def draw_on_map(m):
    all_nodes = {}
    with open('main.gml', 'r') as file:
        line = file.readline()
        while line:
            if "node" in line:
                id = int(file.readline().split()[1])
                file.readline()
                file.readline()

                long = float(file.readline().split()[1])
                lat  = float(file.readline().split()[1])
        
                coords = (long, lat)
                coords = np.array([long, lat])

                coords = transformer.transform(coords)

                all_nodes[id] = coords

            if "edge" in line:
                p1 = all_nodes[int(file.readline().split()[1])]
                p2 = all_nodes[int(file.readline().split()[1])]

                folium.PolyLine(
                    locations=[p1, p2],
                    color="#FF0000",
                    weight=1,
                ).add_to(m)
            line = file.readline()
