import folium
import numpy as np
import transformer

jmu_origin = (38.433702, -78.861496)

def draw_on_map(m):
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

                folium.Circle(
                    location=coords,
                    radius=0.5,
                    color="black",
                    weight=1,
                    fill_opacity=1,
                    opacity=1,
                    fill_color="red",
                    fill=False,  # gets overridden by fill_color
                    tooltip=f"Id {id} coords {long}, {lat}"
                ).add_to(m)
            line = file.readline()
