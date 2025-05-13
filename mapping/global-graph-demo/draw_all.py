import draw_paths
import draw_nodes 
import folium

jmu_origin = (38.433702, -78.861496)
m = folium.Map(location=jmu_origin, zoom_start=18)

draw_nodes.draw_on_map(m)
draw_paths.draw_on_map(m)

m.save("index.html")
