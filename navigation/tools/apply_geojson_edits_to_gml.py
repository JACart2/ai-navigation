#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path

import networkx as nx


EARTH_RADIUS_M = 6_378_137.0


def geographic_distance_m(
    latitude_1,
    longitude_1,
    latitude_2,
    longitude_2,
):
    lat1 = math.radians(latitude_1)
    lon1 = math.radians(longitude_1)
    lat2 = math.radians(latitude_2)
    lon2 = math.radians(longitude_2)

    delta_latitude = lat2 - lat1
    delta_longitude = lon2 - lon1

    value = (
        math.sin(delta_latitude / 2.0) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(delta_longitude / 2.0) ** 2
    )

    return EARTH_RADIUS_M * 2.0 * math.atan2(
        math.sqrt(value),
        math.sqrt(1.0 - value),
    )


def load_geojson_nodes(path):
    data = json.loads(path.read_text(encoding="utf-8"))

    if data.get("type") != "FeatureCollection":
        raise ValueError("Expected a GeoJSON FeatureCollection.")

    nodes = {}

    for feature in data.get("features", []):
        properties = feature.get("properties", {})
        geometry = feature.get("geometry", {})

        node_id = properties.get("node_id")

        if not node_id:
            raise ValueError("A feature is missing node_id.")

        if node_id in nodes:
            raise ValueError(f"Duplicate node_id: {node_id}")

        if geometry.get("type") != "Point":
            raise ValueError(
                f"Node {node_id} is not Point geometry."
            )

        coordinates = geometry.get("coordinates", [])

        if len(coordinates) < 2:
            raise ValueError(
                f"Node {node_id} has invalid coordinates."
            )

        longitude = float(coordinates[0])
        latitude = float(coordinates[1])

        nodes[node_id] = {
            "lat": latitude,
            "lon": longitude,
        }

    return nodes


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Apply edited GeoJSON node coordinates to a GML graph "
            "and recalculate edge weights."
        )
    )

    parser.add_argument("--input-gml", required=True, type=Path)
    parser.add_argument("--edited-nodes", required=True, type=Path)
    parser.add_argument("--output-gml", required=True, type=Path)

    parser.add_argument(
        "--maximum-node-movement-m",
        type=float,
        default=20.0,
        help="Reject any individual edit larger than this distance.",
    )

    args = parser.parse_args()

    input_gml = args.input_gml.resolve()
    edited_nodes_path = args.edited_nodes.resolve()
    output_gml = args.output_gml.resolve()

    if input_gml == output_gml:
        raise ValueError("Input and output GML must differ.")

    graph = nx.read_gml(input_gml)
    edited_nodes = load_geojson_nodes(edited_nodes_path)

    unknown_nodes = set(edited_nodes) - set(graph.nodes)

    if unknown_nodes:
        preview = sorted(unknown_nodes)[:10]
        raise ValueError(
            f"GeoJSON contains unknown node IDs: {preview}"
        )

    changed_nodes = []

    for node_id, new_coordinates in edited_nodes.items():
        attributes = graph.nodes[node_id]

        old_latitude = float(attributes["lat"])
        old_longitude = float(attributes["lon"])

        new_latitude = new_coordinates["lat"]
        new_longitude = new_coordinates["lon"]

        movement = geographic_distance_m(
            old_latitude,
            old_longitude,
            new_latitude,
            new_longitude,
        )

        if movement > args.maximum_node_movement_m:
            raise ValueError(
                f"{node_id} moved {movement:.3f} m, exceeding "
                f"the {args.maximum_node_movement_m:.3f} m limit."
            )

        if movement > 0.01:
            changed_nodes.append((node_id, movement))

        attributes["lat"] = new_latitude
        attributes["lon"] = new_longitude

    for source, target in graph.edges:
        source_attributes = graph.nodes[source]
        target_attributes = graph.nodes[target]

        graph.edges[source, target]["weight"] = (
            geographic_distance_m(
                float(source_attributes["lat"]),
                float(source_attributes["lon"]),
                float(target_attributes["lat"]),
                float(target_attributes["lon"]),
            )
        )

    output_gml.parent.mkdir(parents=True, exist_ok=True)
    nx.write_gml(graph, output_gml)

    print("=== Manual GML update complete ===")
    print(f"Input GML: {input_gml}")
    print(f"Edited nodes: {edited_nodes_path}")
    print(f"Output GML: {output_gml}")
    print(f"GeoJSON nodes processed: {len(edited_nodes)}")
    print(f"Nodes actually moved: {len(changed_nodes)}")
    print(f"Edge weights recalculated: {graph.number_of_edges()}")
    print()

    for node_id, movement in sorted(
        changed_nodes,
        key=lambda item: item[1],
        reverse=True,
    ):
        print(f"{node_id}: {movement:.3f} m")


if __name__ == "__main__":
    main()
