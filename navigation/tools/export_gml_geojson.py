#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import networkx as nx


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as file:
        json.dump(
            data,
            file,
            indent=2,
            ensure_ascii=False,
            sort_keys=True,
        )

    print(f"Wrote: {path}")


def make_node_feature(
    graph: nx.DiGraph,
    node_id: str,
    component_id: int,
    component_size: int,
    main_component_id: int,
) -> dict:
    attributes = graph.nodes[node_id]

    latitude = float(attributes["lat"])
    longitude = float(attributes["lon"])

    return {
        "type": "Feature",
        "geometry": {
            "type": "Point",
            # GeoJSON always uses longitude, latitude order.
            "coordinates": [longitude, latitude],
        },
        "properties": {
            "node_id": str(node_id),
            "active": int(attributes.get("active", 0)),
            "in_degree": int(graph.in_degree(node_id)),
            "out_degree": int(graph.out_degree(node_id)),
            "total_degree": int(
                graph.in_degree(node_id) + graph.out_degree(node_id)
            ),
            "component_id": component_id,
            "component_size": component_size,
            "is_main_component": component_id == main_component_id,
        },
    }


def make_edge_feature(
    graph: nx.DiGraph,
    source: str,
    target: str,
    component_id: int,
    main_component_id: int,
) -> dict:
    source_attributes = graph.nodes[source]
    target_attributes = graph.nodes[target]
    edge_attributes = graph.edges[source, target]

    source_coordinate = [
        float(source_attributes["lon"]),
        float(source_attributes["lat"]),
    ]

    target_coordinate = [
        float(target_attributes["lon"]),
        float(target_attributes["lat"]),
    ]

    return {
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": [
                source_coordinate,
                target_coordinate,
            ],
        },
        "properties": {
            "source": str(source),
            "target": str(target),
            "weight": float(edge_attributes.get("weight", 0.0)),
            "component_id": component_id,
            "is_main_component": component_id == main_component_id,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export a JACart GML navigation graph to GeoJSON."
    )

    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="Input GML file.",
    )

    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="Directory for generated GeoJSON files.",
    )

    args = parser.parse_args()

    input_path = args.input.resolve()
    output_dir = args.output_dir.resolve()

    if not input_path.exists():
        raise FileNotFoundError(f"GML file does not exist: {input_path}")

    print(f"Reading: {input_path}")
    graph = nx.read_gml(input_path)

    if not graph.is_directed():
        raise ValueError("Expected a directed GML graph.")

    missing_coordinates = [
        node_id
        for node_id, attributes in graph.nodes(data=True)
        if "lat" not in attributes or "lon" not in attributes
    ]

    if missing_coordinates:
        preview = ", ".join(map(str, missing_coordinates[:10]))
        raise ValueError(
            f"{len(missing_coordinates)} nodes are missing lat/lon. "
            f"First nodes: {preview}"
        )

    components = sorted(
        nx.weakly_connected_components(graph),
        key=len,
        reverse=True,
    )

    component_by_node = {}
    component_sizes = {}

    for component_id, component_nodes in enumerate(components):
        component_sizes[component_id] = len(component_nodes)

        for node_id in component_nodes:
            component_by_node[node_id] = component_id

    main_component_id = 0
    main_nodes = set(components[main_component_id])

    all_node_features = []
    main_node_features = []

    for node_id in graph.nodes:
        component_id = component_by_node[node_id]

        feature = make_node_feature(
            graph=graph,
            node_id=node_id,
            component_id=component_id,
            component_size=component_sizes[component_id],
            main_component_id=main_component_id,
        )

        all_node_features.append(feature)

        if node_id in main_nodes:
            main_node_features.append(feature)

    all_edge_features = []
    main_edge_features = []

    for source, target in graph.edges:
        component_id = component_by_node[source]

        feature = make_edge_feature(
            graph=graph,
            source=source,
            target=target,
            component_id=component_id,
            main_component_id=main_component_id,
        )

        all_edge_features.append(feature)

        if source in main_nodes and target in main_nodes:
            main_edge_features.append(feature)

    latitudes = [
        float(attributes["lat"])
        for _, attributes in graph.nodes(data=True)
    ]

    longitudes = [
        float(attributes["lon"])
        for _, attributes in graph.nodes(data=True)
    ]

    summary = {
        "source_file": str(input_path),
        "directed": graph.is_directed(),
        "multigraph": graph.is_multigraph(),
        "node_count": graph.number_of_nodes(),
        "edge_count": graph.number_of_edges(),
        "weak_component_count": len(components),
        "largest_weak_component_nodes": len(main_nodes),
        "largest_weak_component_edges": len(main_edge_features),
        "coordinate_bounds": {
            "minimum_latitude": min(latitudes),
            "maximum_latitude": max(latitudes),
            "minimum_longitude": min(longitudes),
            "maximum_longitude": max(longitudes),
        },
    }

    feature_collection_base = {
        "type": "FeatureCollection",
    }

    write_json(
        output_dir / "nodes_all.geojson",
        {
            **feature_collection_base,
            "name": "JACart GML nodes — all components",
            "features": all_node_features,
        },
    )

    write_json(
        output_dir / "edges_all.geojson",
        {
            **feature_collection_base,
            "name": "JACart GML edges — all components",
            "features": all_edge_features,
        },
    )

    write_json(
        output_dir / "nodes_main.geojson",
        {
            **feature_collection_base,
            "name": "JACart GML nodes — main component",
            "features": main_node_features,
        },
    )

    write_json(
        output_dir / "edges_main.geojson",
        {
            **feature_collection_base,
            "name": "JACart GML edges — main component",
            "features": main_edge_features,
        },
    )

    write_json(
        output_dir / "graph_summary.json",
        summary,
    )

    print()
    print("=== Export complete ===")
    print(f"All nodes: {len(all_node_features)}")
    print(f"All edges: {len(all_edge_features)}")
    print(f"Main-component nodes: {len(main_node_features)}")
    print(f"Main-component edges: {len(main_edge_features)}")


if __name__ == "__main__":
    main()
