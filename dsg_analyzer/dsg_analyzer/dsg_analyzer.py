#!/usr/bin/env python3
import spark_dsg as dsg

def dsg_general_stats_string(G: dsg.DynamicSceneGraph):
    stats_string = ""
    stats_string += f"Number of objects: {G.get_layer(dsg.DsgLayers.OBJECTS).num_nodes()}\n"
    # stats_string += f"Number of places: {G.get_layer(dsg.DsgLayers.PLACES).num_nodes()}\n"
    stats_string += f"Number of places: {G.get_layer(dsg.DsgLayers.MESH_PLACES).num_nodes()}\n"
    stats_string += f"Number of rooms: {G.get_layer(dsg.DsgLayers.ROOMS).num_nodes()}\n"
    
    return stats_string

def get_object_label_histogram(G: dsg.DynamicSceneGraph):
    key = G.get_layer_key(dsg.DsgLayers.OBJECTS)
    labelspace = G.get_labelspace(key.layer, key.partition)

    histogram = {}
    for name in labelspace.names_to_labels:
        histogram[name] = 0

    for node in G.get_layer(dsg.DsgLayers.OBJECTS).nodes:
        label = labelspace.get_node_category(node)
        if label is not None:
            histogram[label] += 1
        else:
            raise ValueError(f"Node {node.id.str()} has no label in the labelspace.")

    return histogram