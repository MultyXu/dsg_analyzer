#!/usr/bin/env python3
import spark_dsg as dsg

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