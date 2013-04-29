#!/usr/bin/env python

from imposm.parser import OSMParser
from pygraph.classes.graph import graph
from pygraph.algorithms.minmax import shortest_path

from readCSV import readCSV

import math

EARTH_RADIUS = 6371e3

class Node(object):
    def __init__(self, lon, lat):
        self.lon = float(lon)
        self.lat = float(lat)
        self.walkable = False
        self.walkDistance = float("inf")
    
    def distanceTo(self, other):
        lat1 = self.lat /180. * math.pi
        lat2 = other.lat /180. * math.pi
        lon1 = self.lon /180. * math.pi
        lon2 = other.lon /180. * math.pi

        val = math.acos(min(math.sin(lat1) * math.sin(lat2) + math.cos(lat1) *
                        math.cos(lat2) * math.cos(lon2-lon1), 1)) * EARTH_RADIUS
        if math.isnan(val):
            return 0
        else:
            return val    
    
class Way(object):
    def __init__(self, tags, refs):
        self.tags = tags
        self.refs = refs

# simple class that handles the parsed OSM data.
class Map(object):
    nodes = {}
    ways = {}
    
    def nodes_callback(self, nodes):
        for osmid, lon, lat in nodes:
            self.nodes[osmid] = Node(lon, lat)

    def ways_callback(self, ways):
        for osmid, tags, refs in ways:
            if 'highway' in tags and 'motor' not in tags['highway']:
                self.ways[osmid] = Way(tags, refs)
                for index in refs:
                    if index in self.nodes:
                        self.nodes[index].walkable = True
    
    def _clearDistances(self, ):
        inf = float("inf")
        for node in self.nodes.values():
            node.walkDistance = inf
    
    def calculateWalkDistances(self, stops):
        self._clearDistances()
        gr = graph()
        gr.add_nodes(self.nodes.keys())
        
        for way in self.ways.values():
            lastNode = None
            for node in way.refs:
                if (lastNode and lastNode in self.nodes and
                        node in self.nodes and not gr.has_edge((lastNode, node))):
                    gr.add_edge((lastNode, node), self.nodes[lastNode].distanceTo(self.nodes[node]))
                lastNode = node
        
        #Add nodes from stops
        for stop in stops:
            #add stop to graph
            gr.add_node(stop)
            
            #find nearest node to stop
            nearest = -1
            nearestDist = float('inf')
            for key, node in self.nodes.iteritems():
                if node.walkable:
                    dist = node.distanceTo(stop)
                    if dist <= 10:
                        gr.add_edge((stop, key), dist)
                    if dist < nearestDist:
                        nearestDist = node.distanceTo(stop)
                        nearest = key
                        
            if nearestDist > 10:
                gr.add_edge((stop, nearest), nearestDist)
            
            st, distances = shortest_path(gr, stop)
            
            for key, dist in distances.iteritems():
                if key in self.nodes and dist < self.nodes[key].walkDistance:
                    self.nodes[key].walkDistance = dist
    
    def generateWalkShed(self, distThreshold, props=None):
        feature = {'type': 'Feature',
                    'geometry' : {'type': 'MultiLineString', 'coordinates': []},
                    'properties': props or {}}
        
        def addSequence(feature, sequence):
            coords = []
            for node in sequence:
                coords.append([node.lon, node.lat])
            feature['geometry']['coordinates'].append(coords)
        
        
        for way in self.ways.values():
            sequence = []
            for i, noderef in enumerate(way.refs):
                if noderef in self.nodes:
                    node = self.nodes[noderef]
                    distLeft = distThreshold - node.walkDistance
                    if distLeft >= 0:
                        if len(sequence) == 0 and i > 0:
                            lastNode = self.nodes[way.refs[i-1]]
                            lastDist = node.distanceTo(lastNode)
                            portion = distLeft / lastDist
                            sequence.append(Node(node.lon + portion * (lastNode.lon - node.lon),
                                                 node.lat + portion * (lastNode.lat - node.lat)))
                        sequence.append(node)
                        if i+1 < len(way.refs):
                            nextNode = self.nodes[way.refs[i+1]]
                            nextDistLeft = distThreshold - nextNode.walkDistance
                            nextDist = node.distanceTo(nextNode)
                            if nextDistLeft < 0 or distLeft + nextDistLeft < nextDist:
                                portion = distLeft / nextDist
                                sequence.append(Node(node.lon + portion * (nextNode.lon - node.lon),
                                                 node.lat + portion * (nextNode.lat - node.lat)))
                                addSequence(feature, sequence)
                                sequence = []
                
            if len(sequence) > 0:
                addSequence(feature, sequence)
        
        return feature
    
def addRouteStops(route, stopNodes, stops):
    for trip in route['trips']:
        for stop in trip['stops']:
            if 'stop_id' in stop:
                stopNodes.append(Node(stops[stop['stop_id']]['stop_lon'],
                                      stops[stop['stop_id']]['stop_lat']))
            else:
                stopNodes.append(Node(stop['lon'], stop['lat']))

# instantiate map and parser and start parsing
osmap = Map()
p = OSMParser(concurrency=2, ways_callback=osmap.ways_callback, coords_callback=osmap.nodes_callback)
p.parse('waterloo.osm.pbf')

stops, headings = readCSV('gtfs/stops.txt', 'stop_id')

import json
with open('sample.json', 'r') as file:
    routes = json.load(file)
    
stopNodes = []
for route in routes:
    if route['id'] == 'LRT' or route['id'] == 'aBRT':
        addRouteStops(route, stopNodes, stops)

osmap.calculateWalkDistances(stopNodes)
features = {"type": "FeatureCollection", "features": []}
rt400 = osmap.generateWalkShed(400, {'distance': 400, 'mode': 'rapid'})
rt800 = osmap.generateWalkShed(800, {'distance': 800, 'mode': 'rapid'})

stopNodes = []
for route in routes:
    if route['id'] != 'LRT' and route['id'] != 'aBRT':
        addRouteStops(route, stopNodes, stops)
osmap.calculateWalkDistances(stopNodes)
features['features'].append(osmap.generateWalkShed(400, {'distance': 400, 'mode': 'express'}))

features['features'].append(rt800)
features['features'].append(rt400)

print 'features = %s;' % features