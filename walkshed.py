#!/usr/bin/env python

from imposm.parser import OSMParser
from pygraph.classes.graph import graph
from pygraph.algorithms.minmax import shortest_path

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
        
        # build the graph from our map
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
                    if dist <= 10: # here we add all nodes within 10 m (approx road width)
                        gr.add_edge((stop, key), dist)
                    if dist < nearestDist:
                        nearestDist = node.distanceTo(stop)
                        nearest = key
            # ensure the nearest node is not missed
            if nearestDist > 10:
                gr.add_edge((stop, nearest), nearestDist)
            
            # this graph libary may be overkill (i.e. we should probably stop Djikstra's algorithm
            # after some distance) but it beats reinventing the wheel
            st, distances = shortest_path(gr, stop)
            
            # Set the walking distance to be whatever is the nearest stop
            for key, dist in distances.iteritems():
                if key in self.nodes and dist < self.nodes[key].walkDistance:
                    self.nodes[key].walkDistance = dist
    
    def generateWalkShed(self, distThreshold, props=None):
        # base geoJSON
        feature = {'type': 'Feature',
                    'geometry' : {'type': 'MultiLineString', 'coordinates': []},
                    'properties': props or {}}
        
        # add coordinates as LineStrings to geoJSON MultiLineString
        def addSequence(feature, sequence):
            coords = []
            for node in sequence:
                coords.append([node.lon, node.lat])
            feature['geometry']['coordinates'].append(coords)
        
        # for each way, determine which nodes are within walking threshold
        for way in self.ways.values():
            sequence = []
            for i, noderef in enumerate(way.refs):
                if noderef in self.nodes:
                    node = self.nodes[noderef]
                    distLeft = distThreshold - node.walkDistance
                    if distLeft >= 0: # node is within walking threshold
                        if len(sequence) == 0 and i > 0:
                            # need to back-track to find the point on way where walkability begins
                            lastNode = self.nodes[way.refs[i-1]]
                            lastDist = node.distanceTo(lastNode)
                            portion = distLeft / lastDist
                            sequence.append(Node(node.lon + portion * (lastNode.lon - node.lon),
                                                 node.lat + portion * (lastNode.lat - node.lat)))
                        sequence.append(node)
                        if i+1 < len(way.refs): # peek ahead to next node
                            nextNode = self.nodes[way.refs[i+1]]
                            nextDistLeft = distThreshold - nextNode.walkDistance
                            nextDist = node.distanceTo(nextNode)
                            if nextDistLeft < 0 or distLeft + nextDistLeft < nextDist:
                                # if the next node is not in the walkable area, or there is a portion
                                # of the way between this node and the next that too far from transit,
                                # find the point of the furthest possible walking distance and stop
                                # line there
                                portion = distLeft / nextDist
                                sequence.append(Node(node.lon + portion * (nextNode.lon - node.lon),
                                                 node.lat + portion * (nextNode.lat - node.lat)))
                                addSequence(feature, sequence)
                                sequence = []
            # if the final node on the way was walkable, need to add this last LineString
            if len(sequence) > 0:
                addSequence(feature, sequence)
        
        return feature
    
def processShed(pbfFile, outFile, distance, mode, concurrent, stopNodes):
    # instantiate map and parser and start parsing
    osmap = Map()
    p = OSMParser(concurrency=concurrent, ways_callback=osmap.ways_callback, coords_callback=osmap.nodes_callback)
    p.parse(pbfFile)

    osmap.calculateWalkDistances(stopNodes)
    walkshed = osmap.generateWalkShed(distance, {'distance': distance, 'mode': mode})
    
    import json
    with open(outFile, 'w') as f:
        json.dump(walkshed, f)
    
def addRouteStops(route, stopNodes, stops):
    # add 
    for trip in route['trips']:
        for stop in trip['stops']:
            if 'stop_id' in stop:
                stopNodes.append(Node(stops[stop['stop_id']]['stop_lon'],
                                      stops[stop['stop_id']]['stop_lat']))
            else:
                stopNodes.append(Node(stop['lon'], stop['lat']))

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate new GTFS rows based on estimates of future service.')
    parser.add_argument('-j', '--json', dest='jsonFile', required=True,
                       help='json file containing future transit service information')
    parser.add_argument('-g', '--gtfs', dest='gtfsDir', required=True,
                       help='directory containing GTFS of existing service')
    parser.add_argument('-p', '--pbf', dest='pbfFile', required=True,
                       help='OSM map file')
    parser.add_argument('-o', '--output', dest='outFile', required=True,
                       help='file where geoJSON data is to be output')
    parser.add_argument('-d', '--distance', dest='distance', default=400, type=float,
                       help='walkable distance from transit stations')
    parser.add_argument('-m', '--mode', dest='mode', default='bus', 
                       help='mode of service')
    parser.add_argument('-c', dest='concurrent', default=2, type=int,
                       help='number of processors to use when importing map')
    parser.add_argument('ids', nargs="*", default=[],
                       help='id of route in json file to include')
    args = parser.parse_args()
    
    import os.path
    from readCSV import readCSV
    stops, headings = readCSV(os.path.join(args.gtfsDir, 'stops.txt'), 'stop_id')
    
    import json
    with open(args.jsonFile, 'r') as file:
        routes = json.load(file)
        
    stopNodes = []
    for route in routes:
        if len(args.ids) == 0 or route['id'] in args.ids:
            addRouteStops(route, stopNodes, stops)
    
    processShed(args.pbfFile, args.outFile, args.distance, args.mode, args.concurrent, stopNodes)
        
if __name__ == "__main__":
    main()