# Walkshed #

Generates a geoJSON file containing a walkshed of transit stops. See [example](http://real.uwaterloo.ca/~mboos/?p=1174).

    usage: walkshed.py [-h] -j JSONFILE -g GTFSDIR -p PBFFILE -o OUTFILE
                        [-d DISTANCE] [-m MODE] [-c CONCURRENT]
                        [ID [ID ...]]

  - `JSONFILE`    file containing transit stop ids and locations (see sample.json)
  - `GTFSDIR`     folder containing GTFS dataset referenced in `JSONFILE`
  - `PBFFILE`     OpenStreetMap PBF file of geography
  - `OUTFILE`     output geoJSON file
  - `DISTANCE`    maximum extent of walkshed
  - `MODE`        transit mode for annotating geoJSON
  - `CONCURRENT`  number of processors to use in importing PBF file (default: 2)
  - `ID`          route ids to filter for inclusion

## Requirements ##
The python packages imposm and pygraph.

    pip install python-graph-core imposm
