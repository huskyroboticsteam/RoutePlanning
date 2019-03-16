// a more computationally expensive, but potentially more robust in more maze-y situations, alternative to Map
// also this could actually be less expensive than map for really cramped maps
/*
TODOs
1) split Map into two files: one is the Mapper, which converts obstacle coordinates to a graph; the other is
a pather, which finds a path within the graph. Also possibly isolate the obstacle merging part
2) implement inheritance. rename Map to SplitMap or ContinuousMap or something, and then make this and SplitMap inherit
from a new abtract Map.cpp
*/

/*
idea: split entire map into grids (the smaller the grid size the more accurate but also more expensive).
go thru each obstacle and "rasterize" them, painting grids that it passes through (likely as well as nearby grids,
taking into account of the size of robot) as obstacles. At the end, construct massive graph from grid
*/
