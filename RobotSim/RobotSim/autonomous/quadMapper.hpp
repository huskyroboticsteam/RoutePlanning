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
Implementation notes:
Use the QuadTree data structure, using the following splitting algorithm:

    func make_quadtree(AABB):
        if area of AABB < epsilon OR no obstacles within AABB:  # note there is some tolerance value here
            return

        foreach quadrant within AABB:
            make_quadtree(quadrant)

The center of each leaf node is a node in the final graph.

Things I haven't figured out:
- how to efficiently connect start/end node with a quadtree node (should be a search algorithm)
- how to connect node with neighbors

*/
#ifndef RP_DISCRETEMAPPER_HPP
#define RP_DISCRETEMAPPER_HPP

#endif
