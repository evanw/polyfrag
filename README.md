This library contains a stable algorithm for splitting a convex polyhedron
into two pieces on either side of a 3D plane.  It is useful for generating
polyhedron fragments to make a polyhedron look like it's breaking apart.

Example usage:

    Polyhedron *poly = Polyhedron::box();
    Plane plane(Vector3D(1, 2, 3).unit(), 0);
    std::vector<Polyhedron *> result;

    if (poly->slice(plane, result)) {
        result[0]->draw();
        result[1]->draw();
        delete result[0];
        delete result[1];
    } else {
        poly->draw();
    }

    delete poly;

The generated mesh for a polyhedron uses a shared set of vertices which each
polygon indexes into.  All polygons are generated in counterclockwise order.
