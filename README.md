This library contains a stable algorithm for splitting a convex polyhedron
into two pieces on either side of a 3D plane.  It is useful for generating
polyhedron fragments to make a polyhedron look like it's breaking apart.

![](https://github.com/evanw/polyfrag/raw/master/polyfrag-box.png)

![](https://github.com/evanw/polyfrag/raw/master/polyfrag-sphere.png)

Example usage:

    srand(1);
    std::vector<Polyhedron *> polyhedra;
    Polyhedron::recursiveSlice(Polyhedron::box(Vector3D(-0.5, -2, -0.5), Vector3D(0.5, 2, 0.5)), polyhedra, 4);

    for (int i = 0; i < polyhedra.size(); i++)
    {
        Vector3D offset = polyhedra[i]->getCentroid() * 0.5;
        glPushMatrix();
        glTranslatef(offset.x, offset.y, offset.z);
        polyhedra[i]->draw();
        glPopMatrix();
        delete polyhedra[i];
    }

The generated mesh for a polyhedron uses a shared set of vertices which each
polygon indexes into.  All polygons are generated in counterclockwise order.
