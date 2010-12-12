#ifndef POLYFRAG_H
#define POLYFRAG_H

/**
 * The PolyFrag Library
 *
 * This library contains a stable algorithm for splitting a convex polyhedron
 * into two pieces on either side of a 3D plane.  It is useful for generating
 * polyhedron fragments to make a polyhedron look like it's breaking apart.
 */

// Define POLYFRAG_USE_OPENGL to use OpenGL for rendering polyhedra.  Adds a
// draw() method to Polygon and Polyhedron.
#define POLYFRAG_USE_OPENGL

#include <vector>
#include <math.h>

#define EPSILON 0.0001
enum { FRONT, COINCIDENT, BACK, SPLIT };

/**
 * Used for representing 3D vectors and points, overloads standard arithmetic
 * operators.
 */

class Vector3D
{
public:
    float x, y, z;

    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    // Unary operators
    Vector3D operator + () const { return *this; }
    Vector3D operator - () const { return Vector3D(-x, -y, -z); }

    // Binary arithmetic operators
    Vector3D operator + (const Vector3D &v) const { return Vector3D(x + v.x, y + v.y, z + v.z); }
    Vector3D operator - (const Vector3D &v) const { return Vector3D(x - v.x, y - v.y, z - v.z); }
    Vector3D operator * (float f) const { return Vector3D(x * f, y * f, z * f); }
    Vector3D operator / (float f) const { return Vector3D(x / f, y / f, z / f); }

    // In-place arithmetic operators
    void operator += (const Vector3D &v) { x += v.x; y += v.y; z += v.z; }
    void operator -= (const Vector3D &v) { x -= v.x; y -= v.y; z -= v.z; }
    void operator *= (const float f) { x *= f; y *= f; z *= f; }
    void operator /= (const float f) { x /= f; y /= f; z /= f; }

    // Binary boolean operators
    bool operator == (const Vector3D &v) const { return fabsf(x - v.x) < EPSILON && fabsf(y - v.y) < EPSILON && fabsf(z - v.z) < EPSILON; }
    bool operator != (const Vector3D &v) const { return !(*this == v); }

    void normalize() { *this /= length(); }
    Vector3D unit() const { return *this / length(); }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    float dot(const Vector3D &v) const { return x*v.x + y*v.y + z*v.z; }
    Vector3D cross(const Vector3D &v) const { return Vector3D(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
};

/**
 * Represents a plane in 3D space.  To construct a plane about a point, set
 * w = normal.dot(point).
 */

class Plane
{
public:
    Vector3D normal;
    float w;

    Plane() : w(0) {}
    Plane(const Vector3D &_normal, float _w) : normal(_normal), w(_w) {}
    Plane(float x, float y, float z, float _w) : normal(x, y, z), w(_w) {}

    /**
     * Determine whether a point is in front of, in back of, or coincident with
     * this plane.  Coincidence is determined within a small epsilon value for
     * stability (there is a thin range of points that count as being coincident
     * with a plane instead of just the one mathematically correct value).
     *
     * @returns one of: FRONT, COINCIDENT, BACK
     */
    int classify(const Vector3D &v) const;

    /**
     * @returns the intersection point of the infinite line extending though
     * start and end.
     */
    Vector3D lineIntersection(const Vector3D &start, const Vector3D &end) const;
};

/**
 * Each polygon represents a face of a polyhedron. A polygon contains a set of
 * indices which are meaningless without the array of vertices into which they
 * index (this array of vertices exists in the parent Polyhedron object).  All
 * vertices to which a polygon refers are assumed to be coplanar.  The
 * algorithm also assumes all polygons are convex, so concave polygons will
 * likely cause the algorithm to fail.
 */

class Polygon
{
public:
    Polygon();
    Polygon(int a, int b, int c);
    Polygon(int a, int b, int c, int d);
    ~Polygon();

    std::vector<int> m_indices;

#ifdef POLYFRAG_USE_OPENGL
    void draw(const std::vector<Vector3D> &vertices) const;
#endif

    /**
     * Determine whether this polygon is in front of, in back of, coincident with
     * or split by a plane.  Returns by reference up to two polygons, which are
     * either copies of this polygon (if this polygon is not split) or both pieces
     * of this polygon (if this polygon is split).  Also add any new vertices
     * created while splitting to pointsOnPlane.
     *
     * @returns one of:
     * - FRONT (front will be filled in with this polygon)
     * - BACK (back will be filled in with this polygon)
     * - COINCIDENT (front and back will be filled in with this polygon)
     * - SPLIT (front and back will be filled in with this polygon split by plane)
     */
    int classifyAndSlice(const std::vector<Vector3D> &vertices, const Plane &plane,
                         std::vector<Vector3D> &frontVertices, Polygon &front,
                         std::vector<Vector3D> &backVertices, Polygon &back,
                         std::vector<Vector3D> &pointsOnPlane) const;

    /**
     * Given a set of points on a plane and a plane normal, construct a polygon
     * with the correct vertex order (counterclockwise), storing any new vertices
     * in the array of vertices.
     *
     * @returns the constructed polygon using pointsOnPlane in counterclockwise
     * order with respect to planeNormal
     */
    static Polygon fromPoints(const std::vector<Vector3D> &pointsOnPlane, const Vector3D &planeNormal, std::vector<Vector3D> &vertices);
};

/**
 * Each polyhedron is represented by an array of vertices and a set of polygons
 * which store indices into that vertex array.  The algorithm assumes all
 * polyhedra are convex, so concave polyhedra will likely cause the algorithm
 * to fail.
 *
 * Example usage:
 *
 * @code
 * srand(1);
 * std::vector<Polyhedron *> polyhedra;
 * Polyhedron::recursiveSlice(Polyhedron::box(Vector3D(-0.5, -2, -0.5), Vector3D(0.5, 2, 0.5)), polyhedra, 4);
 *
 * for (int i = 0; i < polyhedra.size(); i++)
 * {
 *     Vector3D offset = polyhedra[i]->getCentroid() * 0.5;
 *     glPushMatrix();
 *     glTranslatef(offset.x, offset.y, offset.z);
 *     polyhedra[i]->draw();
 *     glPopMatrix();
 *     delete polyhedra[i];
 * }
 * @endcode
 */

class Polyhedron
{
public:
    Polyhedron();
    ~Polyhedron();

    std::vector<Vector3D> m_vertices;
    std::vector<Polygon> m_polygons;

#ifdef POLYFRAG_USE_OPENGL
    void draw() const;
#endif

    /**
     * @return the average vertex position.
     */
    Vector3D getCentroid() const;

    /**
     * Return by reference the minimum and maximum coordinates of the vertices.
     * This defines an axis-aligned bounding-box around the polyhedron.
     */
    void getAABB(Vector3D &minCoord, Vector3D &maxCoord) const;

    /**
     * Attempt to slice this polyhedron by plane, filling in result with the
     * result of the slice on success.  This polyhedron is not modified.
     *
     * @returns true when the slice succeeds, false when it fails.
     */
    bool slice(const Plane &plane, std::vector<Polyhedron *> &result) const;

    /**
     * @returns an axis-aligned 3D box (polyhedron with 6 sides) with vertex
     * coordinates that extend from min to max.
     */
    static Polyhedron *box(const Vector3D &min = Vector3D(-1, -1, -1), const Vector3D &max = Vector3D(+1, +1, +1));

    /**
     * Attempts to recursively slice polyhedron into 2^(depth + 1) pieces,
     * returning the results by reference in polyhedra.  Will either delete
     * polyhedron if the split succeeded or add polyhedron to polyhedra if
     * it failed.
     */
    static void recursiveSlice(Polyhedron *polyhedron, std::vector<Polyhedron *> &polyhedra, int depth);
};

#endif
