#include "polyfrag.h"

#ifdef POLYFRAG_USE_OPENGL
#ifdef __APPLE_CC__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

#include <float.h>
#include <stdlib.h>
#include <algorithm>

inline float min(float a, float b) { return a < b ? a : b; }
inline float max(float a, float b) { return a < b ? a : b; }
inline float frand() { return (float)rand() / (float)RAND_MAX; }

// Append newVertex to vertices if it isn't already there, and return the index of newVertex in vertices
static int addVertex(const Vector3D &newVertex, std::vector<Vector3D> &vertices)
{
    unsigned int i;
    for (i = 0; i < vertices.size(); i++)
        if (vertices[i] == newVertex)
            return i;
    vertices.push_back(newVertex);
    return i;
}

////////////////////////////////////////////////////////////////////////////////
// class Plane
////////////////////////////////////////////////////////////////////////////////

int Plane::classify(const Vector3D &v) const
{
    float compare = normal.dot(v) - w;
    return (compare > EPSILON) ? FRONT : (compare < -EPSILON) ? BACK : COINCIDENT;
}

Vector3D Plane::lineIntersection(const Vector3D &start, const Vector3D &end) const
{
    Vector3D alongLine = end - start;
    float t = (w - start.dot(normal)) / alongLine.dot(normal);
    return start + alongLine * t;
}

////////////////////////////////////////////////////////////////////////////////
// class Polygon
////////////////////////////////////////////////////////////////////////////////

Polygon::Polygon()
{
}

Polygon::Polygon(int a, int b, int c)
{
    m_indices.push_back(a);
    m_indices.push_back(b);
    m_indices.push_back(c);
}

Polygon::Polygon(int a, int b, int c, int d)
{
    m_indices.push_back(a);
    m_indices.push_back(b);
    m_indices.push_back(c);
    m_indices.push_back(d);
}

Polygon::~Polygon()
{
}

#ifdef POLYFRAG_USE_OPENGL
void Polygon::draw(const std::vector<Vector3D> &vertices) const
{
    Vector3D a = vertices[m_indices[0]];
    Vector3D b = vertices[m_indices[1]];
    Vector3D c = vertices[m_indices[2]];
    Vector3D normal = (b - a).cross(c - a).unit();
    glNormal3f(normal.x, normal.y, normal.z);

    glBegin(GL_POLYGON);
    for (unsigned int i = 0; i < m_indices.size(); i++)
    {
        const Vector3D &v = vertices[m_indices[i]];
        glVertex3d(v.x, v.y, v.z);
    }
    glEnd();
}
#endif

int Polygon::classifyAndSlice(const std::vector<Vector3D> &vertices, const Plane &plane,
                              std::vector<Vector3D> &frontVertices, Polygon &front,
                              std::vector<Vector3D> &backVertices, Polygon &back,
                              std::vector<Vector3D> &pointsOnPlane) const
{
    bool frontUsed = false, backUsed = false;

    front.m_indices.clear();
    back.m_indices.clear();

    // Simplified algorithm for convex polygons only
    for (unsigned int i = 0; i < m_indices.size(); i++)
    {
        // Place the current vertex in either the front or back polygon (or both)
        int index = m_indices[i];
        int classification = plane.classify(vertices[index]);
        switch (classification)
        {
        case FRONT:
            front.m_indices.push_back(addVertex(vertices[index], frontVertices));
            frontUsed = true;
            break;

        case COINCIDENT:
            front.m_indices.push_back(addVertex(vertices[index], frontVertices));
            back.m_indices.push_back(addVertex(vertices[index], backVertices));
            addVertex(vertices[index], pointsOnPlane);
            break;

        case BACK:
            back.m_indices.push_back(addVertex(vertices[index], backVertices));
            backUsed = true;
            break;
        }

        // Add a vertex to both polygons where edges intersect the plane
        unsigned int nextIndex = m_indices[(i + 1) % m_indices.size()];
        int nextClassification = plane.classify(vertices[nextIndex]);
        if ((classification == FRONT && nextClassification == BACK) ||
            (classification == BACK && nextClassification == FRONT))
        {
            Vector3D edgeIntersection = plane.lineIntersection(vertices[index], vertices[nextIndex]);
            front.m_indices.push_back(addVertex(edgeIntersection, frontVertices));
            back.m_indices.push_back(addVertex(edgeIntersection, backVertices));
            addVertex(edgeIntersection, pointsOnPlane);
        }
    }

    if (frontUsed) return backUsed ? SPLIT : FRONT;
    else return backUsed ? BACK : COINCIDENT;
}

// Helper class to order vectors by their angle in an arbitrary coordinate system
class WindingOrdering
{
public:
    Vector3D origin;
    Vector3D xAxis;
    Vector3D yAxis;

    bool operator () (const Vector3D &a, const Vector3D &b) const
    {
        float angleA = atan2f(yAxis.dot(a - origin), xAxis.dot(a - origin));
        float angleB = atan2f(yAxis.dot(b - origin), xAxis.dot(b - origin));
        return angleA < angleB;
    }
};

Polygon Polygon::fromPoints(const std::vector<Vector3D> &pointsOnPlane, const Vector3D &planeNormal, std::vector<Vector3D> &vertices)
{
    // Construct a coordinate system on the polygon with the origin at the polygon center,
    // the x-axis from the center to the first point, and the y-axis perpendicular to the
    // x-axis and the plane normal
    Vector3D center;
    for (unsigned int i = 0; i < pointsOnPlane.size(); i++)
        center += pointsOnPlane[i];
    center /= pointsOnPlane.size();

    // Sort polygons by their angles with respect to this coordinate system
    WindingOrdering ordering;
    ordering.origin = center;
    ordering.xAxis = (pointsOnPlane[0] - center).unit();
    ordering.yAxis = ordering.xAxis.cross(planeNormal);
    std::vector<Vector3D> copy = pointsOnPlane;
    std::sort(copy.begin(), copy.end(), ordering);

    // Create a polygon with those points in that order
    Polygon polygon;
    for (unsigned int i = 0; i < copy.size(); i++)
        polygon.m_indices.push_back(addVertex(copy[i], vertices));
    return polygon;
}

////////////////////////////////////////////////////////////////////////////////
// class Polyhedron
////////////////////////////////////////////////////////////////////////////////

Polyhedron::Polyhedron()
{
}

Polyhedron::~Polyhedron()
{
}

#ifdef POLYFRAG_USE_OPENGL
void Polyhedron::draw() const
{
    for (unsigned int i = 0; i < m_polygons.size(); i++)
        m_polygons[i].draw(m_vertices);
}
#endif

Vector3D Polyhedron::getCentroid() const
{
    Vector3D center;
    for (unsigned int i = 0; i < m_vertices.size(); i++)
        center += m_vertices[i];
    return center / m_vertices.size();
}

void Polyhedron::getAABB(Vector3D &minCoord, Vector3D &maxCoord) const
{
    minCoord.x = FLT_MAX;
    minCoord.y = FLT_MAX;
    minCoord.z = FLT_MAX;
    maxCoord.x = -FLT_MAX;
    maxCoord.y = -FLT_MAX;
    maxCoord.z = -FLT_MAX;

    for (unsigned int i = 0; i < m_vertices.size(); i++)
    {
        const Vector3D &vertex = m_vertices[i];
        minCoord.x = min(minCoord.x, vertex.x);
        minCoord.y = min(minCoord.y, vertex.y);
        minCoord.z = min(minCoord.z, vertex.z);
        maxCoord.x = max(maxCoord.x, vertex.x);
        maxCoord.y = max(maxCoord.y, vertex.y);
        maxCoord.z = max(maxCoord.z, vertex.z);
    }
}

bool Polyhedron::slice(const Plane &plane, std::vector<Polyhedron *> &result) const
{
    std::vector<Vector3D> pointsOnPlane;
    Polyhedron *frontPolyhedron = new Polyhedron();
    Polyhedron *backPolyhedron = new Polyhedron();
    bool frontUsed = false, backUsed = false;

    for (unsigned int i = 0; i < m_polygons.size(); i++)
    {
        Polygon front, back;
        switch (m_polygons[i].classifyAndSlice(m_vertices, plane, frontPolyhedron->m_vertices, front, backPolyhedron->m_vertices, back, pointsOnPlane))
        {
        case FRONT:
            frontPolyhedron->m_polygons.push_back(front);
            frontUsed = true;
            break;

        case COINCIDENT:
        case SPLIT:
            frontPolyhedron->m_polygons.push_back(front);
            backPolyhedron->m_polygons.push_back(back);
            break;

        case BACK:
            backPolyhedron->m_polygons.push_back(back);
            backUsed = true;
            break;
        }
    }

    result.clear();
    if (frontUsed == backUsed)
    {
        // Create polygons to fill in the holes inside the new polyhedra
        frontPolyhedron->m_polygons.push_back(Polygon::fromPoints(pointsOnPlane, plane.normal, frontPolyhedron->m_vertices));
        backPolyhedron->m_polygons.push_back(Polygon::fromPoints(pointsOnPlane, -plane.normal, backPolyhedron->m_vertices));
        result.push_back(frontPolyhedron);
        result.push_back(backPolyhedron);
        return true;
    }
    else
    {
        delete frontPolyhedron;
        delete backPolyhedron;
        return false;
    }
}

Polyhedron *Polyhedron::box(const Vector3D &min, const Vector3D &max)
{
    Polyhedron *cube = new Polyhedron();
    cube->m_vertices.push_back(Vector3D(min.x, min.y, min.z));
    cube->m_vertices.push_back(Vector3D(min.x, min.y, max.z));
    cube->m_vertices.push_back(Vector3D(min.x, max.y, min.z));
    cube->m_vertices.push_back(Vector3D(min.x, max.y, max.z));
    cube->m_vertices.push_back(Vector3D(max.x, min.y, min.z));
    cube->m_vertices.push_back(Vector3D(max.x, min.y, max.z));
    cube->m_vertices.push_back(Vector3D(max.x, max.y, min.z));
    cube->m_vertices.push_back(Vector3D(max.x, max.y, max.z));
    cube->m_polygons.push_back(Polygon(0, 1, 3, 2));
    cube->m_polygons.push_back(Polygon(4, 6, 7, 5));
    cube->m_polygons.push_back(Polygon(0, 4, 5, 1));
    cube->m_polygons.push_back(Polygon(2, 3, 7, 6));
    cube->m_polygons.push_back(Polygon(0, 2, 6, 4));
    cube->m_polygons.push_back(Polygon(1, 5, 7, 3));
    return cube;
}

void Polyhedron::recursiveSlice(Polyhedron *polyhedron, std::vector<Polyhedron *> &polyhedra, int depth)
{
    for (int i = 0; i < 10; i++)
    {
        // Generate a random plane about the centroid of polyhedron
        Vector3D normal = Vector3D(frand() * 2 - 1, frand() * 2 - 1, frand() * 2 - 1).unit();
        Plane plane(normal, polyhedron->getCentroid().dot(normal));

        // Attempt to split polyhedron by the random plane
        std::vector<Polyhedron *> result;
        if (polyhedron->slice(plane, result))
        {
            if (depth)
            {
                recursiveSlice(result[0], polyhedra, depth - 1);
                recursiveSlice(result[1], polyhedra, depth - 1);
            }
            else
            {
                polyhedra.push_back(result[0]);
                polyhedra.push_back(result[1]);
            }
            delete polyhedron;
            return;
        }
    }

    // Couldn't slice polyhedron, just append it to the output
    polyhedra.push_back(polyhedron);
}
