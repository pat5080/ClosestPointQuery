#ifndef CLOSESTPOINTQUERY_H
#define CLOSESTPOINTQUERY_H

#include <iostream>
#include "mesh.h"
#include <cmath>
#include <algorithm>
#include "eigen-3.3.9/Eigen/Geometry"
#include <map>
#include <set>

#define EPISILON 0.005


using namespace Eigen;

/*!
 * \brief       The ClosestPointQuery class
 * \details     This class takes a Eigen::Vector3f of point p, a pointer to an object of class Mesh and a maximum distance parameter.
 *              The class uses these parameters to compute the closest point on the mesh defined by a ply file. It contains
 *              functions that perform computations on a mesh, whereby three functions return the closest distance from the query
 *              point to a chosen vertex, edge and triangular face. Other functions run checks and vector algebra computations that
 *              support the process of finding the closest point on the mesh. An entry point function is used to return the final computed
 *              closest point on the mesh, which is accessed from main.cpp.
 *
 *  \author     Patrick Korczak
 *  \version    1.0
 *  \date       2021
 *  \bug        none reported
 *
 */

class ClosestPointQuery
{
public:

    ///
    /// \brief          ClosestPointQuery
    /// \details        The constructor takes a Eigen::Vector3f point p, const pointer to Mesh and float of max distance
    /// \param          p
    /// \param          m
    /// \param          maxDist
    ///

    ClosestPointQuery(const Vector3f& p, const Mesh* m, const float& maxDist);

    ///
    /// \brief          distCalculation
    /// \details        This function calls other class functions to calculate the shortest distance on the mesh, whether it's on a vertex,
    ///                 edge or on a trianglar face. It returns the closest distance from a point on the mesh to the query point.
    /// \return         The closest point on the mesh.
    ///

    Vector3f distCalculation();

private:

    const Mesh* mesh;   //! const pointer to mesh object of class Mesh that is used to access Mesh data

    float maxD;         //! Class variable storing the maximum search distance variable

    Vector3f queryPoint;    //! Class variable storing the query point in 3D space

    ///
    /// \brief          closestVertex
    /// \details        This function accesses the vertex data from the pointer to Mesh object. The distance between the
    ///                 query point and vertices is computed against the maximum search distance. The vertices within the
    ///                 max distance search area are stored and their distances are calculated to find the closest vertex.
    ///                 The distance to the closest vertex and its index is found and stored in a std::pair container.
    /// \return         An integer that references the index of the closest vertex.
    ///

    int closestVertex(std::vector<Vector3f>& vertices);

    ///
    /// \brief          closestEdge
    /// \details        This function takes a list of vertices. It calculates the closest distance from the query point to the edge shared by the closest vertex
    ///                 and the neighbouring vertices. The closest distance may be on any of the two vertices that form the edge line, or a point
    ///                 on the edge itself.
    /// \param          edgeList
    /// \param          closestVertexindex
    /// \return         The closest point on the edge
    ///

    Vector3f closestEdge(std::vector<Vector3f>& vertices, int& closestVertex);

    ///
    /// \brief          closestFace
    /// \details        The function takes a list of faces that share the closest vertex. It computes the normal vector of each triangular face
    ///                 which is used to project the query point onto the plane of the triangular face. Checks are run to see whether the projected
    ///                 point is on the triangular face. If it isn't then the closest distance to the closest edge of the triangle is computed and
    ///                 returned.
    /// \param          vector holding Vector3f eigen type vertices
    /// \param          index of closest vertex
    /// \return         The closest point on the face
    ///

    Vector3f closestFace(std::vector<Vector3f>& vertices, int& closestVertex);

    ///
    /// \brief          inBoundsCheck
    /// \details        This function checks whether the projected point is within the actual triangular mesh face in question.
    /// \param          vector holding Vector3f eigen type vertices
    /// \param          p
    /// \param          f
    /// \return         bool
    ///

    bool inBoundsCheck(std::vector<Vector3f>& vertices, Vector3f & p, Vector3i & f);

    ///
    /// \brief          castRayToShape
    /// \details        This function calculates the closest distance to the edges of the triangular face from the projected point
    /// \param          vector holding Vector3f eigen type vertices
    /// \param          p
    /// \param          f
    /// \return         Point on triangular edge
    ///

    Vector3f castRayToShape(std::vector<Vector3f>& vertices, Vector3f & p, Vector3i & f);

    ///
    /// \brief          pairCompare
    /// \details        This function is used to compute the smallest value in the map of vertex distances.
    /// \param          vector holding Vector3f eigen type vertices
    /// \param          i
    /// \param          j
    /// \return         bool
    ///

    static bool pairCompare( std::pair<int,float> i, std::pair<int,float> j);

    ///
    /// \brief          edgeListSort
    /// \details        This function sorts the index to the vertices that share an edge with the closest vertex. A set is used to
    ///                 eliminate data duplication.
    /// \param          vertex
    /// \return         std::set<int>
    ///

    std::set<int> edgeListSort(int& vertex);

    ///
    /// \brief          faceSort
    /// \details        This function is used to return a list of triangular faces that share the closest vertex
    /// \param          closestVertex
    /// \return         std::set<int>
    ///

    std::vector<Vector3i> faceSort(int &closestVertex);

    ///
    /// \brief          triArea
    /// \details        This function calculates the area of a triangle given its three vertices
    /// \param          a
    /// \param          b
    /// \param          c
    /// \return         float area
    ///

    float triArea(Vector3f a, Vector3f b, Vector3f c);

    ///
    /// \brief          triAreaP1
    /// \details        triAreaP1, triAreaP2 and triAreaP3 are used calculate three segments of a triangle involving the projected point.
    /// \param          p
    /// \param          b
    /// \param          c
    /// \return         float
    ///

    float triAreaP1(Vector3f p, Vector3f b, Vector3f c);

    float triAreaP2(Vector3f p, Vector3f b, Vector3f c);

    float triAreaP3(Vector3f p, Vector3f b, Vector3f c);

    ///
    /// \brief          areSame
    /// \details        This function compares two floating point numbers to check if they are equivalent. This function is required because
    ///                 a tolerance is introduced when using floating point numbers.
    /// \param          a
    /// \param          b
    /// \return         bool
    ///

    bool areSame(float& a, float& b);

    Vector3f queryDist(Vector3f& vertexPoint, Vector3f& edgePoint, Vector3f& facePoint);


};

#endif // CLOSESTPOINTQUERY_H
