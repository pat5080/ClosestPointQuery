#include "closestpointquery.h"


ClosestPointQuery::ClosestPointQuery(const Vector3f& p, const Mesh* m, const float& maxDist)
{
    mesh = m;
    queryPoint = p;
    maxD = maxDist;
}

Vector3f ClosestPointQuery::distCalculation()
{
    std::vector<Vector3f> vertices = mesh->getMeshPoints();

    int closestVertexIndex = closestVertex(vertices);

    Vector3f vertexPoint = vertices.at(closestVertexIndex);

    Vector3f edgePoint = closestEdge(vertices, closestVertexIndex);

    Vector3f facePoint = closestFace(vertices, closestVertexIndex);

    Vector3f closestPoint = queryDist(vertexPoint, edgePoint, facePoint);

    return closestPoint;

}

int ClosestPointQuery::closestVertex(std::vector<Vector3f>& vertices)
{
    std::map<int, float> disPass1;

    for(auto it = begin(vertices); it != end(vertices); ++it)
    {

        Vector3f difference = queryPoint - *it;

        float distance = abs(difference.norm());

        if(distance <= maxD)
        {        
            int index = it - vertices.begin();

            disPass1.insert({ index, distance });
        }
    }

    std::pair<int,float> min = *min_element(disPass1.begin(), disPass1.end(), pairCompare);

    int closestVertexIndex = std::get<0>(min);
    float closestVertDist = std::get<0>(min);

    Vector3f point = vertices.at(closestVertexIndex);

    std::cout << "|Closest Vertex|\n" << "x: " << point.x() << " y: " << point.y() << " z: " << point.z() << std::endl;
    std::cout << "Distance: " << closestVertDist << std::endl;

    return closestVertexIndex;
}


Vector3f ClosestPointQuery::closestEdge(std::vector<Vector3f>& vertices, int& closestVertexindex)
{

    std::vector<float> distance;

    std::set<int> edgeList = edgeListSort(closestVertexindex);

    Vector3f vertex = vertices.at(closestVertexindex);

    std::vector<Vector3f> edgepoint;

    for(auto it = begin(edgeList); it!= end(edgeList); ++it)
    {

        int p = *it;

        Vector3f i = vertices.at(p);

        Vector3f q = queryPoint;

        Vector3f AB = i - vertex;
        Vector3f BE = q - i;
        Vector3f AE = q - vertex;

        float dotProduct1 = AB.dot(BE);
        float dotProduct2 = AB.dot(AE);

        if(dotProduct1 > 0)
        {

            float dist1 = AE.norm();
            distance.push_back(dist1);
            edgepoint.push_back(i);
        }
        else if(dotProduct2 < 0)
        {

            float dist = BE.norm();
            distance.push_back(dist);
            edgepoint.push_back(vertex);
        }
        else{

            float ACross = (AE.cross(AB)).norm();

            float ABn = AB.norm();

            float distn = ACross/ABn;

            abs(distn);

            distance.push_back(distn);

            Vector3f projected = vertex + AE.dot(AB)/ AB.dot(AB)*AB;

            edgepoint.push_back(projected);

        }
    }


    int index = std::distance(distance.begin(), std::min_element(distance.begin(), distance.end()));

    Vector3f pointOnEdge = edgepoint.at(index);

    float x = pointOnEdge.x();

    float y = pointOnEdge.y();

    float z = pointOnEdge.z();

    std::cout << "|Closest point on edge|\n" << "x: " << x << " y: " << y << " z: " << z << std::endl;

    float dist = distance.at(index);

    std::cout << "Distance: " << dist << std::endl;


    return pointOnEdge;
}


Vector3f ClosestPointQuery::closestFace(std::vector<Vector3f>& vertices, int& closestVertex)
{
    std::vector<Vector3i> faceList = faceSort(closestVertex);

    std::vector<float> distance;
    std::vector<Vector3f> facePoints;
    bool flagInBounds = true;

    for(auto it = begin(faceList); it != end(faceList); ++it)
    {

        Vector3i i = *it;

        int x = i.x();

        int y = i.y();

        int z = i.z();


        Vector3f a = vertices.at(x);

        Vector3f b = vertices.at(y);

        Vector3f c = vertices.at(z);

        Vector3f q = queryPoint;

        Vector3f BC = c - b;

        Vector3f AB = b - a;

        Vector3f AQ = q - a;

        Vector3f normal = BC.cross(AB);


        Vector3f projected = q - AQ.dot(normal)*normal;

        float dist = (q - projected).norm();

        bool flag = inBoundsCheck(vertices ,projected, i);

        if (flag)
        {

            distance.push_back(dist);
            facePoints.push_back(projected);

            flagInBounds = false;
        }
        else{

            if(flagInBounds)
            {

            Vector3f pointOnLine = castRayToShape(vertices, projected, i);

            Vector3f EQ = queryPoint - pointOnLine;

            float dist = EQ.norm();

            distance.push_back(dist);
            facePoints.push_back(pointOnLine);
            }

        }
    }

    int index = std::distance(distance.begin(), std::min_element(distance.begin(), distance.end()));

    Vector3f facePoint = facePoints.at(index);

    float x = facePoint.x();

    float y = facePoint.y();

    float z = facePoint.z();

    std::cout << "|Closest point on face|\n" << "x: " << x << " y: " << y << " z: " << z << std::endl;

    float distCal = (queryPoint - facePoint).norm();

    std::cout << "Distance: " << distCal << std::endl;

    return facePoint;

}

Vector3f ClosestPointQuery::queryDist(Vector3f& vertexPoint, Vector3f& edgePoint, Vector3f& facePoint)
{
    float edgeDist = (queryPoint - edgePoint).norm();

    float faceDist = (queryPoint - facePoint).norm();

    float vertDist = (queryPoint - vertexPoint).norm();

    float min = std::min({vertDist, edgeDist, faceDist});


    if(areSame(min, vertDist))
    {

        return vertexPoint;
    }

    else if(areSame(min, edgeDist))
    {
        return edgePoint;

    }

    else if(areSame(min, faceDist))
    {
        return facePoint;

    }
    else {
        std::cout << "Default vertex point is returned." << std::endl;
        return vertexPoint;
    }

}


bool ClosestPointQuery::inBoundsCheck(std::vector<Vector3f>& vertices, Vector3f &p, Vector3i &f)
{
    std::vector<Vector3f> vertices = mesh->getMeshPoints();

    int x = f.x();

    int y = f.y();

    int z = f.z();


    Vector3f a = vertices.at(x);

    Vector3f b = vertices.at(y);

    Vector3f c = vertices.at(z);


    float areaTotal = triArea(a, b, c);

    float area1 = triAreaP1(a, b, p);

    float area2 = triAreaP2(p, b, c);

    float area3 = triAreaP3(a, p, c);


    float areaSum = area1 + area2 + area3;

    bool flag = AreSame(areaTotal, areaSum);

    if(flag)
    {
        return true;
    }
    else{
        return false;
    }
}

Vector3f ClosestPointQuery::castRayToShape(std::vector<Vector3f>& vertices, Vector3f &p, Vector3i &f)
{

    int x = f.x();

    int y = f.y();

    int z = f.z();

    std::vector<Vector3f> edgepoint;

    std::vector<Vector3f> tri_verts;

    std::vector<Vector3f> tri_verts2;


    tri_verts.push_back(vertices.at(x));

    tri_verts.push_back(vertices.at(y));

    tri_verts2.push_back(vertices.at(y));

    tri_verts2.push_back(vertices.at(z));


    std::vector<float> distance;

    int count = 0;

    for(auto it = begin(tri_verts);  it != end(tri_verts); ++it)
    {
        for(auto ix = begin(tri_verts2); ix != end(tri_verts2); ++ix)
        {
            Vector3f i = *ix;

            Vector3f v = *it;

            count++;

            if(count == 3)
            {
                std::advance(ix, 1);
            }

            Vector3f AB = i - v;
            Vector3f BE = p - i;
            Vector3f AE = p - v;


            float dotProduct1 = AB.dot(BE);
            float dotProduct2 = AB.dot(AE);

            if(dotProduct1 > 0)
            {

                float dist1 = AE.norm();
                distance.push_back(dist1);
                edgepoint.push_back(i);
            }
            else if(dotProduct2 < 0)
            {

                float dist = BE.norm();
                distance.push_back(dist);
                edgepoint.push_back(v);

            }
            else{

                float ACross = (AB.cross(AE)).norm();

                float ABn = AB.norm();

                float distn = ACross/ABn;

                abs(distn);

                distance.push_back(distn);

                Vector3f projected = v + AE.dot(AB)/ AB.dot(AB)*AB;

                edgepoint.push_back(projected);

            }
        }
    }

    int index = std::distance(distance.begin(), std::min_element(distance.begin(), distance.end()));

    Vector3f point = edgepoint.at(index);

    return point;

}

bool ClosestPointQuery::pairCompare( std::pair<int,float> i, std::pair<int,float> j)
{
return i.second < j.second;
}

std::set<int> ClosestPointQuery::edgeListSort(int& vertex)
{
    int closestVertexindex = vertex;

    std::set<int> edgeList;
    std::vector<Vector3i> faces = mesh->getMeshFaces();

    for(auto it = begin(faces); it != end(faces); ++it)
    {

        Vector3i test = *it;

        int x = test.x();

        int y = test.y();

        int z = test.z();


        if( x == closestVertexindex || y == closestVertexindex || z == closestVertexindex)
        {

            if(x != closestVertexindex)
            {
                edgeList.insert(x);
            }

            if(y != closestVertexindex)
            {

                edgeList.insert(y);
            }

            if(z != closestVertexindex)
            {
                edgeList.insert(z);
            }
        }
    }

    return edgeList;

}

std::vector<Vector3i> ClosestPointQuery::faceSort(int &closestVertex)
{
    std::vector<Vector3i> faces = mesh->getMeshFaces();

    std::vector<Vector3i> facesort;

    for(auto it = begin(faces); it != end(faces); ++it)
    {

        Vector3i test = *it;

        int x = test.x();

        int y = test.y();

        int z = test.z();


        if( x == closestVertex || y == closestVertex || z == closestVertex)
        {
            facesort.push_back(test);
        }
    }

    return facesort;

}

float ClosestPointQuery::triArea(Vector3f a, Vector3f b, Vector3f c)
{
    Vector3f AC = c - a;

    Vector3f AB = b - a;

    Vector3f areav = AB.cross(AC);

    float area = areav.norm()/2;

    return area;

}

float ClosestPointQuery::triAreaP1(Vector3f a, Vector3f b, Vector3f p)
{

    Vector3f AP1 = p - a;

    Vector3f BP1 = p - b;

    Vector3f areav1 = BP1.cross(AP1);

    float area = areav1.norm()/2;

    return area;

}

float ClosestPointQuery::triAreaP2(Vector3f p, Vector3f b, Vector3f c)
{

    Vector3f BP2 = p - b;

    Vector3f CP2 = p - c;

    Vector3f areav2 = BP2.cross(CP2);

    float area = areav2.norm()/2;

    return area;

}

float ClosestPointQuery::triAreaP3(Vector3f a, Vector3f p, Vector3f c)
{

    Vector3f AP3 = p - a;

    Vector3f CP3 = p - c;

    Vector3f areav3 = AP3.cross(CP3);

    float area = areav3.norm()/2;

    return area;

}

bool ClosestPointQuery::areSame(float& a, float& b)
{
    return fabs(a - b) < EPISILON;
}








