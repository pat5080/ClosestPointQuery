#include <iostream>
#include "closestpointquery.h"
#include "mesh.h"

using namespace std;


int main()
{
    Vector3f point;

    float x, y, z;

    int choice;

    std::cout << "Enter (1) to input custom query point coordinates.\nEnter (2) to run query point (1.5f, 1.5f, 1.5f)." << std::endl;

    cin >> choice;

    if(choice == 1)
    {

        std::cout << "|Please enter the query point as floating point numbers|\n" << "x-coordinate:" << std::endl;

        cin >> x;

        std::cout << "y-coordinate:" << std::endl;

        cin >> y;

        std::cout << "z-coordinate:" << std::endl;

        cin >> z;

    }
    else {

        x = 1.5f;

        y = 1.5f;

        z = 1.5f;

        }

    point << x, y, z;

    float maxDist;

    std::cout << "Enter maximum search distance as a floating point number: " << std::endl;

    std::cin >> maxDist;

    std::cout << "Maximum search distance: " << maxDist << std::endl;

    const std::string filename = "../assets/icosahedron.ply";

    const Mesh* m = new Mesh(filename);

    ClosestPointQuery* Q = new ClosestPointQuery(point, m, maxDist);

    Vector3f minPoint = Q->distCalculation();

    float c = minPoint.x();

    float v = minPoint.y();

    float b = minPoint.z();

    std::cout << "|Closest point on mesh|\n" << "x: " << c << " y: " << v << " z: " << b << std::endl;

    delete Q;

    delete m;

    return 0;
}

