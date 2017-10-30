#include "../../core/trimesh.h"
#include <iostream>

int main (int argc, char **argv) {

    std::vector<TriMesh::Vector> points;
    std::vector<unsigned int> indices;

    points.push_back(TriMesh::Vector(0,0,0));
    points.push_back(TriMesh::Vector(1,0,0));
    points.push_back(TriMesh::Vector(0,1,0));

    indices.push_back(0);
    indices.push_back(1);
    indices.push_back(2);

    TriMesh::Matrix I3;;
    TriMesh::Matrix2 I2;
    I3.setIdentity();
    I2.setIdentity();

    TriMesh mesh(points, indices);
    mesh.computeDeformationGradients();
    Eigen::Matrix<double, 3, 2> F2 = mesh.deformationGradients()[0];
    Eigen::Matrix<double, 2, 2> E2 = 0.5*(F2.transpose()*F2 - I2);

    std::cout << F2 << std::endl;
    std::cout << std::endl;
    std::cout << E2 << std::endl;

    std::cout << std::endl;
    std::cout << std::endl;

    points[0][1] = 4;
    points[1][2] = 7;
    points[2][0] = 2;
    mesh.setPoints(points);
    mesh.computeDeformationGradients();
    Eigen::Matrix<double, 3, 2> F3 = mesh.deformationGradients()[0];
    Eigen::Matrix<double, 2, 2> E3 = 0.5*(F3.transpose()*F3 - I2);
    std::cout << F3 << std::endl;
    std::cout << std::endl;
    std::cout << E3 << std::endl;

    return 0;
}
