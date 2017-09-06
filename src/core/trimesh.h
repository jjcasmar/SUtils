#ifndef TRIMESH_H
#define TRIMESH_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <string.h>
#include <Eigen/Core>

class TriMesh
{
public:
    typedef OpenMesh::TriMesh_ArrayKernelT<> Surface;

private:
    typedef Surface::Point CPoint;
    typedef Eigen::Matrix<double, 3, 1> Vector;

public:
    TriMesh();
    TriMesh(const std::string &filename);
    TriMesh(const std::vector<Vector> &vertices, const std::vector<unsigned int> &facets);
    TriMesh(const std::vector<Vector> &vertices, const std::vector<Vector> &normals, const std::vector<unsigned int> &facets);
//    TriMesh(const std::vector<Vector>&vertices, const std::vector<Vector>& normals, const std::vector<Vector> &uv, const std::vector<unsigned int> &facets);

    std::vector<Vector> points() const;
    std::vector<Vector> normals() const;
//    std::vector<Vector> uv();
    std::vector<unsigned int> facets() const;

    void computeVertexNormals();

//    template <class T2>
//    std::vector<T2> vertexProperty(const std::string &property);

//    template <class T2>
//    std::vector<T2> facetProperty(const std::string &property);

    Surface *surface() const;

private:
    Surface *m_surface;
};

#endif // TRIMESH_H
