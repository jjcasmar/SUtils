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
    struct MyTraits : public OpenMesh::DefaultTraits
    {
        typedef OpenMesh::Vec3d Point; // use double-values points
        typedef OpenMesh::Vec3d Normal;
    };

    typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Surface;
    typedef Surface::Point OMPoint;
    typedef Eigen::Matrix<double, 3, 1> Vector;
    typedef Eigen::Matrix<double, 2, 1> Vector2;
    typedef Eigen::Matrix<double, 3, 3> Matrix;
    typedef Eigen::Matrix<double, 3, 2> Matrix32;
    typedef Eigen::Matrix<double, 2, 2> Matrix2;

    TriMesh();
    TriMesh(const std::string &filename);
    TriMesh(const std::vector<Vector> &vertices, const std::vector<unsigned int> &facets);
    TriMesh(const std::vector<Vector> &vertices, const std::vector<Vector> &normals, const std::vector<unsigned int> &facets);
    TriMesh(const TriMesh &other);
    TriMesh(const std::vector<Vector2> &vertices, const std::vector<unsigned int> &edges, double aspect, double size);
//    TriMesh(const std::vector<Vector>&vertices, const std::vector<Vector>& normals, const std::vector<Vector> &uv, const std::vector<unsigned int> &facets);

    std::vector<Vector> points() const;
    std::vector<Vector> normals() const;
//    std::vector<Vector> uv();
    std::vector<unsigned int> facets() const;

    unsigned int nbVertices() const;
    unsigned int nbFaces() const;

    void setPoints(const std::vector<Vector> &points);

    std::vector<Vector> materialPoints() const;
    std::vector<Vector> materialNormals() const;
    std::vector<Matrix32> deformationGradients() const;
    std::vector<Matrix2> dm() const;
    std::vector<double> areas() const;
    std::vector<double> edgesBending() const;
    std::vector<Matrix32> restPoseInverseMatrix() const;

    void computeVertexNormals();
    void computeDeformationGradients();
    void computeEdgeBending();
    void computeAreas();

    void refine(double targetEdgeLength);

    std::pair<unsigned int, Vector> barycentricCoordinates(const Vector &point, Vector &bCoo);

    Surface *surface() const;

private:
    void initFromPointsAndFacets(const std::vector<Vector> &points, const std::vector<unsigned int> &facets);

    void computeFaceNormals();
    double faceArea(Surface::FaceHandle faceHandle);
    std::array<double, 6> dn(Surface::FaceHandle faceHandle);
    double bending(Surface::EdgeHandle edgeHandle);

    Surface *m_surface;

    Eigen::MatrixXd m_membrane_ru;
    Eigen::MatrixXd m_membrane_rv;

    OpenMesh::VPropHandleT<Surface::Point> m_materialPointVPH;
    OpenMesh::VPropHandleT<Surface::Point> m_materialNormalVPH;
    OpenMesh::FPropHandleT<Surface::Point> m_materialNormalFPH;
    OpenMesh::FPropHandleT<Matrix32> m_deformationGradientFPH;
    OpenMesh::FPropHandleT<double> m_areaFPH;
    OpenMesh::FPropHandleT<std::array<double, 6> > m_dnFPH;
    OpenMesh::FPropHandleT<Matrix2> m_dMFPH;
    OpenMesh::EPropHandleT<double> m_bendingEPH;
    OpenMesh::FPropHandleT<Matrix32> m_restPoseMatrixFPH;
};

#endif // TRIMESH_H
