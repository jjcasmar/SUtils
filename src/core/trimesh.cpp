#include "trimesh.h"

#include <Eigen/Dense>
#include <limits>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Surface_mesh.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangle_3<K> CTriangle;
typedef CGAL::Point_3<K> CPoint;
typedef CGAL::Surface_mesh<CPoint> CSurface;

TriMesh::TriMesh()
    : m_surface(new Surface)
{
    computeFaceNormals();
    computeVertexNormals();

    m_surface->add_property(m_materialPointVPH);
    m_surface->add_property(m_materialNormalVPH);
    m_surface->add_property(m_materialNormalFPH);
    m_surface->add_property(m_deformationGradientFPH);
    m_surface->add_property(m_areaFPH);
}

TriMesh::TriMesh(const std::string &filename)
    : m_surface( new Surface)
{
    OpenMesh::IO::read_mesh(*m_surface, filename);

    m_surface->request_vertex_normals();
    m_surface->request_face_normals();

    computeFaceNormals();
    computeVertexNormals();

    m_surface->add_property(m_materialPointVPH);
    m_surface->add_property(m_materialNormalVPH);
    m_surface->add_property(m_materialNormalFPH);
    m_surface->add_property(m_deformationGradientFPH);
    m_surface->add_property(m_areaFPH);

    auto vBegin = m_surface->vertices_begin();
    auto vEnd = m_surface->vertices_end();
    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
        m_surface->property(m_materialPointVPH, *vIt) = m_surface->point(*vIt);
        m_surface->property(m_materialNormalVPH, *vIt) = m_surface->normal(*vIt);
    }

    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        m_surface->property(m_materialNormalFPH, *fIt) = m_surface->normal(*fIt);
        m_surface->property(m_deformationGradientFPH, *fIt) = Matrix::Identity();
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
        m_surface->property(m_dnFPH, *fIt) = dn(*fIt);
    }
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<unsigned int> &facets)
    : m_surface(new Surface)
{
    std::vector<Surface::VertexHandle> vertexHandles(vertices.size());
    for (uint i = 0; i < vertices.size(); ++i) {
        Vector p = vertices[i];
        vertexHandles[i] = m_surface->add_vertex(OMPoint(p[0], p[1], p[2]));
    }

    for (uint i = 0; i < facets.size()/3; ++i) {
        m_surface->add_face(vertexHandles[facets[3*i+0]],
                vertexHandles[facets[3*i+1]],
                vertexHandles[facets[3*i+2]]);
    }

    m_surface->request_vertex_normals();
    m_surface->request_face_normals();

    computeFaceNormals();
    computeVertexNormals();

    m_surface->add_property(m_materialPointVPH);
    m_surface->add_property(m_materialNormalVPH);
    m_surface->add_property(m_materialNormalFPH);
    m_surface->add_property(m_deformationGradientFPH);
    m_surface->add_property(m_areaFPH);
    m_surface->add_property(m_dnFPH);

    auto vBegin = m_surface->vertices_begin();
    auto vEnd = m_surface->vertices_end();
    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
        m_surface->property(m_materialPointVPH, *vIt) = m_surface->point(*vIt);
        m_surface->property(m_materialNormalVPH, *vIt) = m_surface->normal(*vIt);
    }

    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        m_surface->property(m_materialNormalFPH, *fIt) = m_surface->normal(*fIt);
        m_surface->property(m_deformationGradientFPH, *fIt) = Matrix::Identity();
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
        m_surface->property(m_dnFPH, *fIt) = dn(*fIt);
    }
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<TriMesh::Vector> &normals, const std::vector<unsigned int> &facets) :
    TriMesh(vertices, facets)
{
    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        OMPoint n; auto v = normals[i->idx()];
        n[0] = v[0];
        n[1] = v[1];
        n[2] = v[2];
        m_surface->set_normal(*i, n);
    }
}

//TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<TriMesh::Vector> &normals, const std::vector<TriMesh::Vector> &uv, const std::vector<unsigned int> &facets) :
//    TriMesh(vertices, normals, facets)
//{
//}

std::vector<TriMesh::Vector> TriMesh::points() const
{
    std::vector<TriMesh::Vector> vertices(m_surface->n_vertices());

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        Vector p;
        p[0] = m_surface->point(*it)[0];
        p[1] = m_surface->point(*it)[1];
        p[2] = m_surface->point(*it)[2];

        vertices[it->idx()] = p;
    }

    return vertices;
}

std::vector<TriMesh::Vector> TriMesh::normals() const
{
    std::vector<TriMesh::Vector> normals(m_surface->n_vertices());

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        Vector p;
        p[0] = m_surface->normal(*it)[0];
        p[1] = m_surface->normal(*it)[1];
        p[2] = m_surface->normal(*it)[2];

        normals[it->idx()] = p;
    }

    return normals;
}

//std::vector<TriMesh::Vector> TriMesh::uv()
//{

//}

std::vector<unsigned int> TriMesh::facets() const
{
    std::vector<unsigned int> facets(3*m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    unsigned int idx = 0;
    for (auto fIt = i; fIt != e; ++fIt) {
        for (auto fv_c = m_surface->fv_begin(*fIt); fv_c.is_valid(); ++fv_c) {
            facets[idx++] = fv_c->idx();
        }
    }

    return facets;
}

unsigned int TriMesh::nbVertices() const
{
    return m_surface->n_vertices();
}

unsigned int TriMesh::nbFaces() const
{
    return m_surface->n_faces();
}

void TriMesh::setPoints(const std::vector<TriMesh::Vector> &points)
{
    auto vBegin = m_surface->vertices_begin();
    auto vEnd = m_surface->vertices_end();
    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
        int idx = vIt->idx();
        Vector p = points[idx];
        OMPoint omPoint(p[0], p[1], p[2]);
        m_surface->set_point(*vIt, omPoint);
    }
}

std::vector<TriMesh::Vector> TriMesh::materialPoints() const
{
    std::vector<TriMesh::Vector> d(m_surface->n_vertices());

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        OMPoint omPoint = m_surface->property(m_materialPointVPH, *it);
        Vector p;
        p[0] = omPoint[0];
        p[1] = omPoint[1];
        p[2] = omPoint[2];

        d[it->idx()] = p;
    }

    return d;
}

std::vector<TriMesh::Vector> TriMesh::materialNormals() const
{
    std::vector<TriMesh::Vector> d(m_surface->n_vertices());

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        OMPoint omPoint = m_surface->property(m_materialNormalVPH, *it);
        Vector p;
        p[0] = omPoint[0];
        p[1] = omPoint[1];
        p[2] = omPoint[2];

        d[it->idx()] = p;
    }

    return d;
}

std::vector<TriMesh::Matrix> TriMesh::deformationGradients() const
{
    std::vector<Matrix> d(m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    for (auto it = i; it != e; ++it) {
        Matrix m = m_surface->property(m_deformationGradientFPH, *it);
        d[it->idx()] = m;
    }

    return d;
}

std::vector<double> TriMesh::areas() const
{
    std::vector<double> d(m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    for (auto it = i; it != e; ++it) {
        double m = m_surface->property(m_areaFPH, *it);
        d[it->idx()] = m;
    }

    return d;
}

void TriMesh::computeFaceNormals()
{
    auto fb = m_surface->faces_begin();
    auto fe = m_surface->faces_end();
    for (auto fIt = fb; fIt != fe; ++fIt) {
        Surface::Normal n = m_surface->calc_face_normal(*fIt);
        m_surface->set_normal(*fIt, n);
    }
}

double TriMesh::faceArea(Surface::FaceHandle faceHandle)
{
    auto cfv_it = m_surface->cfv_begin(faceHandle);
    int idx = 0;
    CPoint cgtp[3];

    for (; cfv_it.is_valid(); ++cfv_it, ++idx) {
        OMPoint omPoint = m_surface->point(*cfv_it);
        cgtp[idx] = CPoint(omPoint[0], omPoint[1], omPoint[2]);
    }

    CTriangle tri(cgtp[0], cgtp[1], cgtp[2]);

    return std::sqrt(tri.squared_area());
}

std::array<double, 6> TriMesh::dn(Surface::FaceHandle faceHandle)
{
    auto cfv_it = m_surface->cfv_begin(faceHandle);

    OMPoint cp[3];
    cp[0] = m_surface->point(*cfv_it); cfv_it++;
    cp[1] = m_surface->point(*cfv_it); cfv_it++;
    cp[2] = m_surface->point(*cfv_it); cfv_it++;

    Vector p[3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            p[i][j] = cp[i][j];

    auto m_a = p[0];
    auto m_b = p[1];
    auto m_c = p[2];

    std::array<double, 6> dn;
    double A = 1.0/(2.0*m_surface->property(m_areaFPH, faceHandle));
    dn[0] = A * (m_b[1] - m_c[1]);
    dn[1] = A * (m_c[0] - m_b[0]);
    dn[2] = A * (m_c[1] - m_a[1]);
    dn[3] = A * (m_a[0] - m_c[0]);
    dn[4] = A * (m_a[1] - m_b[1]);
    dn[5] = A * (m_b[0] - m_a[0]);

    return dn;
}

void TriMesh::computeVertexNormals()
{
    computeFaceNormals();

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        m_surface->set_normal(*it, m_surface->calc_vertex_normal(*it));
    }
}

void TriMesh::computeDeformationGradients()
{
    Matrix F;
    F.setZero();

    OMPoint m_a, m_b, m_c;

    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        auto cfv_it = m_surface->cfv_begin(*fIt);
        m_a = m_surface->point(*cfv_it); cfv_it++;
        m_b = m_surface->point(*cfv_it); cfv_it++;
        m_c = m_surface->point(*cfv_it); cfv_it++;

        std::array<double, 6> dn = m_surface->property(m_dnFPH, *fIt);
        double dN1dx = dn[0];
        double dN1dy = dn[1];
        double dN2dx = dn[2];
        double dN2dy = dn[3];
        double dN3dx = dn[4];
        double dN3dy = dn[5];

        F << dN1dx*m_a[0] + dN2dx*m_b[0] + dN3dx*m_c[0], dN1dy*m_a[0] + dN2dy*m_b[0] + dN3dy*m_c[0], 0,
                dN1dx*m_a[1] + dN2dx*m_b[1] + dN3dx*m_c[1], dN1dy*m_a[1] + dN2dy*m_b[1] + dN3dy*m_c[1], 0,
                dN1dx*m_a[2] + dN2dx*m_b[2] + dN3dx*m_c[2], dN1dy*m_a[2] + dN2dy*m_b[2] + dN3dy*m_c[2], 0;

        Vector vx = F.block<3,1>(0,0);
        Vector vy = F.block<3,1>(0,1);
        Vector vz = vx.cross(vy);
        vz.normalize();
        F.block<3,1>(0,2) = vz;

        m_surface->property(m_deformationGradientFPH, *fIt) = F;
    }
}

void TriMesh::computeAreas()
{
    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt)
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
}

void TriMesh::refine()
{
//    CSurface cSurface;
//    std::vector<VertexIndex> vertexIndexes(m_surface->n_vertices());

//    auto vBegin = m_surface->vertices_begin();
//    auto vEnd = m_surface->vertices_end();

//    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
//        OMPoint omPoint = m_surface->point()
//        CPoint cPoint(omPoint[0], omPoint[1], omPoint[2]);
//        vertexIndexes[vIt->idx()] = cSurface.add_vertex(cPoint);
//    }
}

std::pair<unsigned int, TriMesh::Vector> TriMesh::barycentricCoordinates(const TriMesh::Vector &point, Vector &bCoo)
{
    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {

        auto cfv_it = m_surface->cfv_begin(*fIt);
        int idx = 0;
        CPoint cgtp[3];
        OMPoint ctp[3];
        for (; cfv_it.is_valid(); ++cfv_it, ++idx) {
            OMPoint p = m_surface->point(*cfv_it);
            ctp[idx] = p;
            cgtp[idx] = CPoint(p[0], p[1], p[2]);
        }
        CTriangle tri(cgtp[0], cgtp[1], cgtp[2]);
        CPoint cPoint(point[0], point[1], point[2]);

        if (tri.has_on(cPoint)) {
            Vector p[3];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    p[i][j] = ctp[i][j];

            Vector f[3];
            for (uint i = 0; i < 3; ++i)
                f[i] = p[i] - point;

            double A = m_surface->property(m_areaFPH, *fIt);
            Vector bar;
            bar[0] = 0.5*f[1].cross(f[2]).norm()/A;
            bar[1] = 0.5*f[2].cross(f[0]).norm()/A;
            bar[2] = 0.5*f[0].cross(f[1]).norm()/A;
            bCoo = bar;

            return std::pair<unsigned int, Vector>(fIt->idx(), bar);
        }
    }
    std::cout << "Point is not in any triangle of the mesh";
    return std::pair<unsigned int, Vector>(-1, Vector(0,0,0));
}

TriMesh::Surface *TriMesh::surface() const
{
    return m_surface;
}
