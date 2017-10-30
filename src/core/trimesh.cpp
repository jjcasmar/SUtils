#include "trimesh.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <limits>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/polygon_mesh_processing.h>
#include <CGAL/Polygon_mesh_processing/refine.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Delaunay_mesher_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangle_3<K> CTriangle;
typedef CGAL::Point_3<K> CPoint;
typedef CGAL::Surface_mesh<CPoint> CSurface;

typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CDT::Point CDTPoint;

TriMesh::TriMesh()
    : m_surface(new Surface)
{
    m_surface->request_vertex_normals();
    m_surface->request_face_normals();

    m_surface->add_property(m_materialPointVPH);
    m_surface->add_property(m_materialNormalVPH);
    m_surface->add_property(m_materialNormalFPH);
    m_surface->add_property(m_deformationGradientFPH);
    m_surface->add_property(m_areaFPH);
    m_surface->add_property(m_dnFPH);
    m_surface->add_property(m_dMFPH);
    m_surface->add_property(m_bendingEPH);
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
    m_surface->add_property(m_dnFPH);
    m_surface->add_property(m_dMFPH);
    m_surface->add_property(m_bendingEPH);

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
        m_surface->property(m_deformationGradientFPH, *fIt) = Matrix32();
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
        m_surface->property(m_dnFPH, *fIt) = dn(*fIt);
    }

}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<unsigned int> &facets)
    : m_surface(new Surface)
{
    initFromPointsAndFacets(vertices, facets);
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

TriMesh::TriMesh(const TriMesh &other)
{
    m_surface = new Surface(*other.surface());
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector2> &vertices, const std::vector<unsigned int> &edges, double aspect, double size)
    : m_surface(new Surface)
{
    CDT cdt;
    std::vector<CDTPoint> cdtPoints;
    cdtPoints.reserve(vertices.size());
    for (const Vector2 &vertex : vertices) {
        cdtPoints.push_back(CDTPoint(vertex(0), vertex(1)));
    }

    for (uint i = 0; i < edges.size()/2; ++i) {
        cdt.insert_constraint(cdtPoints[edges[2*i+0]], cdtPoints[edges[2*i+1]]);
    }

    CGAL::refine_Delaunay_mesh_2(cdt, Criteria(aspect, size));

    const auto vIt = cdt.vertices_begin();
    const auto vEnd = cdt.vertices_end();
    std::vector<Vector> points;
    std::map<CDT::Vertex_handle, uint> vertexHandles;
    points.reserve(cdt.number_of_vertices());
    uint index = 0;
    for (auto it = vIt; it != vEnd; ++it) {
        points.push_back(Vector(it->point()[0], it->point()[1], 0));
        vertexHandles[it->handle()] = index;
        index++;
    }

    const auto fIt = cdt.faces_begin();
    const auto fEnd = cdt.faces_end();
    std::vector<unsigned int> facets;
    facets.reserve(3*cdt.number_of_faces());
    for (auto it = fIt; it != fEnd; ++it) {
        facets.push_back(vertexHandles[it->vertex(0)->handle()]);
        facets.push_back(vertexHandles[it->vertex(1)->handle()]);
        facets.push_back(vertexHandles[it->vertex(2)->handle()]);
    }

    initFromPointsAndFacets(points, facets);
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

std::vector<TriMesh::Matrix32> TriMesh::deformationGradients() const
{
    std::vector<Matrix32> d(m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    for (auto it = i; it != e; ++it) {
        const Matrix32 m = m_surface->property(m_deformationGradientFPH, *it);
        d[it->idx()] = m;
    }

    return d;
}

std::vector<TriMesh::Matrix2> TriMesh::dm() const
{
    std::vector<Matrix2> d(m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    for (auto it = i; it != e; ++it) {
        Matrix2 m = m_surface->property(m_dMFPH, *it);
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

std::vector<double> TriMesh::edgesBending() const
{
    std::vector<double> d(m_surface->n_edges());

    auto i = m_surface->edges_begin();
    auto e = m_surface->edges_end();

    for (auto it = i; it != e; ++it) {
        if (m_surface->is_boundary(*it))
                continue;
        double m = m_surface->property(m_bendingEPH, *it);
        d[it->idx()] = m;
    }

    return d;
}

std::vector<Eigen::Matrix<double, 3, 2> > TriMesh::restPoseInverseMatrix() const
{
    std::vector<Eigen::Matrix<double, 3, 2> > d(m_surface->n_faces());

    auto i = m_surface->faces_begin();
    auto e = m_surface->faces_end();

    for (auto it = i; it != e; ++it) {
        Eigen::Matrix<double, 3, 2> m = m_surface->property(m_restPoseMatrixFPH, *it);
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
    OMPoint p[3];

    for (; cfv_it.is_valid(); ++cfv_it, ++idx) {
        p[idx] = m_surface->point(*cfv_it);
        cgtp[idx] = CPoint(p[idx][0], p[idx][1], p[idx][2]);
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

double TriMesh::bending(Surface::EdgeHandle edgeHandle)
{
    OMPoint omPoints[4];

    // Get the edge face0
    auto halfedge0 = m_surface->halfedge_handle(edgeHandle, 0);
    auto halfedge1 = m_surface->halfedge_handle(edgeHandle, 1);
    omPoints[1] = m_surface->point(m_surface->to_vertex_handle(halfedge0));
    omPoints[2] = m_surface->point(m_surface->to_vertex_handle(halfedge1));

    halfedge0 = m_surface->next_halfedge_handle(halfedge0);
    halfedge1 = m_surface->next_halfedge_handle(halfedge1);

    omPoints[0] = m_surface->point(m_surface->to_vertex_handle(halfedge0));
    omPoints[3] = m_surface->point(m_surface->to_vertex_handle(halfedge1));

    OMPoint ompe0 = omPoints[2] - omPoints[1];
    OMPoint ompe1 = omPoints[0] - omPoints[2];
    OMPoint ompe1_tilde = omPoints[3] - omPoints[1];

    Vector e0;
    e0 << ompe0[0], ompe0[1], ompe0[2];
    Vector e1;
    e1 << ompe1[0], ompe1[1], ompe0[2];
    Vector e1_tilde;
    e1_tilde << ompe1_tilde[0], ompe1_tilde[1], ompe1_tilde[2];

    Vector n = e0.cross(e1);
    double n_length = n.norm();
    Vector n_tilde = -e0.cross(e1_tilde);
    double n_tilde_length = n_tilde.norm();
    n /= n_length;
    n_tilde /= n_tilde_length;

    double tan_half_theta = (n - n_tilde).norm()/(n + n_tilde).norm();
    double sign_angle = n.cross(n_tilde).dot(e0);
    if (std::isnan(sign_angle)) sign_angle = 1.;
    else sign_angle = sign_angle > 0 ? 1. : -1.;
    return 2.*sign_angle*tan_half_theta;
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
    Matrix32 F;
    F.setZero();

    OMPoint m_a, m_b, m_c;

    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
//    auto f = 0;
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        auto cfv_it = m_surface->cfv_begin(*fIt);
        m_a = m_surface->point(*cfv_it); cfv_it++;
        m_b = m_surface->point(*cfv_it); cfv_it++;
        m_c = m_surface->point(*cfv_it); cfv_it++;

        Matrix poseMatrix;
        poseMatrix << m_a[0], m_b[0], m_c[0],
                m_a[1], m_b[1], m_c[1],
                m_a[2], m_b[2], m_c[2];

        F.setZero();
        F = poseMatrix*m_surface->property(m_restPoseMatrixFPH, *fIt);

        m_surface->property(m_deformationGradientFPH, *fIt) = F;
    }
}

void TriMesh::computeEdgeBending()
{
    auto eBegin = m_surface->edges_begin();
    auto eEnd = m_surface->edges_end();
    auto eIt = eBegin;

    for (; eIt != eEnd; ++eIt){
        if (m_surface->is_boundary(*eIt))
            continue;
        m_surface->property(m_bendingEPH, *eIt) = bending(*eIt);
    }
}

void TriMesh::computeAreas()
{
    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt)
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
}

void TriMesh::refine(double targetEdgeLength)
{
    CSurface cSurface;
    std::vector<CSurface::Vertex_index> vertexIndexes(m_surface->n_vertices());

    auto vBegin = m_surface->vertices_begin();
    auto vEnd = m_surface->vertices_end();

    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
        OMPoint omPoint = m_surface->point(*vIt);
        CPoint cPoint(omPoint[0], omPoint[1], omPoint[2]);
        vertexIndexes[vIt->idx()] = cSurface.add_vertex(cPoint);
    }

    auto fBegin = m_surface->faces_begin();
    auto fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        auto cfv_it = m_surface->cfv_begin(*fIt);
        auto v0 = vertexIndexes[cfv_it->idx()]; cfv_it++;
        auto v1 = vertexIndexes[cfv_it->idx()]; cfv_it++;
        auto v2 = vertexIndexes[cfv_it->idx()]; cfv_it++;
        cSurface.add_face(v0, v1, v2);
    }

    std::vector<CSurface::Face_index>  new_facets;
    std::vector<CSurface::Vertex_index> new_vertices;
    CGAL::Polygon_mesh_processing::isotropic_remeshing(CGAL::faces(cSurface), targetEdgeLength, cSurface);

    m_surface->clean();
    auto cVBegin = cSurface.vertices_begin();
    auto cVEnd = cSurface.vertices_end();
    std::map<CSurface::Vertex_index, Surface::VertexHandle> omVertices;
    unsigned int i = 0;
    for (auto cVIt = cVBegin;  cVIt != cVEnd; ++cVIt, ++i) {
        CPoint cPoint = cSurface.point(*cVIt);
        OMPoint omPoint(cPoint[0], cPoint[1], cPoint[2]);
        omVertices[*cVIt] = m_surface->add_vertex(omPoint);
    }

    auto cFBegin = cSurface.faces_begin();
    auto cFEnd = cSurface.faces_end();
    for (auto cFIt = cFBegin; cFIt != cFEnd; ++cFIt) {
        CGAL::Vertex_around_face_iterator<CSurface> vbegin, vend;
        CSurface::Vertex_index vertexIndex[3];
        vertexIndex[0] = CSurface::Vertex_index();
        vertexIndex[1] = CSurface::Vertex_index();
        vertexIndex[2] = CSurface::Vertex_index();
        unsigned int j = 0;
        for (boost::tie(vbegin, vend) = cSurface.vertices_around_face(cSurface.halfedge(*cFIt));
             vbegin != vend;
             ++vbegin, ++j) {
            vertexIndex[j] = *vbegin;
        }
        std::cout << j << std::endl;
        m_surface->add_face(omVertices[vertexIndex[0]], omVertices[vertexIndex[1]], omVertices[vertexIndex[2]]);
    }

    computeFaceNormals();
    computeVertexNormals();

    vBegin = m_surface->vertices_begin();
    vEnd = m_surface->vertices_end();
    for (auto vIt = vBegin; vIt != vEnd; ++vIt) {
        m_surface->property(m_materialPointVPH, *vIt) = m_surface->point(*vIt);
        m_surface->property(m_materialNormalVPH, *vIt) = m_surface->normal(*vIt);
    }

    fBegin = m_surface->faces_begin();
    fEnd = m_surface->faces_end();
    for (auto fIt = fBegin; fIt != fEnd; ++fIt) {
        m_surface->property(m_materialNormalFPH, *fIt) = m_surface->normal(*fIt);
        m_surface->property(m_deformationGradientFPH, *fIt) = Matrix32();
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
        m_surface->property(m_dnFPH, *fIt) = dn(*fIt);
    }

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
            cgtp[idx] = CPoint(p[0], p[1], 0);
        }
        CTriangle tri(cgtp[0], cgtp[1], cgtp[2]);
        CPoint cPoint(point[0], point[1], 0);

        bool isInTriangle = tri.has_on(cPoint);
        if (isInTriangle) {
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

void TriMesh::initFromPointsAndFacets(const std::vector<TriMesh::Vector> &points, const std::vector<unsigned int> &facets)
{
    std::vector<Surface::VertexHandle> vertexHandles(points.size());
    for (uint i = 0; i < points.size(); ++i) {
        Vector p = points[i];
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
    m_surface->add_property(m_dMFPH);
    m_surface->add_property(m_bendingEPH);
    m_surface->add_property(m_restPoseMatrixFPH);

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
        m_surface->property(m_deformationGradientFPH, *fIt) = Matrix32();
        m_surface->property(m_areaFPH, *fIt) = faceArea(*fIt);
        m_surface->property(m_dnFPH, *fIt) = dn(*fIt);

        auto cfv_it = m_surface->cfv_begin(*fIt);
        OMPoint cp[3];
        cp[0] = m_surface->point(*cfv_it); cfv_it++;
        cp[1] = m_surface->point(*cfv_it); cfv_it++;
        cp[2] = m_surface->point(*cfv_it); cfv_it++;

        Matrix restPoseMatrix;
        restPoseMatrix << cp[0][0], cp[0][1], 1,
                cp[1][0], cp[1][1], 1,
                cp[2][0], cp[2][1], 1;

        m_surface->property(m_restPoseMatrixFPH, *fIt) = restPoseMatrix.transpose().inverse().block<3,2>(0,0);
    }
}
