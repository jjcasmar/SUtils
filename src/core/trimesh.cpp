#include "trimesh.h"

TriMesh::TriMesh()
    : m_surface(new Surface)
{

}

TriMesh::TriMesh(const std::string &filename)
    : m_surface( new Surface)
{
    OpenMesh::IO::read_mesh(*m_surface, filename);

    m_surface->request_vertex_normals();
    m_surface->request_face_normals();

    auto fb = m_surface->faces_begin();
    auto fe = m_surface->faces_end();
    for (auto fIt = fb; fIt != fe; ++fIt) {
        Surface::Normal n = m_surface->calc_face_normal(*fIt);
        m_surface->set_normal(*fIt, n);
    }


    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        Surface::Normal n = m_surface->calc_vertex_normal(*i);
        m_surface->set_normal(*i, n);
    }
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<unsigned int> &facets)
    : m_surface(new Surface)
{
    std::vector<Surface::VertexHandle> vertexHandles;
    for (uint i = 0; i < vertices.size(); ++i) {
        Vector p = vertices[i];
        vertexHandles[i] = m_surface->add_vertex(CPoint(p[0], p[1], p[2]));
    }

    for (uint i = 0; i < facets.size(); ++i) {
        m_surface->add_face(vertexHandles[facets[3*i+0]],
                vertexHandles[facets[3*i+1]],
                vertexHandles[facets[3*i+2]]);
    }

    m_surface->request_vertex_normals();
    m_surface->request_face_normals();

    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        Surface::Normal n = m_surface->calc_vertex_normal(*i);
        m_surface->set_normal(*i, n);
    }
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<TriMesh::Vector> &normals, const std::vector<unsigned int> &facets) :
    TriMesh(vertices, facets)
{
    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        CPoint n; auto v = normals[i->idx()];
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

void TriMesh::computeVertexNormals()
{
    auto fb = m_surface->faces_begin();
    auto fe = m_surface->faces_end();
    for (auto fIt = fb; fIt != fe; ++fIt) {
        Surface::Normal n = m_surface->calc_face_normal(*fIt);
        m_surface->set_normal(*fIt, n);
    }

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        m_surface->set_normal(*it, m_surface->calc_vertex_normal(*it));
    }
}

TriMesh::Surface *TriMesh::surface() const
{
    return m_surface;
}

//template<class T2>
//std::vector<T2> TriMesh::vertexProperty(const std::string &property)
//{

//}

//template<class T2>
//std::vector<T2> TriMesh::facetProperty(const std::string &property)
//{

//}
