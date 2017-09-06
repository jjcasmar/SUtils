#include "trimesh.h"

TriMesh::TriMesh(const std::string &filename)
    : m_surface( new Surface)
{
    OpenMesh::IO::read_mesh(*m_surface, filename);

    m_surface->request_vertex_normals();
    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        Surface::Normal n = m_surface->calc_vertex_normal(i.handle());
        m_surface->set_normal(i.handle(), n);
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

    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        Surface::Normal n = m_surface->calc_vertex_normal(i.handle());
        m_surface->set_normal(i.handle(), n);
    }
}

TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<TriMesh::Vector> &normals, const std::vector<unsigned int> &facets) :
    TriMesh(vertices, facets)
{
    auto b = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();
    for (auto i = b; i != e; ++i) {
        CPoint n; auto v = normals[i.handle().idx()];
        n[0] = v[0];
        n[1] = v[1];
        n[2] = v[2];
        m_surface->set_normal(i.handle(), n);
    }
}

//TriMesh::TriMesh(const std::vector<TriMesh::Vector> &vertices, const std::vector<TriMesh::Vector> &normals, const std::vector<TriMesh::Vector> &uv, const std::vector<unsigned int> &facets) :
//    TriMesh(vertices, normals, facets)
//{
//}

std::vector<TriMesh::Vector> TriMesh::vertices()
{
    std::vector<TriMesh::Vector> vertices(this->vertices().size());

    auto i = m_surface->vertices_begin();
    auto e = m_surface->vertices_end();

    for (auto it = i; it != e; ++it) {
        Vector p;
        p[0] = m_surface->point(it.handle())[0];
        p[1] = m_surface->point(it.handle())[1];
        p[2] = m_surface->point(it.handle())[2];

        vertices[it.handle().idx()] = p;
    }

    return vertices;
}

std::vector<TriMesh::Vector> TriMesh::normals()
{

}

//std::vector<TriMesh::Vector> TriMesh::uv()
//{

//}

std::vector<unsigned int> TriMesh::facets()
{
    std::vector<unsigned int> facets(this->vertices().size());

//    auto i = m_surface.vertices_begin();
//    auto e = m_surface.vertices_end();

//    for (auto it = i; it != e; ++it) {
//        Vector p;
//        p[0] = m_surface.point(it.handle())[0];
//        p[1] = m_surface.point(it.handle())[1];
//        p[2] = m_surface.point(it.handle())[2];

//        vertices[it.handle().idx()] = p;
//    }

    return facets;
}

TriMesh::Surface *TriMesh::surface() const
{
    return m_surface;
}

template<class T2>
std::vector<T2> TriMesh::vertexProperty(const std::string &property)
{

}

template<class T2>
std::vector<T2> TriMesh::facetProperty(const std::string &property)
{

}
