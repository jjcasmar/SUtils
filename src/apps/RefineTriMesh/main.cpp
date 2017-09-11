#include "../../core/trimesh.h"

int main(int argc, char **argv)
{
    TriMesh mesh("coarseMesh.obj");
    mesh.refine();

}
