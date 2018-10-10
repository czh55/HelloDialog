#pragma once
//define by czh
#ifndef HOLEFILLING_H
#define HOLEFILLING_H

#define CGAL_EIGEN3_ENABLED

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

#include <CGAL/boost/graph/graph_traits_PolyMesh_ArrayKernelT.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <CGAL/boost/graph/helpers.h>
#include <boost/foreach.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef OpenMesh::PolyMesh_ArrayKernelT< > Mesh;

typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;

/*
孔洞修复代码，最终输出的结果保存在result_HoleFilling文件夹中，为.off文件。输入文件默认在data文件夹中
注：输入和输出都是硬盘的文件
参数：
fileNameIn：是要处理的网格化文件，文件的格式可以为ply,off。eg:"result_mesh/result_mesh.ply"
fileNameOut:输出的结果文件，格式为off。eg:"filled_OM.off"
*/
void holeFilling(const char* fileNameIn,std::string fileNameOut)
{
	cout << "孔洞修复开始" << endl;
	Mesh mesh;
	if (OpenMesh::IO::read_mesh(mesh, fileNameIn) == -1) {
		cout << "没有读入网格文件" << endl;
		return;
	}

	// Incrementally fill the holes
	unsigned int nb_holes = 0;
	BOOST_FOREACH(halfedge_descriptor h, halfedges(mesh))
	{
		if (CGAL::is_border(h, mesh))
		{
			std::vector<face_descriptor>  patch_facets;
			std::vector<vertex_descriptor> patch_vertices;
			bool success = CGAL::cpp11::get<0>(
				CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
					mesh,
					h,
					std::back_inserter(patch_facets),
					std::back_inserter(patch_vertices),
					CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)).
					geom_traits(Kernel())));

			CGAL_assertion(CGAL::is_valid_polygon_mesh(mesh));

			std::cout << "* FILL HOLE NUMBER " << ++nb_holes << std::endl;
			std::cout << "  Number of facets in constructed patch: " << patch_facets.size() << std::endl;
			std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
			std::cout << "  Is fairing successful: " << success << std::endl;
		}
	}

	CGAL_assertion(CGAL::is_valid_polygon_mesh(mesh));
	std::cout << std::endl;
	std::cout << nb_holes << " holes have been filled" << std::endl;

	OpenMesh::IO::write_mesh(mesh, "result_HoleFilling/"+ fileNameOut );
	cout << "孔洞修复结束， 保存到：result_HoleFilling/" << fileNameOut << endl;
}

#endif // !HOLEFILLING_H

