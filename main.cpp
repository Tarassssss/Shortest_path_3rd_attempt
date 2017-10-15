#include <cassert>
#include <iostream>
#include <stdio.h>
#include <limits.h>
#include "parse_stl.h"
#include <fstream>
#include <conio.h>
#include "geodesic_algorithm_exact.h"


int main(int argc, char* argv[]) {

  std::string stl_file_name = "./Example.stl";
  if (argc <= 4&&argc>=2) {
    stl_file_name = argv[1];
  } else if (argc > 4) {
    std::cout << "ERROR: Too many command line arguments" << std::endl;
  }

  std::vector<stl::point> points;
  auto info = stl::parse_stl(stl_file_name, points);
  for (int i(0); i < points.size(); i++)
	  points[i].index = i;

  std::vector<stl::triangle> triangles = info.triangles;
  std::cout << "STL HEADER = " << info.name << std::endl;
  
  int N = points.size();
  std::vector<double> points2;
  std::vector<unsigned> faces;
  float sx, sy, sz, tx, ty, tz;
  unsigned source_vertex_index;
  unsigned target_vertex_index;
	  
  if (argc == 2)
	{
		std::cout << "Input coordinates of the source:\n";
	    std::cin >> sx >> sy >> sz;
		std::cout << "Input coordinates of the target:\n";
		std::cin >> tx >> ty >> tz;
		bool sourcehere = 0, targethere = 0;
		for (int i(0); i < N; i++)
		{
			if (((round(sx * 10) / 10) == (round(points[i].x * 10) / 10)) && ((round(sy * 10) / 10) == (round(points[i].y * 10) / 10)) && ((round(sz * 10) / 10) == (round(points[i].z * 10) / 10)))
			{
				source_vertex_index = i;
				sourcehere = 1;
			}
			if ((round(tx * 10) / 10) == (round(points[i].x * 10) / 10) && (round(ty * 10) / 10) == (round(points[i].y * 10) / 10) && (round(tz * 10) / 10) == (round(points[i].z * 10) / 10))
			{
				target_vertex_index = i;
				targethere = 1;
			}
		 }
		 if (!sourcehere)
		  {
			  stl::point vv1(sx, sy, sz);
			  vv1.index = points.size(); 
			  source_vertex_index = points.size();
			  points.push_back(vv1);
		  }
		  if (!targethere)
		   {
			  stl::point vv2(tx, ty, tz);
			  vv2.index = points.size();
			  target_vertex_index = points.size();
			  points.push_back(vv2);
		   }
	  }
  N = points.size();
	  
  std::ofstream fout("Remaked.txt");
  fout << points.size() << ' ' << triangles.size() << std::endl;
  for (int i(0); i < N; i++)
	fout << points[i].x << ' ' << points[i].y << ' ' << points[i].z << std::endl;
  for (int i(0); i < triangles.size(); i++)
	fout << triangles[i].v1.index << ' ' << triangles[i].v2.index << ' ' << triangles[i].v3.index << std::endl;
  fout.close();
	  std::ifstream fin("Remaked.txt");
	  bool success = geodesic::read_mesh_from_file("Remaked.txt", points2, faces);
	  if (!success)
	  {
		  std::cout << "something is wrong with the input file" << std::endl;
		  return 0;
	  }

	  geodesic::Mesh mesh;
	  mesh.initialize_mesh_data(points2, faces);		//create internal mesh data structure including edges

	  geodesic::GeodesicAlgorithmExact algorithm(&mesh);	//create exact algorithm for the mesh
	  if(argc>2)
		source_vertex_index = atol(argv[2]);

	  geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);		//create source 
	  std::vector<geodesic::SurfacePoint> all_sources(1, source);					//in general, there could be multiple sources, but now we have only one

	 	//target vertex specified, compute single path
	  {
		  if (argc > 3)
			target_vertex_index = atol(argv[3]);
		  geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);		//create source 

		  std::vector<geodesic::SurfacePoint> path;	//geodesic path is a sequence of SurfacePoints

		  bool const lazy_people_flag = false;		//there are two ways to do exactly the same
		  if (lazy_people_flag)
		  {
			  algorithm.geodesic(source, target, path); //find a single source-target path
		  }
		  else		//doing the same thing explicitly for educational reasons
		  {
			  double const distance_limit = INFINITY;			// no limit for propagation
			  std::vector<geodesic::SurfacePoint> stop_points(1, target);	//stop propagation when the target is covered
			  algorithm.propagate(all_sources, distance_limit, &stop_points);	//"propagate(all_sources)" is also fine, but take more time because covers the whole mesh

			  algorithm.trace_back(target, path);		//trace back a single path 
		  }

		  print_info_about_path(path);
		  for (int i = (path.size() - 1); i >= 0; i--)
		  {
			  geodesic::SurfacePoint& s = path[i];
			  std::cout << s.x() << "\t" << s.y() << "\t" << s.z() << std::endl;
		  }
	  }
	  _getch();
	  fin.close();
	  return 0;
}
