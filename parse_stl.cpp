#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <streambuf>
#include "parse_stl.h"

namespace stl {

  std::ostream& operator<<(std::ostream& out, const point p) {
    out << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    return out;
  }

  std::ostream& operator<<(std::ostream& out, const triangle& t) {
    out << "---- TRIANGLE ----" << std::endl;
    out << t.v1 << std::endl;
    out << t.v2 << std::endl;
    out << t.v3 << std::endl;
    return out;
  }
  
  float parse_float(std::ifstream& s) {
    char f_buf[sizeof(float)];
    s.read(f_buf, 4);
    float* fptr = (float*) f_buf;
    return *fptr;
  }

  point parse_point(std::ifstream& s) {
    float x = parse_float(s);
    float y = parse_float(s);
    float z = parse_float(s);
    return point(x, y, z);
  }

  

  stl_data parse_stl(const std::string& stl_path, std::vector<point> &points) {
    std::ifstream stl_file(stl_path.c_str(), std::ios::in | std::ios::binary);
    if (!stl_file) {
      std::cout << "ERROR: COULD NOT READ FILE" << std::endl;
      assert(false);
    }

    char header_info[80] = "";
    char n_triangles[4];
    stl_file.read(header_info, 80);
    stl_file.read(n_triangles, 4);
    std::string h(header_info);
    stl_data info(h);
    unsigned int* r = (unsigned int*) n_triangles;
    unsigned int num_triangles = *r;
	

    for (unsigned int i = 0; i < num_triangles; i++) {
      auto normal = parse_point(stl_file);
      auto v1 = parse_point(stl_file);
      auto v2 = parse_point(stl_file);
      auto v3 = parse_point(stl_file);
	  
	  bool ishere(0);
	  for (unsigned int j(0); j < points.size(); j++)  
		  if (v1.x == points[j].x&&v1.y == points[j].y&&v1.z == points[j].z)
		  {
			  v1.index = j;
			  ishere = true;
		  }
	  if (!ishere)
	  {
		  v1.index = points.size();
		  points.push_back(v1);
	  }
	  ishere = 0;
	  for (unsigned int j(0); j < points.size(); j++)
		  if (v2.x == points[j].x&&v2.y == points[j].y&&v2.z == points[j].z)
		  {
			  v2.index = j;
			  ishere = true;
		  }
	  if (!ishere)
	  {
		  v2.index = points.size();
		  points.push_back(v2);
	  }
	  ishere = 0;
	  for (unsigned int j(0); j < points.size(); j++)
		  if (v3.x == points[j].x&&v3.y == points[j].y&&v3.z == points[j].z)
		  {
			  v3.index = j;
			  ishere = true;
		  }
	  if (!ishere)
	  {
		  v3.index = points.size();
		  points.push_back(v3);
	  }
	  ishere = 0;

      info.triangles.push_back(triangle(v1, v2, v3));
	  for (int k(0); k < v1.leng.size(); k++)
	  {
		  bool ishere2 = 0;
		  for (int k2(0); k2 < points[v1.index].leng.size(); k2++)
			  if (points[v1.index].leng[k2].first == v1.leng[k].first)
				  ishere2 = true;
		  if (!ishere2)
			  points[v1.index].leng.push_back(v1.leng[k]);
		  ishere2 = 0;
	  }

	  for (int k(0); k < v2.leng.size(); k++)
	  {
		  bool ishere2 = 0;
		  for (int k2(0); k2 < points[v2.index].leng.size(); k2++)
			  if (points[v2.index].leng[k2].first == v2.leng[k].first)
				  ishere2 = true;
		  if (!ishere2)
			  points[v2.index].leng.push_back(v2.leng[k]);
		  ishere2 = 0;
	  }
      char dummy[2];
      stl_file.read(dummy, 2);
    }
    return info;
  }
}
