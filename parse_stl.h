#ifndef PARSE_STL_H
#define PARSE_STL_H

#include <string>
#include <vector>

namespace stl {

  struct point {
    float x;
    float y;
    float z;
	int index;
	
	std::vector<std::pair<int,float>> leng;
	point() : x(0), y(0), z(0) {  }
	point(float xp, float yp, float zp) : x(xp), y(yp), z(zp) { }
  };

  struct triangle {

    point v1;
    point v2;
    point v3;

    triangle(point& v1p, point& v2p, point& v3p) :
	
		v1(v1p), v2(v2p), v3(v3p) { 
		v1p.leng.push_back(std::make_pair(v2p.index,(sqrt((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y) + (v1.z - v2.z)*(v1.z - v2.z)))));
		v1p.leng.push_back(std::make_pair(v3p.index,(sqrt((v1.x - v3.x)*(v1.x - v3.x) + (v1.y - v3.y)*(v1.y - v3.y) + (v1.z - v3.z)*(v1.z - v3.z)))));
		v2p.leng.push_back(std::make_pair(v3p.index, (sqrt((v2.x - v3.x)*(v2.x - v3.x) + (v2.y - v3.y)*(v2.y - v3.y) + (v2.z - v3.z)*(v1.z - v3.z)))));
		
	}
	
  };
  
  std::ostream& operator<<(std::ostream& out, const triangle& t);

  struct stl_data {
    std::string name;
    std::vector<triangle> triangles;
	std::vector<point> pointsis;

    stl_data(std::string namep) : name(namep) {}
  };

  stl_data parse_stl(const std::string& stl_path, std::vector<point> &points);

}

#endif
