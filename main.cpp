#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <map>
#include <assert.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point3d;
typedef bg::model::box<point3d> box;

typedef std::map<float, point3d> trajectory;
typedef std::pair<box, trajectory::iterator> value;
typedef bgi::rtree<value, bgi::rstar<16>> RTree;

typedef std::pair<point3d, point3d> ray;

trajectory buildTraj()
{
}

RTree makeTree(trajectory& t)
{
	RTree rtree;	
	//TODO
	return rtree;
}

value queryBrute(ray& r, trajectory& t)
{

}

value queryTree(ray& r, RTree& tree)
{

}

std::vector<ray> makeRays(int nRays)
{

}

int main()
{
	//1. Criar conjunto de segmentos equivalente à trajetória de um poço.
	trajectory t = buildTraj();
	
	//2. Construção das caixas envolventes de cada segmento.
	//3. Construção da r-tree.
	RTree rtree = makeTree(t);
    
	//4. Criar função para geração de raios arbitrários.
	auto rays = makeRays(1000);
	
	int totalBrute = 0, totalTree = 0;
	int timeBrute = 0, timeTree = 0;

	//5. Testar queries usando r-tree e força bruta.
	for(auto& ray : rays)
	{
		//ClockStart
		auto v1 = queryBrute(ray, t);
		//totalBrute = ClockEnd
	 
		//ClockStart
		auto v2 = queryTree(ray, rtree);
		//totalTree = ClockEnd

		assert(v1==v2);
		
		totalBrute += timeBrute;
		totalTree += timeTree;
	}

	totalBrute /= rays.size();
	timeTree /= rays.size();

	return 0;
}