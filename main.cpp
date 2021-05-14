#include "benchmark.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <boost/optional.hpp>

#include <glm/glm.hpp>

#include <iostream>
#include <map>
#include <random>

#define PACKING_TREE

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point3d;
typedef bg::model::box<point3d> box;

typedef std::map<float, point3d> trajectory;
typedef std::pair<box, trajectory::iterator> value;

#ifdef PACKING_TREE
typedef std::vector<value> trajectory_v;
#endif

#ifdef PACKING_TREE
typedef bgi::rtree<value, bgi::rstar<16>> RTree;
#else
typedef bgi::rtree<value, bgi::quadratic<16>> RTree;
#endif

typedef std::pair<point3d, point3d> ray;

#ifdef _DEBUG
	constexpr unsigned num_samples = 1000;
	constexpr unsigned num_rays = 100;
#else
	constexpr unsigned num_samples = 3500000;
	constexpr unsigned num_rays = 100000;
#endif

constexpr float min = 0.f;
constexpr float max = 1e6f;

namespace tag {
	struct Vertical {};
	struct Diagonal {};
	struct Directional {};
}

std::pair<float, float> random_point2d(float min, float max)
{
	std::random_device rd;
	std::mt19937 mt = std::mt19937(rd());
	std::uniform_real_distribution<> d(min, max);

	return { d(mt), d(mt) };
}

trajectory buildTrajPoints(tag::Vertical)
{
	trajectory out;

	auto [x, y] = random_point2d(min, max);

	for (unsigned i{}; i <= num_samples; ++i)
	{
		float z = static_cast<float>(i);
		out.emplace(z, point3d{ x, y, z });
	}

	return out;
}

trajectory buildTrajPoints(tag::Diagonal)
{
	trajectory out;

	auto [x1, y1] = random_point2d(min, max);
	auto [x2, y2] = random_point2d(min, max);
	if (x1 > x2) std::swap(x1, x2);
	if (y1 > y2) std::swap(y1, y2);

	float inc = 1.f / static_cast<float>(num_samples);
	for (unsigned i{}; i <= num_samples; ++i)
	{
		float offset = i * inc;
		float x = x1 * (1 - offset) + x2 * offset;
		float y = y1 * (1 - offset) + y2 * offset;
		float z = (float)i;
		out.emplace(z, point3d{ x, y, z });
	}

	return out;
}

trajectory buildTrajPoints(tag::Directional)
{
	trajectory out;

	auto [hx, hy] = random_point2d(min, max);
	auto [rx, ry] = random_point2d(min, max);
	for (unsigned i{}; i <= num_samples; ++i)
	{
		float offset = std::log(i + 1);
		float x = hx + rx * offset;
		float y = hy + ry * offset;
		float z = static_cast<float>(i);
		out.emplace(z, point3d{ x, y, z });
	}

	return out;
}

template <typename Tag>
trajectory buildTraj()
{ return buildTrajPoints(Tag{}); }

template <typename Compare>
point3d boxPoint(const point3d& p1, const point3d& p2, Compare pred)
{
	float x = pred(p1.get<0>(), p2.get<0>()) ? p1.get<0>() : p2.get<0>();
	float y = pred(p1.get<1>(), p2.get<1>()) ? p1.get<1>() : p2.get<1>();
	float z = pred(p1.get<2>(), p2.get<2>()) ? p1.get<2>() : p2.get<2>();

	return point3d{ x, y, z };
}

box makeBox(point3d p1, point3d p2)
{
	auto pmin = boxPoint(p1, p2, std::less_equal{});
	auto pmax = boxPoint(p1, p2, std::greater{});
	return box(pmin, pmax);
}

RTree makeTreePacking(trajectory& t, trajectory_v& tv)
{	
	for (auto it = t.begin(); it != std::prev(t.end()); ++it)
	{
		auto& p1 = it->second;
		auto& p2 = std::next(it)->second;
		tv.push_back(std::make_pair(makeBox(p1,p2), it));
	}
	
	RTree rtree(tv);
	return rtree;
}

RTree makeTree(trajectory& t)
{		
	RTree rtree;

	for (auto it = t.begin(); it != std::prev(t.end()); ++it)
	{
		auto& p1 = it->second;
		auto& p2 = std::next(it)->second;
		rtree.insert(std::make_pair(makeBox(p1,p2), it));
	}
	
	return rtree;
}

boost::optional<value> queryBrute(ray& r, trajectory& t)
{
	boost::geometry::model::segment<point3d> seg{ r.first, r.second };
	for (auto it = t.begin(); it != std::prev(t.end()); ++it)
	{
		auto bb = makeBox(it->second, std::next(it)->second);

		if (bg::intersects(seg, bb))
			return boost::make_optional(std::make_pair(bb, it));
	}

	return boost::none;
}

boost::optional<value> queryTree(ray& r, RTree& tree)
{
	std::vector<value> result = {};
	boost::geometry::model::segment<point3d> seg{ r.first, r.second };
	tree.query(bgi::intersects(seg), std::back_inserter(result));
	return result.empty() ? boost::none : boost::make_optional(result[0]);
}

point3d glm2gi(const glm::vec3& v)
{ return point3d(v.x, v.y, v.z); }

glm::vec3 gi2glm(const point3d& v)
{ return glm::vec3(v.get<0>(), v.get<1>(), v.get<2>()); }

std::vector<ray> makeRays(int nRays, box aabb)
{
	//Float aleatório entre 0 e 1
	auto randomFloat = []()
	{ return static_cast <float> (rand()) / static_cast <float> (RAND_MAX); };

	//Vec3 com coordenadas float aleatórias entre 0 e 1
	auto randomVec = [randomFloat]()
	{ return glm::vec3(randomFloat(), randomFloat(), randomFloat()); };
	
	auto min = gi2glm(aabb.min_corner());
	auto max = gi2glm(aabb.max_corner());
	auto delta = max - min;
	auto deltaL = glm::length(delta); //tamanho da diagonal

	std::vector<ray> rays;

	while (--nRays >= 0)
	{
		auto randomOrig = randomVec() * delta + min; //ponto aleatório dentro da box
		auto randomDir =  glm::normalize(randomVec() * glm::vec3(2) - glm::vec3(1)); //direção aleatória normalizada
		auto a = randomOrig + randomDir * deltaL; //ponto a do segmento do raio
		auto b = randomOrig - randomDir * deltaL; //ponto b do segmento do raio
		rays.push_back(std::make_pair(glm2gi(a), glm2gi(b)));
	}

	return rays;
}

int main()
{
	//1. Criar conjunto de segmentos equivalente à trajetória de um poço.
	trajectory t = benchmark("Criando trajetória procedural (" + std::to_string(num_samples) + " amostras)", 
							 [](){ return buildTraj<tag::Directional>(); });

	//2. Construção da r-tree.
#ifdef PACKING_TREE
	trajectory_v tv;
	RTree rtree = benchmark("Construindo RTree Packing", 
							[&t, &tv](){ return makeTreePacking(t, tv); });
#else
	RTree rtree = benchmark("Construindo R*Tree", 
							[&t](){ return makeTree(t); });
#endif
    
	//3. Criar função para geração de raios arbitrários.
	auto rays = benchmark("Criando raios aleatórios (" + std::to_string(num_rays) + ")", 
						  [&rtree](){ return makeRays(num_rays, rtree.bounds()); });	

	using results_map = std::map<ray*, glm::vec3>;
	results_map bruteResults, treeResults;

	//4. Testar queries usando Rtree.
	benchmark("Testando query em RTree", [&rtree, &rays, &treeResults]() {
		for(auto& r : rays)
			if(auto v = queryTree(r, rtree))
				treeResults[&r] = gi2glm(v->second->second);
	});

	//5. Testar queries usando força bruta.
	benchmark("Testando query força bruta", [&t, &rays, &bruteResults]() {
		for(auto& r : rays)
			if(auto v = queryBrute(r, t))
				bruteResults[&r] = gi2glm(v->second->second);
	});

	assert(bruteResults.size() == treeResults.size());
	for (auto& brutePair : bruteResults)
	{
		assert(treeResults.count(brutePair.first));
		assert(brutePair.second == treeResults.at(brutePair.first));
	}

	return 0;
}