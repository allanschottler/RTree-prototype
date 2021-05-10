#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <map>
#include <random>
#include <assert.h>
#include <glm/glm.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> point3d;
typedef bg::model::box<point3d> box;

typedef std::map<float, point3d> trajectory;
typedef std::pair<box, trajectory::iterator> value;
typedef bgi::rtree<value, bgi::rstar<16>> RTree;

typedef std::pair<point3d, point3d> ray;

constexpr unsigned num_samples = 3500000;
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
	std::uniform_real_distribution d(min, max);

	return { d(mt), d(mt) };
}

trajectory buildTrajPoints(tag::Vertical)
{
	trajectory out;

	auto [x, y] = random_point2d(min, max);

	for (unsigned i{}; i <= num_samples; ++i)
	{
		float z = 0.004f * i;
		out.emplace(z, point3d{ x, y, z });
	}

	return out;
}

trajectory buildTrajPoints(tag::Diagonal)
{
	trajectory out;

	auto [x1, y1] = random_point2d(min, max);
	auto [x2, y2] = random_point2d(min, max);
	if (x1 < x2) std::swap(x1, x2);
	if (y1 < y2) std::swap(y1, y2);

	float inc = 1.f / num_samples;
	for (unsigned i{}; i <= num_samples; ++i)
	{
		float offset = i * inc;
		float x = x1 * (1 - offset) + x2 * offset;
		float y = y1 * (1 - offset) + y2 * offset;
		float z = 0.004f * i;
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
		float z = 0.004f * i;

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

RTree makeTree(trajectory& t)
{
	RTree rtree;
	//TODO
	return rtree;
}

value queryBrute(ray& r, trajectory& t)
{
	return {};
}

value queryTree(ray& r, RTree& tree)
{
	return {};
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
	//trajectory t = buildTraj<tag::Directional>();
	
	//2. Construção das caixas envolventes de cada segmento.
	//3. Construção da r-tree.
	//RTree rtree = makeTree(t);
    
	//4. Criar função para geração de raios arbitrários.
	
	//Box de teste. TODO: Usar rtree.bounds() no lugar
	auto testBox = box(point3d(0, 0, 0), point3d(100, 100, 100)); 
	auto rays = makeRays(1000, testBox);
	
	int totalBrute = 0, totalTree = 0;
	int timeBrute = 0, timeTree = 0;

	/*
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
	*/

	return 0;
}