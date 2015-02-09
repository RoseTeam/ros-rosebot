#include "robomovies.h"

shared_ptr<Mesh> createRobomoviesMap()
{
	bool vide = true;

	shared_ptr<Mesh> mm(new Mesh(false));

	std::vector<Point2d> aireDepart = { Point2d(0, 778), Point2d(400, 778), Point2d(400, 800), 
				Point2d(70, 800), Point2d(70, 1200), Point2d(400, 1200), Point2d(400, 1222), Point2d(0, 1222) };

	std::vector<Point2d> coords = {Point2d(0,0)};

	for (auto p : aireDepart)
		coords.push_back(p);

	Point2d stairs[] = { Point2d(0, 2000), Point2d(1200, 2000), Point2d(1200, 1900), Point2d(1800, 1900), 
						Point2d(1800, 2000), Point2d(3000, 2000), };

	for (auto p : stairs)
		coords.push_back(p);

	for (auto it = aireDepart.rbegin(); it != aireDepart.rend(); ++it)
		coords.push_back(Point2d(3000-it->x,it->y));

	Point2d podium[] = { Point2d(3000, 0), Point2d(2033, 0), Point2d(2033, 580), Point2d(967, 580), Point2d(967, 0) };

	for (auto p : podium)
		coords.push_back(p);

	for (Point2d& p : coords)
	{
		double a = p[0];
		p[0] = p[1];
		p[1] = a;
	}

	std::vector<wertex*> bornes = mm->addVertices_py(coords);

	for (auto borne = bornes.begin() + 1; borne != bornes.end(); borne++)
		(*borne)->Connect(**(borne-1));

	bornes.front()->Connect(*bornes.back());

	mm->searchDual->resetConvexAreas();

	if (!vide)
	{
		//TODO add map props
	}
	return mm;

}
