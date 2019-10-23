#include "IGalaxyPathFinder.h"
#include "json.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

struct box
{
	int id;
	int x, y, z;
	float w;
	int target;
};

struct targetPoint
{
	int id;
	float x, y, z;
};

struct ship
{
	float maxFuelWeight;
	float maxCarryingWeight;
};

struct NameSurnamePathFinder : public IGalaxyPathFinder
{
private:
	std::vector<box> boxes;
	std::vector<targetPoint> targetPoints;
	ship myship;

public:
	virtual void FindSolution(const char* inputJasonFile, const char* outputFileName);
	virtual const char* ShowCaptainName() { return "Name Surname"; }
};

void NameSurnamePathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
	std::ifstream i(inputJasonFile);
	
	json j = json::parse(i, nullptr, false);

	// do some stuff

	json j_out;
	j_out["steps"] = json::array();
	// do some stuff

	std::ofstream o(outputFileName);
	o << std::setw(4) << j_out << std::endl;
}


