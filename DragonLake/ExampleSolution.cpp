#include "IGalaxyPathFinder.h"
#include "json.hpp"

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
struct maxCarryingCapacity
{
    int half_x;
    int half_y;
    int half_z;
};
struct ship
{
    std::vector<box> boxes;
	float maxFuelWeight;
	float maxCarryingWeight;
    float resourcesConsumption;




};

struct IvannaBaglayPathFinder : public IGalaxyPathFinder
{
private:
	std::vector<box> boxes; // added only on base
	std::vector<targetPoint> targetPoints;
	ship myship;

public:
	virtual void FindSolution(const char* inputJasonFile, const char* outputFileName);
	virtual const char* ShowCaptainName() { return "Ivanna Baglay"; }
};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
    using json = nlohmann::json;
	std::ifstream i(inputJasonFile);

    json j = json::parse(i, nullptr, false);
	// do some stuff

    /*
    {
    LoadInformationFromJson(json);
        {
         myship = LoadInformationAboutShipFromJson(json["ship"]);
         targetPoints = LoadInformationAboutTargetPointFromJson(json["targetPoints"]);
         boxes = LoadInformationAboutBoxFromJson(json["boxes"]);
        }   

    }

    while(!boxes.empty())
    {
    
        shortestRoutes = FindShortestRoutes(targetPoints);
            
            for (int i = 0; i ! = shortestRoutes.size(); i++)
            {
                myship.PackBoxes(boxes, shortestRoutes[i]);
                DeliverBoxes();  // return string about shippedBoxes
                                // like

            }
            
    }

    */

	 
    json j_out;
    j_out["steps"] = json::array();

	// do some stuff

	//std::ofstream o(outputFileName);
	//o << std::setw(4) << j_out << std::endl; //Write solution in file
}


int main()
{
    IvannaBaglayPathFinder test;
    test.FindSolution("inputData1.json", "outData1.json");
    return 0;
}