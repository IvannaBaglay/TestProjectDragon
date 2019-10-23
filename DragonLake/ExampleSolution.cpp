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
    std::vector<box> boxes;
	float maxFuelWeight;
	float maxCarryingWeight;
    float resourcesConsumption;
    struct maxCarryingCapacity
    {
        int half_x;
        int half_y;
        int half_z;
    };



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
	std::ifstream i(inputJasonFile);
	
	json j = json::parse(i); // create json with keys from file
    json j_out;
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
                j_out["steps"] = json::array(DeliverBoxes());  // return string about shippedBoxes
                                // like

            }
            
    }

    */

	 
	
	// do some stuff

	std::ofstream o(outputFileName);
	o << std::setw(4) << j_out << std::endl; //Write solution in file
}


