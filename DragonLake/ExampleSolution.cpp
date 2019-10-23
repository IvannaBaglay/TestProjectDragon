#include "IGalaxyPathFinder.h"
#include "json.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
using json = nlohmann::json;

struct box
{
    box(int id, int x, int y, int z, float w, int target) :
        id_(id), x_(x), y_(y), z_(z), w_(w), target_(target) {}
	int id_;
	int x_, y_, z_;
	float w_;
	int target_;
};

struct targetPoint
{
    targetPoint(int id, float x, float y, float z) :
        id_(id), x_(x), y_(y), z_(z) {}
	int id_;
	float x_, y_, z_;
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
    } maxCarryCapacity;


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

    void LoadInformationFromJson(json& j);
    void LoadInformationAboutShipFromJson(json& j);
    void LoadInformationAboutTargetPointFromJson(json& j);
    void LoadInformationAboutBoxFromJson(json& j);
};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
   
	std::ifstream i(inputJasonFile);

    json j = json::parse(i, nullptr, false);
	// do some stuff

    LoadInformationFromJson(j);

    /*
    {
    LoadInformationFromJson(json);
        {
         LoadInformationAboutShipFromJson(json["ship"]);
         LoadInformationAboutTargetPointFromJson(json["targetPoints"]);
         LoadInformationAboutBoxFromJson(json["boxes"]);
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

	std::ofstream o(outputFileName);
	o << std::setw(4) << j_out << std::endl; //Write solution in file
}

void IvannaBaglayPathFinder::LoadInformationFromJson(json& j)
{
    LoadInformationAboutShipFromJson(j["ship"]);
    LoadInformationAboutTargetPointFromJson(j["targetPoints"]);
    LoadInformationAboutBoxFromJson(j["boxes"]);
}

void IvannaBaglayPathFinder::LoadInformationAboutShipFromJson(json& j)
{
    myship.maxFuelWeight = j["maxResourcesWeight"];
    myship.maxCarryingWeight = j["maxCarryingWeight"];
    myship.resourcesConsumption = j["resourcesConsumption"];

    myship.maxCarryCapacity.half_x = j["maxCarryingCapacity"]["half_x"];
    myship.maxCarryCapacity.half_y = j["maxCarryingCapacity"]["half_y"];
    myship.maxCarryCapacity.half_z = j["maxCarryingCapacity"]["half_z"];
}
void IvannaBaglayPathFinder::LoadInformationAboutBoxFromJson(json& j)
{
    for (json::iterator it = j.begin(); it != j.end(); it++)
    {
        boxes.push_back(box((*it)["boxId"], (*it)["half_x"], (*it)["half_y"], (*it)["half_z"],
                            (*it)["weight"], (*it)["targetPointId"]));

    }
}
void IvannaBaglayPathFinder::LoadInformationAboutTargetPointFromJson(json& j)
{
    for (json::iterator it = j.begin(); it != j.end(); it++)
    {
        targetPoints.push_back(targetPoint((*it)["pointId"],
                                            (*it)["x"], (*it)["y"], (*it)["z"]));
    }
}

int main()
{
    IvannaBaglayPathFinder test;
    test.FindSolution("inputData1.json", "outData1.json");
    return 0;
}