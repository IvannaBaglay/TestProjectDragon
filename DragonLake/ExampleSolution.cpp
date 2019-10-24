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
    std::vector<box> boxes_;
	float maxFuelWeight_;
	float maxCarryingWeight_;
    float resourcesConsumption_;

    struct maxCarryingCapacity
    {
        int half_x_;
        int half_y_;
        int half_z_;
    } maxCarryCapacity_;


};

struct IvannaBaglayPathFinder : public IGalaxyPathFinder
{
private:
	std::vector<box> boxes_; // added only on base
	std::vector<targetPoint> targetPoints_;
	ship myship_;

public:
	virtual void FindSolution(const char* inputJasonFile, const char* outputFileName);
	virtual const char* ShowCaptainName() { return "Ivanna Baglay"; }

    void LoadInformationFromJson(json& j);
    void LoadInformationAboutShipFromJson(json& j);
    void LoadInformationAboutTargetPointFromJson(json& j);
    void LoadInformationAboutBoxFromJson(json& j);
    void FindShortestRoutes();
    void CreateMatrixForAlgorithm();
};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
    std::vector<targetPoint> shortestRoutes;
	std::ifstream i(inputJasonFile);

    json j = json::parse(i, nullptr, false);
	// do some stuff
    LoadInformationFromJson(j);


    /*
    CreateMatrixForAlgorithm();
    while(!boxes_.empty())
    {
    
        shortestRoutes = FindShortestRoutes(targetPoints_);
        DeliverBoxes(shortestRoutes);  // return string about shippedBoxes
                                // like
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
    myship_.maxFuelWeight_ = j["maxResourcesWeight"];
    myship_.maxCarryingWeight_ = j["maxCarryingWeight"];
    myship_.resourcesConsumption_ = j["resourcesConsumption"];

    myship_.maxCarryCapacity_.half_x_ = j["maxCarryingCapacity"]["half_x"];
    myship_.maxCarryCapacity_.half_y_ = j["maxCarryingCapacity"]["half_y"];
    myship_.maxCarryCapacity_.half_z_ = j["maxCarryingCapacity"]["half_z"];
}
void IvannaBaglayPathFinder::LoadInformationAboutBoxFromJson(json& j)
{
    for (json::iterator it = j.begin(); it != j.end(); it++)
    {
        boxes_.push_back(box((*it)["boxId"], (*it)["half_x"], (*it)["half_y"], (*it)["half_z"],
                            (*it)["weight"], (*it)["targetPointId"]));

    }
}
void IvannaBaglayPathFinder::LoadInformationAboutTargetPointFromJson(json& j)
{
    for (json::iterator it = j.begin(); it != j.end(); it++)
    {
        targetPoints_.push_back(targetPoint((*it)["pointId"],
                                            (*it)["x"], (*it)["y"], (*it)["z"]));
    }
}

void IvannaBaglayPathFinder::FindShortestRoutes()
{
    /* 
            maxKilometerGrowth = FindMaxFromMatrixKilometerGrowth(); // return pair i j;
            
            CanBeNewRoute = FindNewOptimizedRoute(maxKilometerGrowth); // return pair vector route and kilometers

            //TODO: 
            How present set route?
            How unite set with point? 

            if(CheckCondition(NewRoute))
            {
                UniteSimpleRoute();
            }
                
        
        
    */
}



int main()
{
    IvannaBaglayPathFinder test;
    test.FindSolution("inputData1.json", "outData1.json");
    return 0;
}