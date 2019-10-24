#include "IGalaxyPathFinder.h"
#include "json.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <memory>
#include <algorithm>

using json = nlohmann::json;
typedef std::vector <std::unique_ptr<std::vector<float>>> ptr_to_matrix;
// If Sij was used then I write on this plase -1;

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

class Matrix
{
    
public:
    Matrix() {}
    //TODO: Base point can be not first in json file and problem that we don't have base point
    void CreateMartix(size_t sizeMatrix) 
    {
        matrixPtr_ = std::make_shared<ptr_to_matrix>();
        for (size_t i = 0; i < sizeMatrix; i++)
        {
            (*matrixPtr_).push_back(std::make_unique<std::vector<float>>());
            row_++;
            for (size_t j = 0; j <= i; j++)
            {
                (*(*matrixPtr_)[i]).push_back(1);
                sizeMatrix_++;
            }
        }
    }
    void FitMatrix(std::vector<targetPoint>& targetPoints)
    {
        for (size_t i = 0; i < row_; i++)
        {
            for (size_t j = 0; j <= i; j++)
            {
                (*(*matrixPtr_)[i])[j] = sqrt((targetPoints[i].x_ - targetPoints[j].x_) * (targetPoints[i].x_ - targetPoints[j].x_) +
                                                (targetPoints[i].y_ - targetPoints[j].y_) * (targetPoints[i].y_ - targetPoints[j].y_) +
                                                 (targetPoints[i].z_ - targetPoints[j].z_) * (targetPoints[i].z_ - targetPoints[j].z_));
            }
        }
    }
    void FitMatrix(Matrix& matrix)
    {
        for (size_t i = 0; i < row_; i++)
        {
            for (size_t j = 0; j <= i; j++)
            {
                (*(*matrixPtr_)[i])[j] = (*(*matrix.matrixPtr_)[i])[0] + (*(*matrix.matrixPtr_)[j])[0] - (*(*matrix.matrixPtr_)[i])[j];
            }
        }
    }
    const size_t get_row() const
    {
        return row_;
    }
    const auto get_matrix_ptr() const
    {
        return matrixPtr_;
    }
protected:
    std::shared_ptr <ptr_to_matrix> matrixPtr_;
private:    
    size_t sizeMatrix_ = 0;
    size_t row_ = 0;
};

/*class MatrixOfKilometerGrowth: public Matrix
{
public:
    MatrixOfKilometerGrowth(){}
    
private:

protected:

};

class MatrixOfDistanceBetweenPoints : public Matrix
{
public:
    MatrixOfDistanceBetweenPoints(){}
private:

protected:
};*/

struct IvannaBaglayPathFinder : public IGalaxyPathFinder
{
private:
	std::vector<box> boxes_; // added only on base
	std::vector<targetPoint> targetPoints_;
	ship myship_;

    Matrix matrixOfKilometerBetweenPoints_;
    Matrix matrixOfKilometerGrowth_;
public:
	virtual void FindSolution(const char* inputJasonFile, const char* outputFileName);
	virtual const char* ShowCaptainName() { return "Ivanna Baglay"; }

    void LoadInformationFromJson(json& j);
    void LoadInformationAboutShipFromJson(json& j);
    void LoadInformationAboutTargetPointFromJson(json& j);
    void LoadInformationAboutBoxFromJson(json& j);
    void FindShortestRoutes();
    void CreateMatrixForAlgorithm();

    std::pair<size_t, size_t> FindMaxFromMatrixKilometerGrowth();
};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
    std::vector<targetPoint> shortestRoutes;
	std::ifstream i(inputJasonFile);

    json j = json::parse(i, nullptr, false);
	// do some stuff
    LoadInformationFromJson(j);
    CreateMatrixForAlgorithm();


    FindShortestRoutes();
    /*
    
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

    auto maxKilometerGrowth = FindMaxFromMatrixKilometerGrowth(); // return pair i j;

    /* 
            
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

void IvannaBaglayPathFinder::CreateMatrixForAlgorithm()
{
    matrixOfKilometerBetweenPoints_.CreateMartix(targetPoints_.size());
    matrixOfKilometerGrowth_.CreateMartix(targetPoints_.size());
    matrixOfKilometerBetweenPoints_.FitMatrix(targetPoints_);
    matrixOfKilometerGrowth_.FitMatrix(matrixOfKilometerBetweenPoints_);

}

std::pair<size_t, size_t> IvannaBaglayPathFinder::FindMaxFromMatrixKilometerGrowth()
{
    size_t rowMatrix = matrixOfKilometerGrowth_.get_row();
    auto matrixPtr = matrixOfKilometerGrowth_.get_matrix_ptr();
    float maxValue = 0;
    size_t maxj = 0;
    size_t maxi = 0;
    for (size_t i = 0; i < rowMatrix ; i++)
    {
        for (size_t j = 0; j <= i; j++)
        {
            if ((*(*matrixPtr)[i])[j] > maxValue)
            {
                maxValue = (*(*matrixPtr)[i])[j];
                maxi = i;
                maxj = j;
            }
        }
       
    }
    return std::pair<size_t, size_t>(maxj, maxj);
}

int main()
{
    IvannaBaglayPathFinder test;
    test.FindSolution("inputData1.json", "outData1.json");
    return 0;
}