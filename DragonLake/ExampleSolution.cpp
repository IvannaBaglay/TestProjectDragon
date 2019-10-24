#include "IGalaxyPathFinder.h"
#include "json.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <algorithm>
#include <memory>

#define USELESS -1

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

struct SimpleRoute
{
    SimpleRoute(int i, std::vector<box> boxes, targetPoint targetPoints):
        pointsInSimpleRoute_(i), boxes_(boxes), pointIdInFileJson_(targetPoints.id_), isEndOrStartPointInRoute_(true) {}

    int pointsInSimpleRoute_;
    int pointIdInFileJson_;
    bool isEndOrStartPointInRoute_;
    std::vector<box> boxes_;
};

struct NewRoute
{
    /*
    constructor
    */
    NewRoute(std::deque<size_t> route, float way, float weightOfShip = 0, std::vector<box> boxes = {}) :
        pointsInRoute_(route), way_(way), weightOfShip_(weightOfShip), boxesOfShip_(boxes)
    {

    }
    std::deque<size_t> pointsInRoute_;
    float way_;
    float weightOfShip_;
    std::vector<box> boxesOfShip_;
    bool ArePairOfPointsInRoute(std::pair<size_t, size_t> pairOfPoint)
    {
        auto itFirst = std::find(pointsInRoute_.cbegin(), pointsInRoute_.cend(), pairOfPoint.first);
        auto itSecond = std::find(pointsInRoute_.cbegin(), pointsInRoute_.cend(), pairOfPoint.second);
        return (itFirst != pointsInRoute_.end() && itSecond != pointsInRoute_.end()) ? true : false;
    }
    bool IsPointInRoute(size_t point)
    {
        auto it = std::find(pointsInRoute_.cbegin(), pointsInRoute_.cend(), point);
        return (it != pointsInRoute_.end()) ? true : false;
    }

    /*
    function for checked pair in deque;
    function for checked one part of pair 

    added new point or new set to set

    */
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

struct IvannaBaglayPathFinder : public IGalaxyPathFinder
{
private:
	std::vector<box> boxes_; // added only on base
	std::vector<targetPoint> targetPoints_;
	ship myship_;

    Matrix matrixOfKilometerBetweenPoints_;
    Matrix matrixOfKilometerGrowth_;
    std::vector<SimpleRoute> listOfSimpleRoutes;
    std::vector<NewRoute> listOfNewRoutes;
public:
	virtual void FindSolution(const char* inputJasonFile, const char* outputFileName);
	virtual const char* ShowCaptainName() { return "Ivanna Baglay"; }

    void LoadInformationFromJson(json& j);
    void LoadInformationAboutShipFromJson(json& j);
    void LoadInformationAboutTargetPointFromJson(json& j);
    void LoadInformationAboutBoxFromJson(json& j);
    void CreateMatrixForAlgorithm();
    void LoadInformationAboutSimpleRoutes();
    void FindShortestRoutes();
    std::pair<size_t, size_t> FindMaxFromMatrixKilometerGrowth();
    std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> FindPointsInNewRoutes(std::pair<size_t, size_t> pairOfPointer);
    bool AreAllConditionTrue(std::pair<size_t, size_t> pairOfPointer);
    bool AreEndOrStartPoints(std::pair<size_t, size_t> pairOfPointer);
    bool ArePointsInOneClass(std::pair<size_t, size_t> pairOfPointer);

    bool HaveEnoughResources(size_t firstPoint, size_t secondPoint);
    bool HaveEnoughResources(size_t firstPoint, size_t secondPoint, NewRoute route);
    bool HaveEnoughResources(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute);

    bool IsOverload(size_t firstPoint, size_t secondPoint);
    bool IsOverload(size_t point, NewRoute route);
    bool IsOverload(NewRoute firstRoute, NewRoute secondRoute);
   
    bool CanBoxesBePacked(size_t firstPoint, size_t secondPoint);
    bool CanBoxesBePacked(size_t point, NewRoute route);
    bool CanBoxesBePacked(NewRoute firstRoute, NewRoute secondRoute);

    void UniteSimpleRoute(std::pair<size_t, size_t> pairOfPointer);

};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
    std::vector<targetPoint> shortestRoutes;
	std::ifstream i(inputJasonFile);

    json j = json::parse(i, nullptr, false);
	// do some stuff
    LoadInformationFromJson(j);
    CreateMatrixForAlgorithm();
    LoadInformationAboutSimpleRoutes();

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
    if (AreAllConditionTrue(maxKilometerGrowth))
    {
        UniteSimpleRoute(maxKilometerGrowth);
    }
}

void IvannaBaglayPathFinder::CreateMatrixForAlgorithm()
{
    matrixOfKilometerBetweenPoints_.CreateMartix(targetPoints_.size());
    matrixOfKilometerGrowth_.CreateMartix(targetPoints_.size());
    matrixOfKilometerBetweenPoints_.FitMatrix(targetPoints_);
    matrixOfKilometerGrowth_.FitMatrix(matrixOfKilometerBetweenPoints_);

}

void IvannaBaglayPathFinder::LoadInformationAboutSimpleRoutes()
{
    std::vector<box> boxesForCurrentPoint;
    for (size_t i = 1; i < targetPoints_.size(); i++)
    {
        int currentPoint = targetPoints_[i].id_;
        copy_if(boxes_.begin(), boxes_.end(), back_inserter(boxesForCurrentPoint), [currentPoint](box b) { return b.target_ == currentPoint; });
        listOfSimpleRoutes.push_back(SimpleRoute(i, boxesForCurrentPoint, targetPoints_[i]));
        boxesForCurrentPoint.clear();
    }
}

std::pair<size_t, size_t> IvannaBaglayPathFinder::FindMaxFromMatrixKilometerGrowth()
{
    size_t rowMatrix = matrixOfKilometerGrowth_.get_row();
    size_t maxj = 0;
    size_t maxi = 0;  
    float maxValue = -1;
    auto matrixPtr = matrixOfKilometerGrowth_.get_matrix_ptr();
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
    (*(*matrixPtr)[maxi])[maxj] = USELESS; // will not be used further
    return std::pair<size_t, size_t>(maxj, maxj);
}

bool IvannaBaglayPathFinder::AreAllConditionTrue(std::pair<size_t, size_t> pairOfPointer) 
{
    if (AreEndOrStartPoints(pairOfPointer) && !ArePointsInOneClass(pairOfPointer))
    {
        /*
        load information about route
        if one point containing in new route we must return NewRoute (checked two points)
        else we work with two points
        overload function for work with (point, point) (point, route) (route, route)
        */

        //return HaveEnoughResources(pairOfPointer) && IsOverload(pairOfPointer) && CanBoxesBePacked(pairOfPointer);
        auto pairOfNewRoute = FindPointsInNewRoutes(pairOfPointer);
        
        if (pairOfNewRoute.first == listOfNewRoutes.end() && pairOfNewRoute.second == listOfNewRoutes.end())
        {
            return HaveEnoughResources(pairOfPointer.first, pairOfPointer.second) /*&& IsOverload(pairOfPointer.first, pairOfPointer.second) && CanBoxesBePacked(pairOfPointer.first, pairOfPointer.second)*/;
        }
        else if (pairOfNewRoute.first == listOfNewRoutes.end() && pairOfNewRoute.second != listOfNewRoutes.end())
        {
            return HaveEnoughResources(pairOfPointer.first, pairOfPointer.second, *pairOfNewRoute.second) /*&& IsOverload(pairOfPointer.first, pairOfPointer.second) && CanBoxesBePacked(pairOfPointer.first, pairOfPointer.second)*/;
        }
        else if (pairOfNewRoute.first != listOfNewRoutes.end() && pairOfNewRoute.second == listOfNewRoutes.end())
        {
            return HaveEnoughResources(pairOfPointer.second, pairOfPointer.first, *pairOfNewRoute.first) /*&& IsOverload(pairOfPointer.first, pairOfPointer.second) && CanBoxesBePacked(pairOfPointer.first, pairOfPointer.second)*/;
        }
        else
        {
            return HaveEnoughResources(*pairOfNewRoute.first, *pairOfNewRoute.second, pairOfPointer.first, pairOfPointer.second) /*&& IsOverload(pairOfPointer.first, pairOfPointer.second) && CanBoxesBePacked(pairOfPointer.first, pairOfPointer.second)*/;
        }
    }
    return false;
}

bool IvannaBaglayPathFinder::AreEndOrStartPoints(std::pair<size_t, size_t> pairOfPointer)
{
    return listOfSimpleRoutes[pairOfPointer.first].isEndOrStartPointInRoute_ && listOfSimpleRoutes[pairOfPointer.second].isEndOrStartPointInRoute_;
}

bool IvannaBaglayPathFinder::ArePointsInOneClass(std::pair<size_t, size_t> pairOfPointer)
{
    for (size_t i = 0; i < listOfNewRoutes.size(); i++)
    {
        if (listOfNewRoutes[i].ArePairOfPointsInRoute(pairOfPointer))
        {
            return true;
        }
    }
    return false;
}

bool IvannaBaglayPathFinder::HaveEnoughResources(size_t firstPoint, size_t secondPoint)
{
    auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
    float newRoute;
    if (firstPoint < secondPoint)
    {
        newRoute = (*(*matrixPtr)[firstPoint])[0] + (*(*matrixPtr)[firstPoint])[secondPoint] + (*(*matrixPtr)[secondPoint])[0];
    }
    else
    {
        newRoute = (*(*matrixPtr)[firstPoint])[0] + (*(*matrixPtr)[secondPoint])[firstPoint] + (*(*matrixPtr)[secondPoint])[0];
    }
   
    return (newRoute < myship_.maxFuelWeight_ / myship_.resourcesConsumption_) ? true : false;
}

bool IvannaBaglayPathFinder::HaveEnoughResources(size_t point, size_t pointInRoute, NewRoute route)
{
    auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
    // ?????????????
    float newRoute;
    if (point < pointInRoute)
    {
        newRoute = route.way_ - (*(*matrixPtr)[pointInRoute])[0] + (*(*matrixPtr)[point])[0] + (*(*matrixPtr)[point])[pointInRoute];
    }
    else
    {
        newRoute = route.way_ - (*(*matrixPtr)[pointInRoute])[0] + (*(*matrixPtr)[point])[0] + (*(*matrixPtr)[pointInRoute])[point];
    }

    return (newRoute < myship_.maxFuelWeight_ / myship_.resourcesConsumption_) ? true : false;
}

bool IvannaBaglayPathFinder::HaveEnoughResources(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute)
{
    auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
    float newRoute;
    if (pointInFirstRoute < pointInSecondRoute)
    {
        newRoute = firstRoute.way_ - (*(*matrixPtr)[pointInFirstRoute])[0] + secondRoute.way_ - (*(*matrixPtr)[pointInSecondRoute])[0] + (*(*matrixPtr)[pointInFirstRoute])[pointInSecondRoute];
    }
    else
    {
        newRoute = firstRoute.way_ - (*(*matrixPtr)[pointInFirstRoute])[0] + secondRoute.way_ - (*(*matrixPtr)[pointInSecondRoute])[0] + (*(*matrixPtr)[pointInSecondRoute])[pointInFirstRoute];
    }
    return (newRoute < myship_.maxFuelWeight_ / myship_.resourcesConsumption_) ? true : false;
}

bool IvannaBaglayPathFinder::IsOverload(size_t firstPoint, size_t secondPoint)
{
    return true;
}
bool IvannaBaglayPathFinder::CanBoxesBePacked(size_t firstPoint, size_t secondPoint)
{
    // ????????????????????
    return true;
}

std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> IvannaBaglayPathFinder::FindPointsInNewRoutes(std::pair<size_t, size_t> pairOfPointer)
{
    auto itFirst = std::find_if(listOfNewRoutes.cbegin(), listOfNewRoutes.cend(), [&](NewRoute newRoute)
        {
            auto it = std::find(newRoute.pointsInRoute_.cbegin(), newRoute.pointsInRoute_.cend(), pairOfPointer.first);
            return (it != newRoute.pointsInRoute_.end()) ? true : false;
        });
    auto itSecond = std::find_if(listOfNewRoutes.cbegin(), listOfNewRoutes.cend(), [&](NewRoute newRoute)
        {
            auto it = std::find(newRoute.pointsInRoute_.cbegin(), newRoute.pointsInRoute_.cend(), pairOfPointer.second);
            return (it != newRoute.pointsInRoute_.end()) ? true : false;
        });
    return std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> { itFirst, itSecond};
}

void IvannaBaglayPathFinder::UniteSimpleRoute(std::pair<size_t, size_t> pairOfPointer)
{
    auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
    auto pairOfNewRoute = FindPointsInNewRoutes(pairOfPointer);
    std::deque<size_t> OrderPointsInNewWay;
    float newWay;
    /*
    checking iterators

    */

    /*
    read information:
        gives way in points vector
        sum way

        late : weight and boxes
    */

    if (pairOfNewRoute.first == listOfNewRoutes.end() && pairOfNewRoute.second == listOfNewRoutes.end())
    {
        // combine two points + push new route;
        OrderPointsInNewWay.push_back(pairOfPointer.first);
        OrderPointsInNewWay.push_back(pairOfPointer.second);
        if (pairOfPointer.first < pairOfPointer.second)
        {
            newWay = (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.first])[pairOfPointer.second] + (*(*matrixPtr)[pairOfPointer.second])[0];
        }
        else
        {
            newWay = (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.second])[pairOfPointer.first] + (*(*matrixPtr)[pairOfPointer.second])[0];
        }

        listOfNewRoutes.push_back(NewRoute(OrderPointsInNewWay, newWay));
    }
    else if (pairOfNewRoute.first != listOfNewRoutes.end() && pairOfNewRoute.second == listOfNewRoutes.end())
    {
        //  read information about first set and takes in constructor + point second 
        // erase iterator first and push new;

        OrderPointsInNewWay = pairOfNewRoute.first->pointsInRoute_;
        (pairOfNewRoute.first->pointsInRoute_[0] == pairOfPointer.first) ? OrderPointsInNewWay.push_front(pairOfPointer.second) : OrderPointsInNewWay.push_back(pairOfPointer.second);

        if (pairOfPointer.first < pairOfPointer.second)
        {
            newWay = pairOfNewRoute.first->way_ - (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.second])[0] + (*(*matrixPtr)[pairOfPointer.first])[pairOfPointer.second];

        }
        else
        {
            newWay = pairOfNewRoute.first->way_ - (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.second])[0] + (*(*matrixPtr)[pairOfPointer.second])[pairOfPointer.first];
        }
        listOfNewRoutes.erase(pairOfNewRoute.first);
        listOfNewRoutes.push_back(NewRoute(OrderPointsInNewWay, newWay));
    } 
    else if (pairOfNewRoute.first == listOfNewRoutes.end() && pairOfNewRoute.second != listOfNewRoutes.end())
    {
        //  read information about first set and takes in constructor + point first
        // erase iterator second and push new;
        OrderPointsInNewWay = pairOfNewRoute.second->pointsInRoute_;
        (pairOfNewRoute.second->pointsInRoute_[0] == pairOfPointer.second) ? OrderPointsInNewWay.push_front(pairOfPointer.first) : OrderPointsInNewWay.push_back(pairOfPointer.first);

        if (pairOfPointer.first < pairOfPointer.second)
        {
            newWay = pairOfNewRoute.second->way_ - (*(*matrixPtr)[pairOfPointer.second])[0] + (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.first])[pairOfPointer.second];

        }
        else
        {
            newWay = pairOfNewRoute.second->way_ - (*(*matrixPtr)[pairOfPointer.second])[0] + (*(*matrixPtr)[pairOfPointer.first])[0] + (*(*matrixPtr)[pairOfPointer.second])[pairOfPointer.first];
        }
        listOfNewRoutes.erase(pairOfNewRoute.second);
        listOfNewRoutes.push_back(NewRoute(OrderPointsInNewWay, newWay));
    }
    else
    {
        //  read information about both set and takes in constructor
        // erase both iterator and push new;

    }


}

int main()
{
    IvannaBaglayPathFinder test;
    test.FindSolution("inputData1.json", "outData1.json");
    return 0;
}