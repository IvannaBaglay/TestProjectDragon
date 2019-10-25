#include "IGalaxyPathFinder.h"
#include "json.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <deque>
#include <algorithm>
#include <memory>
#include <iterator>

#define USELESS -1

using json = nlohmann::json;
typedef std::vector <std::unique_ptr<std::vector<float>>> ptr_to_matrix;

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
    SimpleRoute(size_t i, std::vector<box> boxes, targetPoint targetPoints):
        pointsInSimpleRoute_(i), boxes_(boxes), pointIdInFileJson_(targetPoints.id_), isEndOrStartPointInRoute_(true) {}

    size_t pointsInSimpleRoute_;
    int pointIdInFileJson_;
    bool isEndOrStartPointInRoute_;
    std::vector<box> boxes_;
};

struct NewRoute
{
    NewRoute(std::deque<size_t> route, float way, float weightOfShip = 0, std::vector<box> boxes = {}) :
        pointsInRoute_(route), way_(way), weightOfShip_(weightOfShip), boxesOfShip_(boxes) {}

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
};

class Matrix
{
public:
    Matrix() {}

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

	const size_t get_size_matrix() const
	{
		return sizeMatrix_;
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
	std::vector<SimpleRoute> listOfSimpleRoutes;
	std::vector<NewRoute> listOfNewRoutes;
	std::vector<box> boxes_;
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
    void CreateMatrixForAlgorithm();
    void LoadInformationAboutSimpleRoutes();
	void LoadFirstInformationAboutNewRoutes();
    void FindShortestRoutes();
	void UniteSimpleRoute(std::pair<size_t, size_t> pairOfPointer);
	void ChangeInformationAboutSimpleWay(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints);
    std::pair<size_t, size_t> FindMaxFromMatrixKilometerGrowth();
    std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> FindPointsInNewRoutes(std::pair<size_t, size_t> pairOfPoints);
	std::vector<NewRoute>::const_iterator FindIteratorOfNewRoute(size_t point);
	std::deque<size_t> CreateNewRoute(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints);
	float TakeNewWay(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints);
    bool AreAllConditionTrue(std::pair<size_t, size_t> pairOfPointer);
    bool AreEndOrStartPoints(std::pair<size_t, size_t> pairOfPointer);
    bool ArePointsInOneClass(std::pair<size_t, size_t> pairOfPointer);
    bool HaveEnoughResources(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute);
    bool IsOverload(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute);
    bool CanBoxesBePacked(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute);
};

void IvannaBaglayPathFinder::FindSolution(const char* inputJasonFile, const char* outputFileName)
{
    std::vector<targetPoint> shortestRoutes;
	std::ifstream i(inputJasonFile);
    json j = json::parse(i, nullptr, false);

    LoadInformationFromJson(j);
    CreateMatrixForAlgorithm();
    LoadInformationAboutSimpleRoutes();
	LoadFirstInformationAboutNewRoutes();
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
	std::sort(targetPoints_.begin(), targetPoints_.end(), [](targetPoint first, targetPoint second)
		{
			return first.id_ < second.id_;
		});
}

void IvannaBaglayPathFinder::FindShortestRoutes()
{
	size_t counterMatrixPoints = 0;
	do 
	{
		auto maxKilometerGrowth = FindMaxFromMatrixKilometerGrowth(); // return pair i j;
		counterMatrixPoints++;
		if (AreAllConditionTrue(maxKilometerGrowth))
		{
			UniteSimpleRoute(maxKilometerGrowth);
		}
	} while (counterMatrixPoints <= matrixOfKilometerGrowth_.get_size_matrix());
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
    for (size_t i = 0; i < targetPoints_.size(); i++)
    {
        int currentPoint = targetPoints_[i].id_;
        copy_if(boxes_.begin(), boxes_.end(), back_inserter(boxesForCurrentPoint), [currentPoint](box b) { return b.target_ == currentPoint; });
        listOfSimpleRoutes.push_back(SimpleRoute(i, boxesForCurrentPoint, targetPoints_[i]));
        boxesForCurrentPoint.clear();
    }
}

void IvannaBaglayPathFinder::LoadFirstInformationAboutNewRoutes()
{
	float way = 0;
	auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
	for (size_t i = 0; i < listOfSimpleRoutes.size(); i++)
	{
		way = 2 * (*(*matrixPtr)[i])[0];
		listOfNewRoutes.push_back(NewRoute({i}, way));
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
        for (size_t j = 0; j < i; j++)
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
    return std::pair<size_t, size_t>(maxi, maxj);
}

bool IvannaBaglayPathFinder::AreAllConditionTrue(std::pair<size_t, size_t> pairOfPointer) 
{
    if (AreEndOrStartPoints(pairOfPointer) && !ArePointsInOneClass(pairOfPointer))
    {
        auto pairOfNewRoute = FindPointsInNewRoutes(pairOfPointer);
		return HaveEnoughResources(*pairOfNewRoute.first, *pairOfNewRoute.second, pairOfPointer.first, pairOfPointer.second) /*&& IsOverload(pairOfPoints.first, pairOfPoints.second) && CanBoxesBePacked(pairOfPoints.first, pairOfPoints.second)*/;
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

bool IvannaBaglayPathFinder::HaveEnoughResources(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute)
{
    auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
    float newRoute;
    if (pointInFirstRoute < pointInSecondRoute)
    {
        newRoute = firstRoute.way_ - (*(*matrixPtr)[pointInFirstRoute])[0] + secondRoute.way_ - (*(*matrixPtr)[pointInSecondRoute])[0] + (*(*matrixPtr)[pointInSecondRoute])[pointInFirstRoute];
    }
    else
    {
        newRoute = firstRoute.way_ - (*(*matrixPtr)[pointInFirstRoute])[0] + secondRoute.way_ - (*(*matrixPtr)[pointInSecondRoute])[0] + (*(*matrixPtr)[pointInFirstRoute])[pointInSecondRoute];
    }
    return (newRoute < myship_.maxFuelWeight_ / myship_.resourcesConsumption_) ? true : false;
}

bool IvannaBaglayPathFinder::IsOverload(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute)
{
    return true;
}
bool IvannaBaglayPathFinder::CanBoxesBePacked(NewRoute firstRoute, NewRoute secondRoute, size_t pointInFirstRoute, size_t pointInSecondRoute)
{
    // ????????????????????
    return true;
}

std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> IvannaBaglayPathFinder::FindPointsInNewRoutes(std::pair<size_t, size_t> pairOfPointer)
{
	auto itFirst = FindIteratorOfNewRoute(pairOfPointer.first);
    auto itSecond = FindIteratorOfNewRoute(pairOfPointer.second);
    return std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> { itFirst, itSecond};
}

std::vector<NewRoute>::const_iterator IvannaBaglayPathFinder::FindIteratorOfNewRoute(size_t point)
{
	auto it = std::find_if(listOfNewRoutes.cbegin(), listOfNewRoutes.cend(), [&](NewRoute newRoute)
		{
			auto it = std::find(newRoute.pointsInRoute_.cbegin(), newRoute.pointsInRoute_.cend(), point);
			return (it != newRoute.pointsInRoute_.end()) ? true : false;
		});
	return it;
}

void IvannaBaglayPathFinder::UniteSimpleRoute(std::pair<size_t, size_t> pairOfPoints)
{
   
    auto pairOfNewRoute = FindPointsInNewRoutes(pairOfPoints);
	std::deque<size_t> OrderPointsInNewWay = CreateNewRoute(pairOfNewRoute, pairOfPoints);
	float newWay = TakeNewWay(pairOfNewRoute, pairOfPoints);
	ChangeInformationAboutSimpleWay(pairOfNewRoute, pairOfPoints);
	listOfNewRoutes.erase(FindIteratorOfNewRoute(pairOfPoints.first));
	listOfNewRoutes.erase(FindIteratorOfNewRoute(pairOfPoints.second));
	listOfNewRoutes.push_back(NewRoute(OrderPointsInNewWay, newWay));	
}

std::deque<size_t> IvannaBaglayPathFinder::CreateNewRoute(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints)
{
	auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
	std::deque<size_t> OrderPointsInNewWay = pairOfNewRoute.first->pointsInRoute_;
	if (pairOfNewRoute.first->pointsInRoute_[0] == pairOfPoints.first)
	{
		auto it = OrderPointsInNewWay.begin();
		if (pairOfNewRoute.second->pointsInRoute_[0] != pairOfPoints.second)
		{
			OrderPointsInNewWay.insert(it, pairOfNewRoute.second->pointsInRoute_.begin(), pairOfNewRoute.second->pointsInRoute_.end());
		}
		else
		{
			std::deque<size_t> reverseDeque(pairOfNewRoute.second->pointsInRoute_.rbegin(), pairOfNewRoute.second->pointsInRoute_.rend());
			OrderPointsInNewWay.insert(it, reverseDeque.begin(), reverseDeque.end());
		}
	}
	else
	{
		auto it = OrderPointsInNewWay.end();
		if (pairOfNewRoute.second->pointsInRoute_[0] == pairOfPoints.second)
		{
			OrderPointsInNewWay.insert(it, pairOfNewRoute.second->pointsInRoute_.begin(), pairOfNewRoute.second->pointsInRoute_.end());
		}
		else
		{
			std::deque<size_t> reverseDeque(pairOfNewRoute.second->pointsInRoute_.rbegin(), pairOfNewRoute.second->pointsInRoute_.rend());
			OrderPointsInNewWay.insert(it, reverseDeque.begin(), reverseDeque.end());
		}
	}
	return OrderPointsInNewWay;
}

float IvannaBaglayPathFinder::TakeNewWay(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints)
{
	auto matrixPtr = matrixOfKilometerBetweenPoints_.get_matrix_ptr();
	float newWay;
	if (pairOfPoints.first < pairOfPoints.second)
	{
		newWay = pairOfNewRoute.first->way_ - (*(*matrixPtr)[pairOfPoints.first])[0] + pairOfNewRoute.second->way_ - (*(*matrixPtr)[pairOfPoints.second])[0] + (*(*matrixPtr)[pairOfPoints.second])[pairOfPoints.first];
	}
	else
	{
		newWay = pairOfNewRoute.first->way_ - (*(*matrixPtr)[pairOfPoints.first])[0] + pairOfNewRoute.second->way_ - (*(*matrixPtr)[pairOfPoints.second])[0] + (*(*matrixPtr)[pairOfPoints.first])[pairOfPoints.second];
	}
	return newWay;
}

void IvannaBaglayPathFinder::ChangeInformationAboutSimpleWay(std::pair<std::vector<NewRoute>::const_iterator, std::vector<NewRoute>::const_iterator> pairOfNewRoute, std::pair<size_t, size_t> pairOfPoints)
{
	if (FindIteratorOfNewRoute(pairOfPoints.first)->pointsInRoute_.size() > 1)
	{
		listOfSimpleRoutes[pairOfPoints.first].isEndOrStartPointInRoute_ = false;
	}
	if (FindIteratorOfNewRoute(pairOfPoints.second)->pointsInRoute_.size() > 1)
	{
		listOfSimpleRoutes[pairOfPoints.second].isEndOrStartPointInRoute_ = false;
	}
}

int main()
{
    IvannaBaglayPathFinder test1;
	IvannaBaglayPathFinder test2;
	IvannaBaglayPathFinder test3;
	IvannaBaglayPathFinder test4;
	IvannaBaglayPathFinder test5;
    test1.FindSolution("inputData1.json", "outData1.json");
	test2.FindSolution("inputData2.json", "outData1.json");
	test3.FindSolution("inputData3.json", "outData1.json");
	test4.FindSolution("inputData4.json", "outData1.json");
	test5.FindSolution("inputData5.json", "outData1.json");
    return 0;
}