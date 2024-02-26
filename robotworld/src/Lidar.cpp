
#include "Lidar.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include <random>
#include "DistanceStimuli.hpp"

namespace Model
{

double Lidar::stddev = 10.0;

Lidar::Lidar(Robot &aRobot): AbstractSensor( aRobot) {}

std::shared_ptr<AbstractStimulus> Lidar::getStimulus() const {
	Robot* robot = dynamic_cast<Robot*>(agent);
	auto stimuli = std::make_shared<DistanceStimuli>();
	if(robot)
	{
		std::random_device rd{};
		std::mt19937 gen{rd()};
	    std::normal_distribution<> noise{0,Lidar::stddev};

		std::vector< WallPtr > walls = RobotWorld::getRobotWorld().getWalls();
		for(double angle = 0; angle < 360; angle += 2) {
			for (std::shared_ptr< Wall > wall : walls)
			{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint robotLocation = robot->getPosition();
				wxPoint laserEndpoint{static_cast<int>(robotLocation.x + std::cos( Utils::MathUtils::toRadians(angle)) * lidarBeamLength + noise(gen)) ,
									static_cast<int>(robotLocation.y + std::sin( Utils::MathUtils::toRadians(angle)) * lidarBeamLength + noise(gen))};

				wxPoint interSection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, robotLocation, laserEndpoint);

				if(interSection != wxDefaultPosition)
				{
					double distance = Utils::Shape2DUtils::distance(robotLocation,interSection);
					stimuli->stimuli.push_back(DistanceStimulus(angle,distance));
				}
			}
		}
		return stimuli;
	}
	return std::make_shared< DistanceStimulus >( noAngle,noDistance);
}

std::shared_ptr<AbstractPercept> Lidar::getPerceptFor(
		std::shared_ptr<AbstractStimulus> anAbstractStimulus) const {
	Robot* robot = dynamic_cast< Robot* >( agent);
	auto percepts = std::make_shared<DistancePercepts>();
	 std::shared_ptr<DistanceStimuli> stimuli = std::dynamic_pointer_cast<DistanceStimuli>(anAbstractStimulus);
	if (robot)
	{
		wxPoint robotLocation = robot->getPosition();
		if(stimuli) {
			for(long unsigned int i = 0; i < stimuli->stimuli.size(); ++i) {
				if(stimuli->stimuli[i].distance == noDistance)
				{
					percepts->pointCloud.push_back(DistancePercept(wxPoint(noObject,noObject)));
				}
				wxPoint endpoint{	static_cast< int >( robotLocation.x + std::cos( Utils::MathUtils::toRadians(stimuli->stimuli[i].angle))*stimuli->stimuli[i].distance),
								static_cast< int >( robotLocation.y + std::sin( Utils::MathUtils::toRadians(stimuli->stimuli[i].angle))*stimuli->stimuli[i].distance)};

				percepts->pointCloud.push_back(DistancePercept(endpoint));
			}
		}
	}

	return percepts;
}

std::string Lidar::asString() const {
	return "Lidar";
}

std::string Lidar::asDebugString() const {
	return asString();
}

void Lidar::drawLidar(wxDC& dc) {
	Robot* robot = dynamic_cast<Robot*>(agent);
	auto stimuli = std::make_shared<DistanceStimuli>();
	if(robot) {
		std::random_device rd{};
		std::mt19937 gen{rd()};
	    std::normal_distribution<> noise{0,Lidar::stddev};

		// loop over all the walls
		// check if there is a intersection


		std::vector< WallPtr > walls = RobotWorld::getRobotWorld().getWalls();
		for(double angle = 0; angle < 360; angle += 2) {
			std::vector<wxPoint> intersections;
			wxPoint robotLocation = robot->getPosition();
			wxPoint laserEndpoint{static_cast<int>(robotLocation.x + std::cos( Utils::MathUtils::toRadians(angle)) * (lidarBeamLength + noise(gen))) ,
								static_cast<int>(robotLocation.y + std::sin( Utils::MathUtils::toRadians(angle)) * (lidarBeamLength + noise(gen)))};
			for (std::shared_ptr< Wall > wall : walls)
			{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();

				wxPoint interSection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, robotLocation, laserEndpoint);
				if(interSection != wxDefaultPosition) {
					intersections.push_back(interSection);
				}

			}

			if(intersections.size() == 1) {
				for(wxPoint intersection : intersections) {
					dc.SetPen( wxPen(  "RED", 2, wxPENSTYLE_SOLID));
					dc.DrawCircle(intersection.x + noise(gen), intersection.y + noise(gen), 2);
					//dc.DrawLine( robotLocation.x, robotLocation.y, intersection.x + noise(gen), intersection.y + noise(gen));
				}
			} else if(intersections.size() > 1) {
				// Intersects with more than one wall. Find the wall closest to the robot
				std::vector<short> distances;
				for(wxPoint intersection : intersections) {
					short distance = static_cast<short>(std::abs(robotLocation.x - intersection.x) + std::abs(robotLocation.y - intersection.y));
					distances.push_back(distance);
				}
				long index = std::distance(std::begin(distances), std::min_element(std::begin(distances), std::end(distances)));

				dc.SetPen( wxPen(  "RED", 2, wxPENSTYLE_SOLID));
				dc.DrawCircle(intersections[index].x + noise(gen), intersections[index].y + noise(gen),2);
				//dc.DrawLine( robotLocation.x, robotLocation.y, intersections[index].x + noise(gen), intersections[index].y + noise(gen));
			} else {
					dc.SetPen( wxPen(  "RED", 2, wxPENSTYLE_SOLID));
					dc.DrawCircle(laserEndpoint.x + noise(gen), laserEndpoint.y + noise(gen),2);
					//dc.DrawLine( robotLocation.x, robotLocation.y, laserEndpoint.x + noise(gen), laserEndpoint.y + noise(gen));
			}
		}
	}
}

}

