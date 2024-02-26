
#include "LidarSensor.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"

#include <random>

namespace Model
{
	/**
	 *
	 */
	/* static */ double LidarSensor::stddev = 10.0;
	/**
	 *
	 */
	LidarSensor::LidarSensor( Robot& aRobot) :
								AbstractSensor( aRobot)
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > LidarSensor::getStimulus() const
	{
		Robot* robot = dynamic_cast<Robot*>(agent);
		if(robot)
		{
			std::random_device rd{};
			std::mt19937 gen{rd()};
		    std::normal_distribution<> noise{0,LidarSensor::stddev};
			Stimuli stimuli;
			Utils::MathUtils utils;

			std::vector< WallPtr > walls = RobotWorld::getRobotWorld().getWalls();
			for (std::shared_ptr< Wall > wall : walls)
			{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint robotLocation = robot->getPosition();

				for(int i = 0; i < 360; i += 2) {
					wxPoint laserEndpoint{static_cast<int>(robotLocation.x + std::cos(utils.toRadians((double)i)) * (lidarBeamLength + noise(gen))) ,
					static_cast<int>(robotLocation.y + std::sin(utils.toRadians((double)i)) * (lidarBeamLength + noise(gen)))};
					wxPoint interSection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, robotLocation, laserEndpoint);
					if(interSection != wxDefaultPosition)
					{
						double distance = Utils::Shape2DUtils::distance(robotLocation,interSection);
						DistanceStimulus stim(utils.toRadians((double)i), distance);
						stimuli.push_back(stim);
						
					}
				}

			}
			return std::make_shared<DistanceStimuli> (stimuli);
		}
		return std::make_shared< DistanceStimulus >( noAngle,noDistance);
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > LidarSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		Robot* robot = dynamic_cast< Robot* >( agent);
		if (robot)
		{
			wxPoint robotLocation = robot->getPosition();
			PointCloud cloud;
			DistanceStimuli* distanceStimuli = dynamic_cast< DistanceStimuli* >( anAbstractStimulus.get());

			for(int i = 0; i < distanceStimuli->stimuli.size(); ++i) {
				if(distanceStimuli->stimuli[i].distance == noDistance)
				{
					DistancePercept percept(wxPoint(noObject,noObject));
					cloud.push_back(percept);
				}
				wxPoint endpoint{	static_cast< int >( robotLocation.x + std::cos( distanceStimuli->stimuli[i].angle)*distanceStimuli->stimuli[i].distance),
								static_cast< int >( robotLocation.y + std::sin( distanceStimuli->stimuli[i].angle)*distanceStimuli->stimuli[i].distance)};
				DistancePercept percept(endpoint);
				cloud.push_back(percept);

			}
			return std::make_shared<DistancePercepts>(cloud);

		}

		return std::make_shared<DistancePercept>( wxPoint(invalidDistance,invalidDistance));
	}
	/**
	 *
	 */
	std::string LidarSensor::asString() const
	{
		return "LidarSensor";
	}
	/**
	 *
	 */
	std::string LidarSensor::asDebugString() const
	{
		return asString();
	}
} // namespace Model
