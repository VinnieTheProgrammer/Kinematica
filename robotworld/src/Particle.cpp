#include "Particle.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include "RobotWorld.hpp"
#include "Lidar.hpp"
#include <random>
#include "Robot.hpp"
#include <Odometer.hpp>

Particle::Particle(const wxPoint & position): position(position) {
    weight = 0;
}

void Particle::collectMeasurements() {
	lidarMeasurements.clear();
    // get Lidar data
    std::vector< Model::WallPtr > walls = Model::RobotWorld::getRobotWorld().getWalls();
		for(double angle = 0; angle < 360; angle += 2) {
			std::vector<wxPoint> intersections;
            std::vector<double> angles;
			for (std::shared_ptr< Model::Wall > wall : walls)
			{
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint laserEndpoint{static_cast<int>(position.x + std::cos( Utils::MathUtils::toRadians(angle)) * Model::lidarBeamLength) ,
									static_cast<int>(position.y + std::sin( Utils::MathUtils::toRadians(angle)) * Model::lidarBeamLength)};

				wxPoint intersection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, position, laserEndpoint);
				if(intersection != wxDefaultPosition)
				{
					intersections.push_back(intersection);
                    angles.push_back(angle);
				} else {
					intersections.push_back(laserEndpoint);
					angles.push_back(angle);
				}

			}
			if(intersections.size() > 1) {
				// Intersects with more than one wall. Find the wall closest to the robot
				std::vector<short> distances;
				for(wxPoint intersection : intersections) {
					short distance = static_cast<short>(std::abs(position.x - intersection.x) + std::abs(position.y - intersection.y));
					distances.push_back(distance);
				}
				int index = std::distance(std::begin(distances), std::min_element(std::begin(distances), std::end(distances)));
				double distance = Utils::Shape2DUtils::distance(position,intersections[index]);
                wxPoint endpoint{	static_cast< int >( position.x + std::cos( Utils::MathUtils::toRadians(angles[index]))* distance),
								static_cast< int >( position.y + std::sin( Utils::MathUtils::toRadians(angles[index]))* distance)};
                lidarMeasurements.push_back(endpoint);
			} else if(intersections.size() == 1) {
				double distance = Utils::Shape2DUtils::distance(position,intersections[0]);
                wxPoint endpoint{	static_cast< int >( position.x + std::cos( Utils::MathUtils::toRadians(angles[0]))* distance),
								static_cast< int >( position.y + std::sin( Utils::MathUtils::toRadians(angles[0]))* distance)};
                lidarMeasurements.push_back(endpoint);
			}

			// calc update 
			std::random_device rd{};
			std::mt19937 gen{rd()};
			std::normal_distribution<> noise{0,Model::Odometer::getStdDev()};
			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot("Robot");
			double updateAngle = std::abs(Utils::MathUtils::toDegrees(Utils::Shape2DUtils::getAngle( robot->getFront())) + noise(gen));
			float speed = robot->getSpeed() + noise(gen);
			update = std::make_pair(updateAngle,speed);
		}
}