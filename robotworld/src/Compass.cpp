
#include "Compass.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include <random>
#include "AngleStimulus.hpp"

namespace Model
{

double Compass::stddev = 2.0;

Compass::Compass(Robot &aRobot): AbstractSensor( aRobot) {}

std::shared_ptr<AbstractStimulus> Compass::getStimulus() const {
	Robot* robot = dynamic_cast<Robot*>(agent);
	auto stimulus = std::make_shared<AngleStimulus>();
	if(robot)
	{
		std::random_device rd{};
		std::mt19937 gen{rd()};
	    std::normal_distribution<> noise{0,Compass::stddev};
		double angle = std::abs(Utils::MathUtils::toDegrees(Utils::Shape2DUtils::getAngle( robot->getFront())) + noise(gen));
		// std::cout << "compass angle: " << angle << "\n";

		// add this to stimulus
		stimulus->angle = angle;

		return stimulus;
	}
	stimulus->angle = -1.0;
	return stimulus;
}

std::shared_ptr<AbstractPercept> Compass::getPerceptFor(
		std::shared_ptr<AbstractStimulus> anAbstractStimulus) const {
	Robot* robot = dynamic_cast< Robot* >( agent);
	auto percept = std::make_shared<AnglePercept>();
	 std::shared_ptr<AngleStimulus> stimulus = std::dynamic_pointer_cast<AngleStimulus>(anAbstractStimulus);
	if (robot)
	{
		if(stimulus) {
			percept->angle = stimulus->angle;
		}
	}
	return percept;
}

std::string Compass::asString() const {
	return "Compass";
}

std::string Compass::asDebugString() const {
	return asString();
}

void Compass::drawCompass(wxDC& dc) {
	Robot* robot = dynamic_cast<Robot*>(agent);
	auto stimuli = std::make_shared<DistanceStimuli>();
	if(robot) {
		std::random_device rd{};
		std::mt19937 gen{rd()};
	    std::normal_distribution<> noise{0,Compass::stddev};
	}
}

}

