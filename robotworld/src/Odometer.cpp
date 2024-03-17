
#include "Odometer.hpp"

#include "Logger.hpp"
#include "Robot.hpp"
#include "RobotWorld.hpp"
#include "Wall.hpp"
#include "Shape2DUtils.hpp"
#include "MathUtils.hpp"
#include <random>
#include "OdoStimulus.hpp"
#include "OdoPercept.hpp"
#include "Configurator.hpp"

namespace Model
{

unsigned short Odometer::stddev = 0;

Odometer::Odometer(Robot &aRobot): AbstractSensor( aRobot) {
	Odometer::setStdDev(Configurator::getCompassStdev());
}

std::shared_ptr<AbstractStimulus> Odometer::getStimulus() const {
	Robot* robot = dynamic_cast<Robot*>(agent);
	auto stimulus = std::make_shared<OdoStimulus>();
	if(robot)
	{
		std::random_device rd{};
		std::mt19937 gen{rd()};
	    std::normal_distribution<> noise{0,Odometer::stddev};
		double angle = std::abs(Utils::MathUtils::toDegrees(Utils::Shape2DUtils::getAngle( robot->getFront())) + noise(gen));
		float speed = robot->getSpeed() + noise(gen);

		// add this to stimulus
		stimulus->angle = angle;
		stimulus->distance = speed;

		return stimulus;
	}
	return std::make_shared< OdoStimulus >( noAngle,noDistance);
}

std::shared_ptr<AbstractPercept> Odometer::getPerceptFor(
	std::shared_ptr<AbstractStimulus> anAbstractStimulus) const {
	Robot* robot = dynamic_cast< Robot* >( agent);
	auto percept = std::make_shared<OdoPercept>();
	 std::shared_ptr<OdoStimulus> stimulus = std::dynamic_pointer_cast<OdoStimulus>(anAbstractStimulus);
	if (robot)
	{
		if(stimulus) {
			wxPoint endpoint{	static_cast< int >( robot->getPosition().x + std::cos( Utils::MathUtils::toRadians(stimulus->angle))*stimulus->distance),
				static_cast< int >( robot->getPosition().y + std::sin( Utils::MathUtils::toRadians(stimulus->angle))*stimulus->distance)};
			percept->point = endpoint;
		}
	}
	return percept;
}

std::string Odometer::asString() const {
	return "Odometer";
}

std::string Odometer::asDebugString() const {
	return asString();
}

void Odometer::drawOdometer(wxDC& dc) {
	Robot* robot = dynamic_cast<Robot*>(agent);
	if(robot) {
		if(robot->currectOdometerReading.size() < 1) {return;}
		
		for(size_t i = 0; i < robot->currectOdometerReading.size() -1; ++i) {
			dc.SetPen( wxPen(  "BLUE", 2, wxPENSTYLE_SOLID));
			wxPoint currentPoint = robot->currectOdometerReading[i];
			wxPoint nextPoint = robot->currectOdometerReading[i + 1];
			dc.DrawLine(currentPoint, nextPoint);
		}

	}
}

}

