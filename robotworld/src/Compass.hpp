#ifndef COMPASS_H_
#define COMPASS_H_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "AnglePercept.hpp"
#include "Widgets.hpp"

namespace Model
{

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	class Compass : public AbstractSensor {
	public:
		explicit Compass(Robot& aRobot);
		virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
		virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
		void drawCompass(wxDC& dc);
		/**
		 *
		 */
		static void setStdDev(double aStdDev) {Compass::stddev = aStdDev;}
		/**
		 *
		 */
		static double getStdDev(){ return stddev;}

		/**
		 * Returns a 1-line description of the object
		 */
		virtual std::string asString() const override;
		/**
		 * Returns a description of the object with all data of the object usable for debugging
		 */
		virtual std::string asDebugString() const override;
	private:
		/**
		 * Standard deviation of the odometer per 10 pixels
		 */
		static double stddev;

	};

}

#endif //COMPASS_H_
