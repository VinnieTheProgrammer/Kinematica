#ifndef ODOMETER_H_
#define ODOMETER_H_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "OdoPercept.hpp"
#include "Widgets.hpp"
#include <vector>

namespace Model
{

	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	class Odometer : public AbstractSensor {
	public:
		inline static std::vector<wxPoint> path;
		explicit Odometer(Robot& aRobot);
		virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
		virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
		void drawOdometer(wxDC& dc);
		/**
		 *
		 */
		static void setStdDev(double aStdDev) {Odometer::stddev = aStdDev;}
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

#endif //ODOMETER_H_
