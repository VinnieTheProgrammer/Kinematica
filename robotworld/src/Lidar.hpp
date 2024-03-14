#ifndef LIDAR_H_
#define LIDAR_H_

#include "Config.hpp"

#include "AbstractSensor.hpp"
#include "DistancePercept.hpp"
#include "Widgets.hpp"

namespace Model
{

	/**
	 * Compile time configurable length of the laser beam
	 */
	const short int lidarBeamLength = 1024;


	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	class Lidar : public AbstractSensor {
	public:
		explicit Lidar(Robot& aRobot);
		virtual std::shared_ptr< AbstractStimulus > getStimulus() const override;
		virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const override;
		void drawLidar(wxDC& dc);
		/**
		 * 
		 */
		static void setStdDev(unsigned short aStdDev) {Lidar::stddev = aStdDev;}
		/**
		 *
		 */
		static unsigned short getStdDev(){ return stddev;}

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
		static unsigned short stddev;

	};

}

#endif //LIDAR_H_
