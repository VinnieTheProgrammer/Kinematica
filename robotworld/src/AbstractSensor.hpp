#ifndef ABSTRACTSENSOR_HPP_
#define ABSTRACTSENSOR_HPP_

#include "Config.hpp"

#include "ModelObject.hpp"

#include <mutex>
#include <thread>


namespace Model
{
	/**
	 * Invalid angle
	 */
	const double noAngle = std::numeric_limits<double>::infinity();
	/**
	 * No distance measured, i.e. the noObject distance
	 */
	const int noDistance = -1;
	/**
	 * No object detected
	 */
	const int noObject = -2;
	/**
	 * Invalid distance
	 */
	const int invalidDistance = -3;

	class AbstractAgent;
	typedef std::shared_ptr< AbstractAgent > AbstractAgentPtr;

	/**
	 *
	 */
	class AbstractStimulus
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractStimulus()  = default;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const {return "AbstractStimulus";}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const  {return "AbstractStimulus";}

	};
	// class AbstractStimulus
	/**
	 *
	 */
	class AbstractPercept
	{
		public:
			/**
			 *
			 */
			virtual ~AbstractPercept() = default;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const {return "AbstractPercept";}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const  {return "AbstractPercept";}
	};
	// class AbstractPercept

	class AbstractSensor : public ModelObject
	{
		public:
			/**
			 *
			 */
			explicit AbstractSensor( AbstractAgent& anAgent);
			/**
			 *
			 */
			virtual ~AbstractSensor() = default;
			/**
			 * A sensor reads 10 stimuli/second (it sleeps for 100 ms) by default
			 */
			virtual void setOn( unsigned long aSleepTime = 100);
			/**
			 *
			 */
			virtual void setOff();
			/**
			 * Get the raw measurements. This typically is done in the low level driver.
			 */
			virtual std::shared_ptr< AbstractStimulus > getStimulus() const = 0;
			/**
			 * Translate the stimulus into a percept. This typically is done in the high level driver.
			 */
			virtual std::shared_ptr< AbstractPercept > getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractPercepts) const = 0;
			/**
			 * Add the percepts to the AbstractAgent's perceptQueue
			 */
			virtual void sendPercept( std::shared_ptr< AbstractPercept > anAbstractPercept);
			/**
			 * Read the stimulus after aSleepingTime and send it after translating it to a percept to the Agent.
			 */
			virtual void run( unsigned long aSleepTime);
			/**
			 *
			 */
			virtual void attachAgent( AbstractAgent& anAgent);
			/**
			 *
			 */
			virtual void detachAgent();
			/**
			 *
			 */
			virtual void reset(){}
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override;
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override;
			//@}

		protected:
			AbstractAgent* agent;
			bool running;
			std::thread sensorThread;
			mutable std::recursive_mutex sensorMutex;

		private:
	};
// class AbstractSensor
}// namespace Model

#endif // ABSTRACTSENSOR_HPP_
