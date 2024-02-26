#ifndef DISTANCESTIMULUS_HPP_
#define DISTANCESTIMULUS_HPP_

#include "Config.hpp"

#include "AbstractSensor.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class DistanceStimulus : public AbstractStimulus
	{
		public:
			/**
			 *
			 */
			DistanceStimulus( 	double anAngle,
								double aDistance) :
				angle(anAngle),
				distance( aDistance)
			{
			}
			double angle;
			double distance;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "DistancePercept: " + std::to_string(angle) + ", " + std::to_string(distance);
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	}; // class DistanceStimulus
} // namespace Model

#endif /* DISTANCESTIMULUS_HPP_ */
