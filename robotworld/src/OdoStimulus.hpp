#ifndef ODOSTIMULUS_HPP_
#define ODOSTIMULUS_HPP_

#include "Config.hpp"

#include "AbstractSensor.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class OdoStimulus : public AbstractStimulus
	{
		public:
			OdoStimulus() = default;
			/**
			 *
			 */
			OdoStimulus( 	double anAngle,
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
				return "OdoStimulus: " + std::to_string(angle) + ", " + std::to_string(distance);
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	};
} // namespace Model

#endif /* ODOSTIMULUS_HPP_ */
