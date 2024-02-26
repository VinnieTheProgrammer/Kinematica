#ifndef ANGLESTIMULUS_HPP__
#define ANGLESTIMULUS_HPP__

#include "Config.hpp"

#include "AbstractSensor.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class AngleStimulus : public AbstractStimulus
	{
		public:

			AngleStimulus() = default;
			/**
			 *
			 */
			AngleStimulus( 	double anAngle) :
				angle(anAngle)
			{
			}
			double angle;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "AngleStimuus: "  + std::to_string(angle);
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	};
}

#endif
