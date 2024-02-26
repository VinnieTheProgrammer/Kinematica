#ifndef ANGLEPERCEPT_HPP_
#define ANGLEPERCEPT_HPP_

#include "Config.hpp"

#include "DistanceStimulus.hpp"
#include "Point.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class AnglePercept : public AbstractPercept
	{
		public:

			AnglePercept() = default;
			/**
			 *
			 */
			explicit AnglePercept( const double& aAngle) :
				angle(aAngle)
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
				return "AnglePercept: " + std::to_string(angle);
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
