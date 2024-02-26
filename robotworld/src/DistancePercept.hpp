#ifndef DISTANCEPERCEPT_HPP_
#define DISTANCEPERCEPT_HPP_

#include "Config.hpp"

#include "DistanceStimulus.hpp"
#include "Point.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class DistancePercept : public AbstractPercept
	{
		public:
			/**
			 *
			 */
			explicit DistancePercept( const wxPoint& aPoint) :
				point(aPoint)
			{
			}
			wxPoint point;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "DistancePercept: " + std::to_string(point.x) + ", " + std::to_string(point.y);
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	}; //	class DistancePercept
} // namespace Model

#endif /* DISTANCEPERCEPT_HPP_ */
