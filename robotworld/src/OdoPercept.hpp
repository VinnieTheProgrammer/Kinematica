#ifndef ODOPERCEPT_H_
#define ODOPERCEPT_H_

#include "Config.hpp"

#include "Point.hpp"

#include <limits>

namespace Model
{
	/**
	 *
	 */
	class OdoPercept : public AbstractPercept
	{
		public:
			OdoPercept() = default;
			/**
			 *
			 */
			explicit OdoPercept( const wxPoint& aPoint) :
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
				return "OdoPercept: " + std::to_string(point.x) + ", " + std::to_string(point.y);
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

#endif /* ODOPERCEPT_H_ */
