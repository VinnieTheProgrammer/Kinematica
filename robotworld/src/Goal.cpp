#include "Goal.hpp"

#include "Logger.hpp"
#include "Shape2DUtils.hpp"

#include <sstream>

namespace Model
{
	/**
	 *
	 */
	Goal::Goal( const std::string& aName) :
								WayPoint( aName)
	{
	}
	/**
	 *
	 */
	Goal::Goal( const std::string& aName,
				const wxPoint& aPosition) :
								WayPoint( aName, aPosition)
	{
	}
	/**
	 *
	 */
	/**
	 *
	 */
	std::string Goal::asString() const
	{
		std::ostringstream os;

		os << "Goal " << getName() << " at (" << getPosition().x << "," << getPosition().y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Goal::asDebugString() const
	{
		std::ostringstream os;

		os << "Goal:\n";
		os << WayPoint::asDebugString();

		return os.str();
	}
} // namespace Model
