#include "WayPoint.hpp"

#include "Logger.hpp"

#include <sstream>

namespace Model
{
	/**
	 *
	 */
	WayPoint::WayPoint( const std::string& aName) :
								name( aName)
	{
	}
	/**
	 *
	 */
	WayPoint::WayPoint( const std::string& aName,
						const wxPoint& aPosition) :
								name( aName),
								position( aPosition)
	{
	}
	/**
	 *
	 */
	void WayPoint::setName( const std::string& aName,
							bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
wxSize WayPoint::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void WayPoint::setSize( const wxSize& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void WayPoint::setPosition( const wxPoint& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	wxRegion WayPoint::getRegion() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		wxPoint originalUpperLeft( x, y);
		wxPoint originalUpperRight( x + size.x, y);
		wxPoint originalBottomLeft( x, y + size.y);
		wxPoint originalBottomRight( x + size.x, y + size.y);

		wxPoint originalPoints[] = { originalUpperRight, originalUpperLeft, originalBottomLeft, originalBottomRight };

		return wxRegion( 4, originalPoints);
	}
	/**
	 *
	 */
	bool WayPoint::intersects( const wxRegion& aRegion) const
	{
		return getRegion().Intersect( aRegion);
	}
	/**
	 *
	 */
	std::string WayPoint::asString() const
	{
		std::ostringstream os;

		os << "WayPoint " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string WayPoint::asDebugString() const
	{
		std::ostringstream os;

		os << "WayPoint:\n";
		os << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
} // namespace Model
