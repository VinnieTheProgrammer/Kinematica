#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

#include "Config.hpp"

#include "ModelObject.hpp"
#include "Point.hpp"
#include "Region.hpp"

namespace Model
{
	class WayPoint;
	typedef std::shared_ptr<WayPoint> WayPointPtr;

	/**
	 *
	 */
	class WayPoint : 	public ModelObject
	{
		public:
			explicit WayPoint( const std::string& aName);
			/**
			 *
			 */
			WayPoint(	const std::string& aName,
						const wxPoint& aPosition);
			/**
			 *
			 */
			std::string getName() const
			{
				return name;
			}
			/**
			 *
			 */
			void setName( const std::string& aName,
						  bool aNotifyObservers = true);
			/**
			 *
			 */
			wxSize getSize() const;
			/**
			 *
			 */
			void setSize(	const wxSize& aSize,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			wxPoint getPosition() const
			{
				return position;
			}
			/**
			 *
			 */
			void setPosition(	const wxPoint& aPosition,
								bool aNotifyObservers = true);
			/**
			 *
			 */
			wxRegion getRegion() const;
			/**
			 *
			 */
			bool intersects( const wxRegion& aRegion) const;
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
		private:
			std::string name;
		wxSize size;
			wxPoint position;
	};
} // namespace Model
#endif // WAYPOINT_HPP_
