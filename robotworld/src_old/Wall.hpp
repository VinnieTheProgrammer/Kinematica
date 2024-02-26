#ifndef WALL_HPP_
#define WALL_HPP_

#include "Config.hpp"

#include "ModelObject.hpp"
#include "Point.hpp"

namespace Model
{
	class Wall;
	typedef std::shared_ptr<Wall> WallPtr;

	/**
	 *
	 */
	class Wall :  public ModelObject
	{
		public:
			/**
			 *
			 */
			Wall(	const wxPoint& aPoint1,
					const wxPoint& aPoint2);
			/**
			 *
			 */
			const wxPoint& getPoint1() const
			{
				return point1;
			}
			/**
			 *
			 */
			void setPoint1( const wxPoint& aPoint1,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			const wxPoint& getPoint2() const
			{
				return point2;
			}
			/**
			 *
			 */
			void setPoint2( const wxPoint& aPoint2,
							bool aNotifyObservers = true);
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
			wxPoint point1;
			wxPoint point2;
	};
} // namespace Model
#endif // WALL_HPP_
