#ifndef SHAPE2DUTILS_HPP_
#define SHAPE2DUTILS_HPP_

#include "Config.hpp"

#include "BoundedVector.hpp"
#include "MathUtils.hpp"
#include "Point.hpp"
#include "Size.hpp"

#include <string>

namespace Utils
{
	/**
	 *
	 */
	class Shape2DUtils
	{
		public:
			/**
			 *
			 * @param aVector
			 * @return the counterclockwise angle in radians between the positive X axis and the given vector.
			 */
			static double getAngle( const Model::BoundedVector& aVector);
			/**
			 *
			 * @param aStartpoint
			 * @param anEndPoint
			 * @return the counterclockwise angle in radians between the positive X axis and the line between the points
			 */
			static double getAngle( const wxPoint& aStartpoint,
									const wxPoint& anEndPoint);
			/**
			 *
			 * @param aStartLine1
			 * @param aEndLine1
			 * @param aStartLine2
			 * @param anEndLine2
			 * @return true if the lines intersect, false otherwise
			 */
			static bool intersect(	const wxPoint& aStartLine1,
									const wxPoint& anEndLine1,
									const wxPoint& aStartLine2,
									const wxPoint& anEndLine2);
			/**
			 *
			 * @param aStartLine1
			 * @param aEndLine1
			 * @param aStartLine2
			 * @param anEndLine2
			 * @return a valid wxPoint it the ;lines intersect, wxDefaultPosition otherwise
			 */
			static wxPoint getIntersection(	const wxPoint& aStartLine1,
											const wxPoint& anEndLine1,
											const wxPoint& aStartLine2,
											const wxPoint& anEndLine2);
			/**
			 *
			 * @param aPoint1
			 * @param aPoint2
			 * @return the Cartesian plane distance
			 */
			static double distance(const wxPoint& aPoint1,
								   const wxPoint& aPoint2);
			/**
			 *
			 * @param aPolygon The array of points
			 * @param numberOfPoints The number of points in the array
			 * @param p The point for which to test
			 * @return True if the point is inside the polygon, false otherwise
			 */
			static bool isInsidePolygon( 	const wxPoint* aPolygon,
											int aNumberOfPoints,
											const wxPoint& aPoint);
			/**
			 *
			 * @param aStartpoint
			 * @param anEndPoint
			 * @param aPoint
			 * @param aRadius The number of pixels we can be wrong
			 * @return True if aPoint is within aRadius of the given line
			 */
			static bool isOnLine(	const wxPoint& aStartPoint,
									const wxPoint& anEndPoint,
									const wxPoint& aPoint,
									int aRadius = 6);

			/**
			 *
			 * @param aPolygon The array of points that form a path
			 * @param numberOfPoints The number of points in the array
			 * @param aPoint
			 * @param aRadius The number of pixels we can be wrong
			 * @param aClosedShape True if the path is a closed, false otherwise
			 * @return True if aPoint is within aRadius of the given path lines
			 */
			static bool isOnLine( 	const wxPoint* aPolygon,
									int aNumberOfPoints,
									const wxPoint& aPoint,
									int aRadius = 6,
									bool aClosedShape = true);

			/**
			 * The rotation will be done around (0.0)
			 *
			 * @param aPoint
			 * @param anAngle
			 * @return The rotated point
			 */
			static wxPoint rotate(	const wxPoint& aPoint,
									double anAngle);
			/**
			 * Enum to find the point of a rectangle
			 */
			enum CompassPoint
			{
				CP_NORTH,
				CP_NORTHEAST,
				CP_EAST,
				CP_SOUTHEAST,
				CP_SOUTH,
				CP_SOUTHWEST,
				CP_WEST,
				CP_NORTHWEST
			};
			/**
			 *
			 * @param aPoint A wxPoint on the border of the shape
			 * @return
			 */
			static CompassPoint getCompassPoint(	const wxPoint& aPoint,
													const wxSize& aSize,
													const wxPoint& aBorderPoint,
													int aRadius = 6);
			/**
			 *
			 * @param aBorderPoint
			 * @param aCompassPoint
			 * @param aRadius
			 * @return
			 */
			static bool isCompassPoint( const wxPoint& aPoint,
										const wxSize& aSize,
										const wxPoint& aBorderPoint,
										CompassPoint aCompassPoint,
										int aRadius = 6);

			/**
			 *
			 */
			static std::string asString( const wxPoint& aPoint);
			/**
			 *
			 */
			static std::string asString( const wxSize& aSize);

		protected:
			static double rotateX(	const wxPoint& aPoint,
									double anAngle);
			static double rotateY(	const wxPoint& aPoint,
									double anAngle);
			static double rotateX( 	int anX,
									int anY,
									double anAngle);
			static double rotateY( 	int anX,
									int anY,
									double anAngle);

	};
	//	class Shape2DUtils
} // namespace Utils

#endif // SHAPE2DUTILS_HPP_
