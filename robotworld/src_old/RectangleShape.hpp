#ifndef RECTANGLESHAPE_HPP_
#define RECTANGLESHAPE_HPP_

#include "Config.hpp"

#include "Point.hpp"
#include "Shape.hpp"
#include "Size.hpp"

#include <string>

namespace View
{

	class RectangleShape;
	typedef std::shared_ptr<RectangleShape> RectangleShapePtr;

	/**
	 *
	 */
	class RectangleShape : public Shape
	{
		public:
			/**
			 *
			 */
			explicit RectangleShape( const std::string& aTitle = "");
			/**
			 *
			 */
			explicit RectangleShape(const wxPoint& aCentrePoint,
									const std::string& aTitle = "",
									int aBorderWidth = 2,
									int aSpacing = 2);
			/**
			 *
			 */
			RectangleShape( Model::ModelObjectPtr aModelObject,
							const wxPoint& aCentrePoint,
							const std::string& aTitle = "",
							int aBorderWidth = 2,
							int aSpacing = 2);
			/**
			 *
			 */
			RectangleShape& operator=( const RectangleShape& aRectangleShape);
			/**
			 * @name Observer functions
			 */
			//@{
			/**
			 * A Notifier will call this function if this Observer will handle the notifications of that
			 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
			 */
			virtual void handleNotification() override {}
			//@}
			/**
			 * @name Pure virtual abstract Shape functions
			 */
			//@{
			/**
			 *
			 */
			virtual void draw( wxDC& dc) override;
			/**
			 *
			 * @param aPoint
			 * @return True if the point is in the shape
			 */
			virtual bool occupies( const wxPoint& aPoint) const override;
			//@}
			/**
			 *
			 * @return True if the point is on the border of the shape
			 */
			virtual bool isBorderPoint( const wxPoint& aPoint,
										int aRadius = 3) const;
			/**
			 *
			 */
			virtual wxPoint getCentre() const override;
			/**
			 *
			 */
			virtual void setCentre( const wxPoint& aPoint) override;
			/**
			 *
			 */
			std::string getTitle() const;
			/**
			 *
			 */
			virtual wxColour getNormalColour() const
			{
				return "BLACK";
			}
			/**
			 *
			 */
			virtual wxColour getSelectionColour() const
			{
				return "RED";
			}
			/**
			 *
			 */
			virtual wxColour getActivationColour() const
			{
				return "BLUE";
			}
			;
			/**
			 *
			 */
			void setTitle( const std::string& aTitle);
			/**
			 *
			 */
			virtual wxSize getSize() const;
			/**
			 *
			 */
			virtual void setSize( const wxSize& aSize);
			/**
			 *
			 */
			int getBorderWidth() const;
			/**
			 *
			 */
			void setBorderWidth( int aBorderWidth);
			/**
			 *
			 */
			int getSpacing() const;
			/**
			 *
			 */
			void setSpacing( int aSpacing);
			/**
			 *
			 */
			virtual void handleActivated() override;
			/**
			 *
			 */
			virtual void handleSelection() override;
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
			/**
			 *
			 */
			wxPoint centre;
			/**
			 *
			 */
			wxSize size;
			/**
			 *
			 */
			std::string title;
			/**
			 *
			 */
			wxSize titleSize;
			/**
			 *
			 */
			int borderWidth;
			/**
			 *
			 */
			int spacing;

	};
	//	class RectangleShape
} // namespace View
#endif // RECTANGLESHAPE_HPP_
