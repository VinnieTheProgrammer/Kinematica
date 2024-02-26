#include "RobotShape.hpp"

#include "Goal.hpp"
#include "LaserDistanceSensor.hpp"
#include "LidarSensor.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "Notifier.hpp"
#include "Robot.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include "RobotWorldCanvas.hpp"
#include "Shape2DUtils.hpp"
#include "Trace.hpp"

#include <cmath>
#include <random>

namespace View
{
	/**
	 *
	 */
	RobotShape::RobotShape( Model::RobotPtr aRobot) :
								RectangleShape( std::dynamic_pointer_cast<Model::ModelObject>(aRobot), aRobot->getPosition(), aRobot->getName()),
								robotWorldCanvas(nullptr)
	{
	}
	/**
	 *
	 */
	Model::RobotPtr RobotShape::getRobot() const
	{
		return std::dynamic_pointer_cast<Model::Robot>(getModelObject());
	}
	/**
	 *
	 */
	// cppcheck-suppress unusedFunction
	void RobotShape::setRobot( Model::RobotPtr aRobot)
	{
		setModelObject(std::dynamic_pointer_cast<Model::ModelObject>(aRobot));
	}
	/**
	 *
	 */
	void RobotShape::handleActivated()
	{
		Model::GoalPtr goal = Model::RobotWorld::getRobotWorld().getGoal( "Goal");
		if (goal)
		{
			wxPoint goalPosition = goal->getPosition();
			wxPoint robotPosition = getRobot()->getPosition();
			getRobot()->setFront( Model::BoundedVector( goalPosition, robotPosition), false);
		}
	}
	/**
	 *
	 */
	void RobotShape::handleSelection()
	{
		if (robotWorldCanvas->isShapeSelected() && robotWorldCanvas->getSelectedShape()->getObjectId() == getObjectId())
		{
			setSelected();
		} else
		{
			setSelected(false);
		}
	}
	/**
	 *
	 */
	void RobotShape::handleNotification()
	{
		setCentre( getRobot()->getPosition());
		robotWorldCanvas->handleNotification();
	}
	/**
	 *
	 */
	void RobotShape::draw( wxDC& dc)
	{
		//FUNCTRACE_DEVELOP();

		updateSizeToTitle( dc);

		drawStartPosition( dc);

		if(Application::MainApplication::getSettings().getDrawOpenSet())
		{
			drawOpenSet( dc);
		}

		drawPath( dc);
		drawLidar(dc);
		drawRobot( dc);
		//drawLaser( dc);

	}
	/**
	 *
	 */
	bool RobotShape::occupies( const wxPoint& aPoint) const
	{
		wxPoint cornerPoints[] = { getRobot()->getFrontRight(), getRobot()->getFrontLeft(), getRobot()->getBackLeft(), getRobot()->getBackRight() };
		return Utils::Shape2DUtils::isInsidePolygon( cornerPoints, 4, aPoint);
	}
	/**
	 *
	 */
	void RobotShape::setCentre( const wxPoint& aPoint)
	{
		getRobot()->setPosition( aPoint, false);
		RectangleShape::setCentre( getRobot()->getPosition());
	}
	/**
	 *
	 */
	void RobotShape::handleEndDrag()
	{
		FUNCTRACE_DEVELOP();
	}
	/**
	 *
	 */
	std::string RobotShape::asString() const
	{
		std::ostringstream os;

		os << "RobotShape " << RectangleShape::asString();

		return os.str();
	}
	/**
	 *
	 */
	std::string RobotShape::asDebugString() const
	{
		std::ostringstream os;

		os << "RobotShape:\n";
		os << RectangleShape::asDebugString() << "\n";
		if (getRobot())
		{
			os << getRobot()->asDebugString();
		}

		return os.str();
	}
	/**
	 *
	 */
	void RobotShape::updateSizeToTitle( wxDC& dc)
	{
		// The minimum size of the RectangleShape is the size of the title
		titleSize = dc.GetTextExtent( title);
		if (size.x < (titleSize.x + 2 * spacing + 2 * borderWidth))
		{
			size.x = titleSize.x + 2 * spacing + 2 * borderWidth;
		}
		if (size.y < (titleSize.y + 2 * spacing + 2 * borderWidth))
		{
			size.y = titleSize.y + 2 * spacing + 2 * borderWidth;
		}
		if (getRobot()->getSize() != size)
		{
			getRobot()->setSize( size, false);
		}
	}
	/**
	 *
	 */
	void RobotShape::drawStartPosition( wxDC& dc)
	{
		// Draw the start position
		dc.SetPen( wxPen(  "RED", borderWidth + 5, wxPENSTYLE_SOLID));
		dc.DrawCircle( getRobot()->startPosition, 3);
	}
	/**
	 *
	 */
	void RobotShape::drawOpenSet( wxDC& dc)
	{
		PathAlgorithm::OpenSet openSet = getRobot()->getOpenSet();
		if (openSet.size() != 0)
		{
			dc.SetPen( wxPen( "PALE GREEN", borderWidth, wxPENSTYLE_SOLID));
			for (const PathAlgorithm::Vertex &vertex : openSet)
			{
				dc.DrawPoint( vertex.asPoint());
			}
		}
	}
	/**
	 *
	 */
	void RobotShape::drawPath( wxDC& dc)
	{
		PathAlgorithm::Path path = getRobot()->getPath();
		if (path.size() != 0)
		{
			dc.SetPen( wxPen(  "BLACK", borderWidth, wxPENSTYLE_SOLID));
			for (const PathAlgorithm::Vertex &vertex : path)
			{
				dc.DrawPoint( vertex.asPoint());
			}
		}
	}
	/**
	 *
	 */
	void RobotShape::drawRobot( wxDC& dc)
	{
		// Draws a rectangle with the given top left corner, and with the given size.
		dc.SetBrush( *wxWHITE_BRUSH);
		if (isSelected())
		{
			dc.SetPen( wxPen( getSelectionColour(), borderWidth, wxPENSTYLE_SOLID));
		} else
		{
			dc.SetPen( wxPen( getNormalColour(), borderWidth, wxPENSTYLE_SOLID));
		}
		wxPoint cornerPoints[] = { getRobot()->getFrontRight(), getRobot()->getFrontLeft(), getRobot()->getBackLeft(), getRobot()->getBackRight() };
		dc.DrawPolygon( 4, cornerPoints);

		dc.SetPen( wxPen(  "RED", borderWidth + 2, wxPENSTYLE_SOLID));
		dc.DrawPoint( cornerPoints[1]);
		dc.SetPen( wxPen(  "GREEN", borderWidth + 2, wxPENSTYLE_SOLID));
		dc.DrawPoint( cornerPoints[0]);
		dc.SetPen( wxPen( "INDIAN RED", borderWidth + 2, wxPENSTYLE_SOLID));
		dc.DrawPoint( cornerPoints[2]);
		dc.SetPen( wxPen( "PALE GREEN", borderWidth + 2, wxPENSTYLE_SOLID));
		dc.DrawPoint( cornerPoints[3]);

		double angle = Utils::Shape2DUtils::getAngle( getRobot()->getFront());

		// Draw the nose
		dc.SetPen( wxPen(  "BLACK", 1, wxPENSTYLE_SOLID));
		dc.DrawLine( centre.x, centre.y, static_cast< int >( centre.x + std::cos( angle) * 25), static_cast< int >( centre.y + std::sin( angle) * 25));

		int textOffsetx = static_cast< int >( std::cos( -angle - 0.5 * Utils::PI) * (titleSize.x / 2) + std::sin( -angle - 0.5 * Utils::PI) * (titleSize.y / 2));
		int textOffsety = static_cast< int >( std::sin( -angle - 0.5 * Utils::PI) * (titleSize.x / 2) - std::cos( -angle - 0.5 * Utils::PI) * (titleSize.y / 2));
		dc.DrawRotatedText( title, centre.x - textOffsetx, centre.y + textOffsety, (-angle - 0.5 * Utils::PI) / Utils::PI * 180);
	}
	/**
	 *
	 */
	void RobotShape::drawLaser( wxDC& dc)
	{
		double angle = Utils::Shape2DUtils::getAngle( getRobot()->getFront());

		// Draw the laser beam
		dc.SetPen( wxPen(  "RED", 1, wxPENSTYLE_SOLID));
		dc.DrawLine( centre.x, centre.y, static_cast< int >( centre.x + std::cos( angle) * Model::laserBeamLength), static_cast< int >( centre.y + std::sin( angle) * Model::laserBeamLength));

		// Draw the radar endPoints that are actually touching the walls
		for (const Model::DistancePercept &d : getRobot()->currentRadarPointCloud)
		{
			if (d.point != wxDefaultPosition || (d.point.x != Model::noObject && d.point.y != Model::noObject))
			{
				dc.SetPen( wxPen(  "RED", borderWidth, wxPENSTYLE_SOLID));
				dc.DrawCircle( d.point, 1);
			}
		}
	}

	void RobotShape::drawLidar(	wxDC& dc)
	{

		Utils::MathUtils utils;
		std::random_device rd{};
		std::mt19937 gen{rd()};
		std::normal_distribution<> noise{0,Model::LidarSensor::getStdDev()};

		dc.SetPen( wxPen(  "BLUE", 1, wxPENSTYLE_SOLID));
		
		for(int i = 0; i < 360; i += 2) {
			int dev = noise(gen);
			std::vector< Model::WallPtr > walls = Model::RobotWorld::getRobotWorld().getWalls();
			bool lineDrawn = false;
			for (Model::WallPtr wall : walls) {
				wxPoint wallPoint1 = wall->getPoint1();
				wxPoint wallPoint2 = wall->getPoint2();
				wxPoint laserEndpoint{static_cast<int>(centre.x + std::cos(utils.toRadians((double)i)) * (Model::lidarBeamLength + dev)) ,
					static_cast<int>(centre.y + std::sin(utils.toRadians((double)i)) * (Model::lidarBeamLength + dev))};
				wxPoint interSection = Utils::Shape2DUtils::getIntersection( wallPoint1, wallPoint2, centre, laserEndpoint);
				if(interSection != wxDefaultPosition) {
					dc.DrawLine( centre.x, centre.y, interSection.x, interSection.y);
					lineDrawn = true;
				}
			}
			if(!lineDrawn) {
				dc.DrawLine( centre.x, centre.y, static_cast<int>(centre.x + std::cos(utils.toRadians((double)i)) * (Model::lidarBeamLength + dev)), static_cast<int>(centre.y + std::sin(utils.toRadians((double)i)) * (Model::lidarBeamLength + dev)));
			}
		}
		// Draw the radar endPoints that are actually touching the walls
		for (const Model::DistancePercept &d : getRobot()->currentRadarPointCloud)
		{
			if (d.point != wxDefaultPosition || (d.point.x != Model::noObject && d.point.y != Model::noObject))
			{
				dc.SetPen( wxPen(  "BLUE", borderWidth, wxPENSTYLE_SOLID));
				dc.DrawCircle( d.point, 1);
			}
		}
	}
} // namespace View
