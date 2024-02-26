#ifndef POINT_HPP_
#define POINT_HPP_

#include "Config.hpp"

#include <wx/gdicmn.h>

#include <iostream>

inline std::ostream& operator<<(std::ostream& os, const wxPoint& aPoint)
{
	return os << "(" << aPoint.x << "," << aPoint.y << ")";
}

#endif // POINT_HPP_
