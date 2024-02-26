#include "Widgets.hpp"

/**
 *
 */
std::string GetTextFromUser(	const std::string& aTitleBarMessage,
								const std::string& aMessage,
								const std::string& aDefaultValue /* = "" */,
								wxWindow* aParent /* = nullptr */,
								bool UNUSEDPARAM(centre) /* = false */,
								const wxPoint& aPoint /* = wxDefaultPosition */)
{
	std::string text( wxGetTextFromUser( aTitleBarMessage, aMessage, aDefaultValue, aParent, aPoint.x, aPoint.y));
	return text;
}

