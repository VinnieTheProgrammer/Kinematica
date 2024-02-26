#ifndef WIDGETS_HPP_
#define WIDGETS_HPP_

#include "Config.hpp"

#include <wx/app.h>
#include <wx/brush.h>
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/colour.h>
#include <wx/cursor.h>
#include <wx/dc.h>
#include <wx/dcclient.h>
#include <wx/defs.h>
#include <wx/dnd.h>
#include <wx/event.h>
#include <wx/font.h>
#include <wx/frame.h>
#include <wx/gbsizer.h>
#include <wx/gdicmn.h>
#include <wx/generic/textdlgg.h>
#include <wx/imaglist.h>
#include <wx/listbase.h>
#include <wx/menu.h>
#include <wx/msgdlg.h>
#include <wx/radiobox.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/splitter.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/string.h>
#include <wx/toolbar.h>
#include <wx/treebase.h>
#include <wx/validate.h>

#include <string>

/**
 *
 * @param aTitleBarMessage
 * @param aMessage
 * @param aDefaultValue
 * @param aParent
 * @param centre
 * @param aPoint
 * @return
 */
std::string GetTextFromUser(	const std::string& aTitleBarMessage,
								const std::string& aMessage,
								const std::string& aDefaultValue = "",
								wxWindow* aParent = nullptr,
								bool centre = false,
								const wxPoint& aPoint = wxDefaultPosition);

#endif /* WIDGETS_HPP_ */
