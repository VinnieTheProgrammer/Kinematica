#ifndef NOTIFICATIONEVENT_HPP_
#define NOTIFICATIONEVENT_HPP_

#include "Config.hpp"

#include "Widgets.hpp"

namespace Base
{
	const wxEventType EVT_NOTIFICATIONEVENT = wxNewEventType();
	typedef void (wxEvtHandler::*NotificationEventFunction)( wxNotifyEvent&);
} // namespace Base
#endif // NOTIFICATIONEVENT_HPP_
