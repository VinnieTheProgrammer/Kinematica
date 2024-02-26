#ifndef NOTIFICATIONFUNCTIONTYPE_HPP_
#define NOTIFICATIONFUNCTIONTYPE_HPP_

#include "Config.hpp"

#include "Widgets.hpp"

#include <functional>

namespace Base
{
	template< typename NotificationFunction >
	struct NotificationFunctionType
	{
			static void call( 	const NotificationFunction& aNotificationFunction,
								wxNotifyEvent&)
			{
				aNotificationFunction();
			}
	};

	template< >
	struct NotificationFunctionType< std::function< void( wxNotifyEvent&) > >
	{
			static void call( 	const std::function< void( wxNotifyEvent&) >& aNotificationFunction,
								wxNotifyEvent& event)
			{
				aNotificationFunction( event);
			}
	};
	/**
	 * Dummy function to allow for usage of not-yet implemented
	 * std::function<void (wxCommandEvent&)> functions
	 * @param
	 */
	void Ooops( wxNotifyEvent&);
} //namespace Base
#endif /* NOTIFICATIONFUNCTIONTYPE_HPP_ */
