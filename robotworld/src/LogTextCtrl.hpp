#ifndef LOGTEXTCTRL_HPP_
#define LOGTEXTCTRL_HPP_

#include "Config.hpp"

#include "Widgets.hpp"

namespace Application
{
	/**
	 *
	 */
	wxDECLARE_EVENT(ROBOTWORLD_TRACE_EVENT, wxCommandEvent);
	/**
	 * A text widget that accepts the tracing of a trace function.
	 */
	class LogTextCtrl : public wxTextCtrl
	{
		public:
			/**
			 * @name Constructors and destructor
			 */
			//@{
			/**
			 *
			 */
			explicit LogTextCtrl( 	wxWindow* aParent,
									wxWindowID aWindowID = wxID_ANY,
									long aStyle = wxTE_READONLY | wxTE_MULTILINE | wxTE_DONTWRAP,
									const std::string& anInitialText = "",
									const wxPoint& aPoint = wxDefaultPosition,
									const wxSize& aSize = wxDefaultSize);
			//@}
			/**
			 *
			 * @param aString
			 */
			virtual void log( const std::string& aString);
		protected:
			/**
			 *
			 * @param anEvent
			 */
			void OnCommandEvent( wxCommandEvent& anEvent);
		private:
	};
	// class LogTextCtrl
} // namespace Application
#endif // LOGTEXTCTRL_HPP_
