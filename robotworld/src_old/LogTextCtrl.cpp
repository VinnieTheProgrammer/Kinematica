#include "LogTextCtrl.hpp"

namespace Application
{
	/**
	 *
	 */
	wxDEFINE_EVENT(ROBOTWORLD_TRACE_EVENT, wxCommandEvent);
	/**
	 *
	 */
	LogTextCtrl::LogTextCtrl( 	wxWindow* aParent,
								wxWindowID aWindowID /*  = wxID_ANY */,
								long aStyle /* = wxTE_READONLY | wxTE_MULTILINE | wxTE_DONTWRAP */,
								const std::string& anInitialText /* = "" */,
								const wxPoint& aPoint /* = wxDefaultPosition */,
								const wxSize& aSize /* = DefaultSize */) :
								wxTextCtrl( aParent, aWindowID, anInitialText, aPoint, aSize, aStyle)
	{
		Bind(ROBOTWORLD_TRACE_EVENT,
			 [this](wxCommandEvent& anEvent){ this->OnCommandEvent(anEvent);});
	}
	/**
	 *
	 */
	void LogTextCtrl::log( const std::string& aString)
	{
		AppendText( aString + "\n");
	}
	/**
	 *
	 */
	void LogTextCtrl::OnCommandEvent( wxCommandEvent& anEvent)
	{
		log( anEvent.GetString().ToStdString());
	}
} // namespace Application
