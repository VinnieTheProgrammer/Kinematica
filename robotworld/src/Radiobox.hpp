#ifndef RADIOBOX_HPP_
#define RADIOBOX_HPP_

#include "Config.hpp"

#include "Widgets.hpp"

#include <functional>
#include <string>

namespace Application
{
	/**
	 * Utility function that creates a RadioBox with aLabel that binds the given ActionFunction with the
	 * wxEVT_RADIOBOX of the radiobox.
	 *
	 * @param aParent
	 * @param aLabel
	 * @param anActionFunction
	 * @return the created RadioBox
	 */
	template< typename ActionFunction >
	wxRadioBox* makeRadiobox(	wxWindow* aParent,
								const std::string& aLabel,
								const ActionFunction& anActionFunction)
	{
		wxRadioBox* radiobox= new wxRadioBox( aParent, wxID_ANY, aLabel);
		radiobox->Bind(wxEVT_RADIOBOX, anActionFunction);
		return radiobox;
	}
	/**
	 * Utility function that creates a RadioBox with aListOfChoices that binds the given ActionFunction with the
	 * wxEVT_RADIOBOX of the radiobox.
	 *
	 * @tparam ActionFunction
	 * @param aParent
	 * @param aNumberOfChoices
	 * @param aListOfChoices
	 * @param anActionFunction
	 * @param aLabel
	 * @param aStyle
	 * @param aNumberOfRowsOrColumns
	 * @return
	 */
	template< typename ActionFunction, std::size_t N >
	wxRadioBox* makeRadiobox( 	wxWindow* aParent,
								const std::array< std::string, N >& aListOfChoices,
								const ActionFunction& anActionFunction,
								const std::string& aLabel = "",
								long aStyle = wxRA_SPECIFY_ROWS,
								int aNumberOfRowsOrColumns = 0)
	{
		wxArrayString listOfChoices;
		for ( auto& choice : aListOfChoices)
		{
			// cppcheck-suppress useStlAlgorithm
			listOfChoices.push_back(choice);
		}
		wxRadioBox* radiobox = new wxRadioBox(	aParent,
												wxID_ANY,
												aLabel,
												wxDefaultPosition,
												wxDefaultSize,
												listOfChoices,
												aNumberOfRowsOrColumns,
												aStyle);
		radiobox->Bind(wxEVT_RADIOBOX, anActionFunction);
		return radiobox;
	}

} // namespace Application




#endif /* RADIOBOX_HPP_ */
