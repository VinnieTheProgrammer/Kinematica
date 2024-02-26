#include "AbstractActuator.hpp"

#include "AbstractAgent.hpp"

namespace Model
{
	/**
	 *
	 */
	AbstractActuator::AbstractActuator( AbstractAgent& anAgent) :
					agent { &anAgent }
	{
	}
	/**
	 *
	 */
	void AbstractActuator::attachAgent( AbstractAgent& anAgent)
	{
		agent = &anAgent;
	}
	/**
	 *
	 */
	// cppcheck-suppress unusedFunction
	void AbstractActuator::detachAgent()
	{
		agent = nullptr;
	}
	/**
	 *
	 */
	std::string AbstractActuator::asString() const
	{
		return "AbstractActuator";
	}
	/**
	 *
	 */
	std::string AbstractActuator::asDebugString() const
	{
		return asString();
	}
} // namespace Model
