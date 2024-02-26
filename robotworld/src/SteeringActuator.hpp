#ifndef STEERINGACTUATOR_HPP_
#define STEERINGACTUATOR_HPP_

#include "Config.hpp"

#include "AbstractActuator.hpp"

namespace Model
{
	/**
	 *
	 */
	class SteeringActuator : public AbstractActuator
	{
		public:
			/**
			 *
			 */
			SteeringActuator() = default;
			/**
			 *
			 */
			virtual ~SteeringActuator() = default;
			/**
			 *
			 */
			virtual void handleCommand( AbstractCommand& anAbstractCommand) override;
	};
} // namespace Model
#endif // STEERINGACTUATOR_HPP_
