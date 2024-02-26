#ifndef DISTANCESTIMULI_HPP_
#define DISTANCESTIMULI_HPP_

#include "Config.hpp"

#include "DistanceStimulus.hpp"

#include <vector>

namespace Model
{
	using Stimuli = std::vector<DistanceStimulus>;

	/**
	 *
	 */
	class DistanceStimuli : public AbstractStimulus
	{
		public:
			/**
			 *
			 */
			DistanceStimuli() = default;
			/**
			 *
			 */
			explicit DistanceStimuli(const Stimuli& aStimuli) : stimuli(aStimuli)
			{
			}
			Stimuli stimuli;

			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "DistanceStimuli: " + std::to_string(stimuli.size());
			}
			/**
			 * Returns a description of the object with all data of the object usable for debugging
			 */
			virtual std::string asDebugString() const override
			{
				return asString();
			}
	};//	class PointCloudPercept
} // namespace Model




#endif /* DISTANCESTIMULI_HPP_ */
