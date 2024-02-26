#ifndef DISTANCEPERCEPTS_HPP_
#define DISTANCEPERCEPTS_HPP_

#include "Config.hpp"

#include "DistancePercept.hpp"
#include "DistanceStimuli.hpp"

#include <vector>

namespace Model
{
	using PointCloud = std::vector<DistancePercept>;

	/**
	 *
	 */
	class DistancePercepts : public AbstractPercept
	{
		public:
			/**
			 *
			 */
			DistancePercepts() = default;
			/**
			 *
			 */
			explicit DistancePercepts(const PointCloud& aPointCloud) : pointCloud(aPointCloud)
			{
			}
			PointCloud pointCloud;
			/**
			 * @name Debug functions
			 */
			//@{
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const override
			{
				return "DistancePercepts: " + std::to_string(pointCloud.size());
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

#endif /* DISTANCEPERCEPTS_HPP_ */
