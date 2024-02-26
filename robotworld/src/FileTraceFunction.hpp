#ifndef SRC_FILETRACEFUNCTION_HPP_
#define SRC_FILETRACEFUNCTION_HPP_

#include "ITraceFunction.hpp"

#include <fstream>

namespace Base
{
	/**
	 *
	 */
	class FileTraceFunction :  public ITraceFunction
	{
		public:
			/**
			 *
			 * @param aFileStem
			 * @param aFileExtension
			 * @param timeStamped If true, the file name contains a timestamp so that the log file of multiple runs can coexist.
			 */
			explicit FileTraceFunction(const std::string& aFileStem= "trace",
									   const std::string& aFileExtension= "log",
									   bool timeStamped = false);
			/**
			 *
			 */
			virtual ~FileTraceFunction();
			/**
			 *
			 */
			virtual void trace(	const std::string& aText) override;
		private:
			std::ofstream logFile;
	}; // class StdOutTraceFunction

} /* namespace Base */

#endif // SRC_FILETRACEFUNCTION_HPP_ 
