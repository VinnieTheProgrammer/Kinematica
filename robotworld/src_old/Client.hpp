#ifndef CLIENT_HPP_
#define CLIENT_HPP_

#include "Config.hpp"

#include "Session.hpp"

#include <string>

namespace Messaging
{
	/**
	 *
	 */
	class Client
	{
		public:
			/**
			 *
			 */
			Client( const std::string& aHostName,
					unsigned short aPort,
					ResponseHandlerPtr aResponseHandler) :
							host( aHostName),
							port( aPort),
							responseHandler( aResponseHandler)
			{
			}
			/**
			 * Dispatch the message to the client's remote server
			 *
			 * @startuml
			 * -> client: dispatchMessage(message)
			 * activate client
			 * client -->> session
			 * activate session
			 * deactivate session
			 * client -->> endpoint
			 * client -\ socket: async_connect(endpoint,(){handleConnect();})
			 * <-- client
			 * deactivate client
			 * client <- socket : handleConnect(session,error)
			 * activate socket
			 * activate client
			 * client -> session :start
			 * deactivate client
			 * deactivate socket
			 * activate session
			 * == Writing the request ==
			 * session -> session: writeMessage(message)
			 * activate session
			 * session -\ socket: async_write(socket,messageHeaderBuffer,(){handleHeaderWriten();})
			 * deactivate session
			 * activate socket
			 * client <-- session
			 * deactivate session
			 * session <- socket : handleHeaderWriten()
			 * deactivate socket
			 * activate session
			 * session -\ socket: async_write(socket,messageBodyBuffer,(){handleBodyWriten();})
			 * deactivate session
			 * activate socket
			 * session <- socket : handleBodyWriten()
			 * deactivate socket
			 * activate session
			 * session -> session : handleMessageWritten(error)
			 * activate session
			 * session -> session : handleMessageWritten()
			 * activate session
			 * == Reading the response ==
			 * session -> session: readMessage
			 * activate session
			 * session -\ socket: async_read(socket messageHeaderBuffer, (){handleHeaderRead();})
			 * deactivate session
			 * activate socket
			 * deactivate session
			 * deactivate session
			 * deactivate session
			 * session <- socket : handleHeaderRead(message,error)
			 * deactivate socket
			 * activate session
			 * session -\ socket: async_read(socket messageBodyBuffer,(){handleBodyRead();})
			 * deactivate session
			 * activate socket
			 * session <- socket : handleBodyRead()
			 * deactivate socket
			 * activate session
			 * session -> session : handleMessageRead(error)
			 * activate session
			 * session -> session : handleMessageRead()
			 * activate session
			 * == Handling the response ==
			 * session -> responseHandler: handleResponse(message)
			 * activate responseHandler
			 * session <-- responseHandler
			 * deactivate responseHandler
			 * deactivate session
			 * deactivate session
			 * destroy session
			 * @enduml
			 */
			void dispatchMessage( const Message& aMessage)
			{
				// Create the session that will handle the next outgoing connection
				ClientSession* session = new ClientSession( aMessage,
															responseHandler);

				// Build up the remote address to which we will connect
				boost::asio::ip::tcp::resolver resolver( CommunicationService::getCommunicationService().getIOContext());
				boost::asio::ip::tcp::resolver::query query( boost::asio::ip::tcp::v4(), host, std::to_string(port));
				boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve( query);
				boost::asio::ip::tcp::endpoint endpoint = *endpoint_iterator;

				// Let the session handle any outgoing messages
				session->getSocket().async_connect( endpoint, // @suppress("Method cannot be resolved") // @suppress("Invalid arguments")
													[this,session](const boost::system::error_code& error)
													{
														handleConnect(session, error);
													});
			}
		private:
			/**
			 *
			 */
			void handleConnect( ClientSession* aSession,
								const boost::system::error_code& error) const
			{
				if (!error)
				{
					aSession->start();
				} else
				{
					delete aSession;

					std::ostringstream os;
					os << __PRETTY_FUNCTION__ << ": error connecting to " << host << " at port " << port << ", reason: " << error.message();
					TRACE_DEVELOP(os.str());
				}
			}
			/**
			 *
			 */
			std::string host;
			/**
			 *
			 */
			unsigned short port;
			/**
			 *
			 */
			ResponseHandlerPtr responseHandler;
	};
} // namespace Messaging

#endif // CLIENT_HPP_
