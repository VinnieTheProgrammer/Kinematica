#ifndef SERVER_HPP_
#define SERVER_HPP_

#include "Config.hpp"

#include "Session.hpp"
#include "CommunicationService.hpp"

#include <atomic>
#include <iostream>
#include <memory>

namespace Messaging
{
	/*
	 *
	 */
	class Server : public std::enable_shared_from_this< Server >
	{
		public:
			/**
			 *
			 */
			Server(	unsigned short aPort,
					RequestHandlerPtr aRequestHandler);
			/**
			 *
			 */
			unsigned short getPort() const
			{
				return port;
			}
			/**
			 *
			 */
			void startHandlingRequests()
			{
				boost::asio::ip::tcp::endpoint ep(boost::asio::ip::tcp::v4(), port);
				acceptor.open(ep.protocol());
				acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
				acceptor.bind(ep);
				acceptor.listen();

				stopAccepting.store(false);
				handleAccept( nullptr, boost::system::error_code());
			}
			/**
			 *
			 */
			void stopHandlingRequests()
			{
				stopAccepting.store(true);

				timer.expires_from_now(boost::posix_time::seconds(1));
				timer.async_wait([this](const boost::system::error_code& UNUSEDPARAM(e)) // @suppress("Method cannot be resolved")
								 {
									boost::asio::post(	[this]() // @suppress("Invalid arguments")
														{
															acceptor.cancel();
														});
								 });
			}
		private:
			/**
			 *	Handle any incoming connections
			 *
			 * @startuml
			 * -> server: handleAccept
			 * activate server
			 * server -->> session
			 * activate session
			 * deactivate session
			 * server -\ acceptor : asyn_accept(socket,(){handleAccept();})
			 * server -> session : start
			 * deactivate server
			 * activate session
			 * == Reading the request ==
			 * session -\ session: readMessage
			 * activate session
			 * session -\ socket: async_read(socket, messageHeaderBuffer,(){handleHeaderRead();})
			 * deactivate session
			 * activate socket
			 * server <-- session
			 * deactivate session
			 * deactivate session
			 * deactivate session
			 * session <- socket : handleHeaderRead()
			 * deactivate socket
			 * activate session
			 * session -\ socket: async_read(socket,messageBodyBuffer,(){handleBodyRead();})
			 * deactivate session
			 * activate socket
			 * session <- socket : handleBodyRead()
			 * deactivate socket
			 * activate session
			 * session -> session : handleMessageRead(error)
			 * activate session
			 * session -> session : handleMessageRead()
			 * activate session
			 * == Handling the request ==
			 * session -> requestHandler : handleRequest(message)
			 * activate requestHandler
			 * session <-- requestHandler
			 * deactivate requestHandler
			 * == Writing the response ==
			 * session -> session: writeMessage(message)
			 * activate session
			 * session -\ socket: async_write(socket, messageHeaderBuffer,(){handleHeaderWriten();})
			 * deactivate session
			 * activate socket
			 * deactivate session
			 * deactivate session
			 * deactivate session
			 * session <- socket : handleHeaderWriten()
			 * deactivate socket
			 * activate session
			 * session -\ socket: async_write(socket messageBodyBuffer,(){handleBodyWriten();})
			 * deactivate session
			 * activate socket
			 * session <- socket : handleBodyWriten()
			 * deactivate socket
			 * activate session
			 * session -> session : handleMessageWriten(error)
			 * activate session
			 * session -> session : handleMessageWritten()
			 * activate session
			 * deactivate session
			 * deactivate session
			 * destroy session
			 * @enduml
			 */
			void handleAccept( 	ServerSession* aSession,
								const boost::system::error_code& error)
			{
				if (!error)
				{
					if(!stopAccepting.load())
					{
						// Create the session that will handle the next incoming connection
						ServerSession* session = new ServerSession( requestHandler);
						// Let the acceptor wait for any new incoming connections
						// and let it call server::handle_accept on the happy occasion
						acceptor.async_accept(	session->getSocket(), // @suppress("Method cannot be resolved")
												[this, session](const boost::system::error_code& error)
												{
							handleAccept(session,error);
												});

						// If there is a session, start it up....
						if (aSession)
						{
							aSession->start();
						}
					}else
					{
						TRACE_DEVELOP("Server does not accept any sessions anymore");
					}
				} else
				{
					if (aSession)
					{
						delete aSession;
					}
					if(!stopAccepting && error != boost::asio::error::basic_errors::operation_aborted)
					{
						std::ostringstream os;
						os << "************ " << __PRETTY_FUNCTION__ << ": " << error.message() << ", stopAccepting = " << stopAccepting;
						TRACE_DEVELOP(os.str());
						throw std::runtime_error( os.str());
					}
				}
			}
		private:
			/**
			 *
			 */
			unsigned short port;
			/**
			 * Provides the ability to accept new connections
			 */
			boost::asio::ip::tcp::acceptor acceptor;
			/**
			 *
			 */
			std::atomic<bool> stopAccepting = false;
			/**
			 *
			 */
			RequestHandlerPtr requestHandler;
			/**
			 *
			 */
			boost::asio::deadline_timer timer;
	};
	/**
	 *
	 */
	typedef std::shared_ptr< Server > ServerPtr;
} /* namespace Messaging */

#endif // SERVER_HPP_ 
