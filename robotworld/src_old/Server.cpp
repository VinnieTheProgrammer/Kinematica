#include "Server.hpp"

namespace Messaging
{
	/**
	 *
	 */
	Server::Server(unsigned short aPort, RequestHandlerPtr aRequestHandler) :
					port(aPort),
					acceptor(CommunicationService::getCommunicationService().getIOContext()),
					requestHandler(aRequestHandler),
					timer( Messaging::CommunicationService::getCommunicationService().getIOContext())
	{
	}
} /* namespace Messaging */
