// File:  cppSocket.h
// Date:  9/3/2013
// Auth:  K. Loux
// Desc:  Cross-platform wrapper around socket calls.

#ifndef CPP_SOCKET_H_
#define CPP_SOCKET_H_

// pThread headers (must be first!)
#include <pthread.h>

// Standard C++ headers
#include <string>
#include <vector>
#include <iostream>

#ifdef WIN32
// Windows headers
#define NOMINMAX
#include <WinSock2.h>
#else
// *nix headers
#endif

// Forward declarations
struct sockaddr_in;

class CPPSocket
{
public:
	enum SocketType
	{
		SocketTCPServer,
		SocketTCPClient,
		SocketUDPServer,
		SocketUDPClient,
		SocketICMP
	};

#ifdef WIN32
	typedef unsigned int SocketID;
	typedef char DataType;
#else
	typedef int SocketID;
	typedef unsigned char DataType;
#endif

	CPPSocket(SocketType type = SocketTCPClient, std::ostream &outStream = std::cout);
	~CPPSocket();

	bool Create(const unsigned short &port = 0, const std::string &target = "");
	void Destroy();

	bool SetBlocking(bool blocking);

	int Receive(struct sockaddr_in *outSenderAddr = NULL);

	// NOTE:  If type == SocketTCPServer, calling method MUST aquire and release mutex when using GetLastMessage
	const DataType* GetLastMessage() { clientMessageSize = 0; return rcvBuffer; };

	bool GetLock(void);
	bool ReleaseLock(void);
	pthread_mutex_t& GetMutex(void) { return bufferMutex; };
	
	bool SetOption(const int &level, const int &option, const DataType* value, const int &size);
	bool WaitForSocket(struct timeval &timeout);

	bool Bind(const sockaddr_in &address);
	static sockaddr_in AssembleAddress(const unsigned short &port, const std::string &target = "");

	bool UDPSend(const char *addr, const short &port, const DataType* buffer, const int &bufferSize);// UDP version
	bool TCPSend(const DataType* buffer, const int &bufferSize);// TCP version

	inline bool IsICMP(void) const { return type == SocketICMP; };
	inline bool IsTCP(void) const { return type == SocketTCPServer || type == SocketTCPClient; };
	inline bool IsServer(void) const { return type == SocketTCPServer || type == SocketUDPServer; };

	unsigned int GetClientCount(void) const;

	SocketID GetFileDescriptor(void) const { return sock; };

	std::string GetErrorString(void) const { return GetLastError(); };

#ifndef WIN32
	static const int SOCKET_ERROR = -1;
#endif

	static const unsigned int maxMessageSize;

private:
	static const unsigned int maxConnections;
	static const unsigned int tcpListenTimeout;// [sec]

	const SocketType type;
	std::ostream &outStream;

	SocketID sock;
	DataType* rcvBuffer;

	bool Listen(void);
	bool Connect(const sockaddr_in &address);
	bool EnableAddressReusue(void);

	static std::vector<std::string> GetLocalIPAddress(void);
	static std::string GetBestLocalIPAddress(const std::string &destination);
	static std::string GetTypeString(SocketType type);
	static std::string GetLastError(void);

	int DoReceive(SocketID sock, struct sockaddr_in *senderAddr = NULL);
	bool TCPServerSend(const DataType* buffer, const int &bufferSize);

	// TCP server methods and members
	friend void *LaunchThread(void *pThisSocket);
	void ListenThreadEntry(void);
	void HandleClient(SocketID newSock);

	volatile bool continueListening;
	volatile int clientMessageSize;
	pthread_t listenerThread;
	pthread_mutex_t bufferMutex;
	fd_set clients;
	fd_set readSocks;
	SocketID maxSock;

	static void AddSocketToSet(SocketID socketFD, fd_set &set);
	static void RemoveSocketFromSet(SocketID socketFD, fd_set &set);
};

#endif// CPP_SOCKET_H_