#ifndef UNICODE
#define UNICODE
#endif

#include <winsock2.h>
#include <Ws2tcpip.h>

#include <windows.h>

// Used for this example
#include <stdio.h>
#include <conio.h>

#include "SMS_UDP_Definitions.hpp"

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

int main()
{
	unsigned short port(SMS_UDP_PORT);
	char packetBuffer[SMS_UDP_MAX_PACKETSIZE];
	PacketBase packetHeader;
	memset(&packetHeader, 0, sizeof( PacketBase ));

	int iResult(0);
	WSADATA wsaData;
	SOCKET RecvSocket;

	sockaddr_in Receiver;
	Receiver.sin_family = AF_INET;
	Receiver.sin_port = htons(port);
	Receiver.sin_addr.s_addr = htonl(INADDR_ANY);

	sockaddr_in Sender;
  int SenderAddrSize(sizeof (Sender));
	 
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != NO_ERROR)
	{
      wprintf(L"WSAStartup failed with error %d\n", iResult);
      return 1;
  }

		RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (RecvSocket == INVALID_SOCKET)
	{
     wprintf(L"socket failed with error %d\n", WSAGetLastError());
      return 1;
  }

	iResult = bind(RecvSocket, (SOCKADDR *) & Receiver, sizeof (Receiver));
	if (iResult != 0)
	{
			wprintf(L" Failed to bind to socket with error %d\n", WSAGetLastError());
			return 1;
	}

	//------------------------------------------------------------------------------
	// TEST DISPLAY CODE
	//------------------------------------------------------------------------------
	unsigned int updateIndex(0);
	unsigned int indexChange(0);
	sParticipantsData pData;
	sParticipantsData pData2;
	sParticipantVehicleNamesData pVehicles;
	sParticipantVehicleNamesData pVehicles2;
	sVehicleClassNamesData pClasses;
	sGameStateData		stateData;

	memset(&pData, 0, sizeof( sParticipantsData ));
	memset(&pData2, 0, sizeof( sParticipantsData ));
	memset(&stateData, 0, sizeof( sGameStateData ));
	memset(&pVehicles, 0, sizeof( sParticipantVehicleNamesData ));
	memset(&pVehicles2, 0, sizeof( sParticipantVehicleNamesData ));
	memset(&pClasses, 0, sizeof( sVehicleClassNamesData ));

	bool participantsReceived(false);
	bool participantsReceived2(false);
	bool stateReceived(false);
	bool vehiclesReceived(false);
	bool vehiclesReceived2(false);

	printf( "ESC TO EXIT\n\n" );
	while (true)
	{
		printf( "Receiving Packets \n" );
		iResult = recvfrom(RecvSocket,  packetBuffer , SMS_UDP_MAX_PACKETSIZE, 0, (SOCKADDR *) & Sender, &SenderAddrSize);
    if (iResult == SOCKET_ERROR)
		{
        wprintf(L"recvfrom failed with error %d\n", WSAGetLastError());
    }
		else
		{
			if (iResult > 0)
			{
				memcpy(&packetHeader, packetBuffer, sizeof( PacketBase ));
				switch ( packetHeader.mPacketType )
				{
						case eParticipants :
						if (packetHeader.mPartialPacketIndex == 1)
						{
							memcpy(&pData, packetBuffer, sizeof( sParticipantsData ));
						}
						if (packetHeader.mPartialPacketIndex == 2)
						{
							memcpy(&pData2, packetBuffer, sizeof( sParticipantsData ));
							participantsReceived2 = true;
						}
						if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
						{
							participantsReceived = true;
						}

						break;
					case eGameState :
						memcpy(&stateData, packetBuffer, sizeof( sGameStateData ));
						stateReceived = true;
						break;
					case eParticipantVehicleNames :
					{
						//last packet are always the vehicle class names
						if (packetHeader.mPartialPacketIndex == packetHeader.mPartialPacketNumber)
						{
							memcpy(&pClasses, packetBuffer, sizeof( sVehicleClassNamesData ));
							vehiclesReceived= true;
						}
						else
						{
							if (packetHeader.mPartialPacketIndex == 1)
							{
								memcpy(&pVehicles, packetBuffer, sizeof( sParticipantVehicleNamesData ));
							}
							if (packetHeader.mPartialPacketIndex == 2)
							{
								memcpy(&pVehicles2, packetBuffer, sizeof( sParticipantVehicleNamesData ));
							}
						}
					}
					default: break;
				}
			}
		}
		wprintf(L" Last packet size %d, Packet count %d, packet type count %d, group index %d, total in group %d - of type \n", iResult, packetHeader.mPacketNumber,packetHeader.mCategoryPacketNumber,packetHeader.mPartialPacketIndex,packetHeader.mPartialPacketNumber,packetHeader.mPacketVersion );
		if (stateReceived)
		{
			int gameState = stateData.mGameState & 7;
			int sessionState = stateData.mGameState >> 4;

			wprintf(L" Game State %i, gameState  %i, sessionState %i \n", stateData.mGameState, gameState, sessionState );
			wprintf(L" Race Participants  \n");
			if (participantsReceived)
			{
				for (int i(0);i<PARTICIPANTS_PER_PACKET;++i)
				{
					if (pData.sName[i][0] != '\0')
					{
						wprintf(L" Name %S \n",pData.sName[i]);
					}
				}
				if (participantsReceived2)
				{
					for (int i(0);i<PARTICIPANTS_PER_PACKET;++i)
					{
						if (pData2.sName[i][0] != '\0')
						{
							wprintf(L" Name %S \n",pData2.sName[i]);
						}
					}
				}
			}
			if (vehiclesReceived)
			{
				wprintf(L"Vehicle Names\n");
				for (int i(0);i<VEHICLES_PER_PACKET;++i)
				{
					if (pVehicles.sName[i][0] != '\0')
					{
						wprintf(L"Vehicle Name %S, index %d, class %d \n",pVehicles.sName[i],pVehicles.sIndex[i], pVehicles.sClass[i]);
					}
				}
				if (vehiclesReceived2)
				{
					for (int i(0);i<VEHICLES_PER_PACKET;++i)
					{
						if (pVehicles2.sName[i][0] != '\0')
						{
							wprintf(L"Vehicle Name %S, index %d, class %d \n",pVehicles2.sName[i],pVehicles2.sIndex[i], pVehicles2.sClass[i]);
						}
					}
				}
				wprintf(L"Class Names\n");
				for (int i(0);i<CLASSES_SUPPORTED_PER_PACKET;++i)
				{
					if (pClasses.sClassName[i][0] != '\0')
					{
						wprintf(L"Class Name %S, index %d`\n",pClasses.sClassName[i],pClasses.sClassIndex[i]);
					}
				}
			}
		}

		system("cls");

		if ( _kbhit() && _getch() == 27 ) // check for escape
		{
			break;
		}
	}
	//------------------------------------------------------------------------------

	closesocket(RecvSocket);

	return 0;
}
