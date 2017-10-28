/* 
    Simple example of sending an OSC message using oscpack.
*/

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include <iostream>
#include<string>
using namespace std;


#define ADDRESS "127.0.0.1"
#define PORT 7000

#define OUTPUT_BUFFER_SIZE 1024

int main(int argc, char* argv[])
{
    (void) argc; // suppress unused parameter warnings
    (void) argv; // suppress unused parameter warnings

    UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
	
	string oscMessages = "/joints/";
	oscMessages += "joint/";
	oscMessages += "x";
	oscMessages.c_str();
	
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(oscMessages.c_str())
		<< .005 << osc::EndMessage
		<< osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}

