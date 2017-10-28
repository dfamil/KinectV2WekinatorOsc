#include "OSCSendMessage.h"
#include "app.h"


#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#define ADDRESS "192.168.2.9"
#define PORT 6448


#define OUTPUT_BUFFER_SIZE 1024
UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, PORT));
char buffer[OUTPUT_BUFFER_SIZE];

using namespace std;
using namespace osc;
OSCSendMessage  ::OSCSendMessage()
{
}
void OSCSendMessage::setMesseageToSendData(string  jointMessage_, float jointPosition_, float jointPosition1_, float jointPosition2_
	, float jointPosition3_, float jointPosition4_, float jointPosition5_, float jointPosition6_, float jointPosition7_, float jointPosition8_, 
	int jointPosition9_, int jointPosition10_, int jointPosition11_, int jointPosition12_, int jointPosition13_, int jointPosition14_, 
	int jointPosition15_, int jointPosition16_, int jointPosition17_) {
	jointMessage= jointMessage_;
	jointPosition = jointPosition_;
	jointPosition1 = jointPosition1_;
	jointPosition2 = jointPosition2_;
	jointPosition3 = jointPosition3_;
	jointPosition4 = jointPosition4_;
	jointPosition5 = jointPosition5_;
	jointPosition6 = jointPosition6_;
	jointPosition7 = jointPosition7_;
	jointPosition8 = jointPosition8_;
	jointPosition9 = jointPosition9_;
	jointPosition10 = jointPosition10_;
	jointPosition11 = jointPosition11_;
	jointPosition12 = jointPosition12_;
	jointPosition13 = jointPosition13_;
	jointPosition14 = jointPosition14_;
	jointPosition15 = jointPosition15_;
	jointPosition16 = jointPosition16_;
	jointPosition17 = jointPosition17_;
	

	osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(jointMessage.c_str())
		<< float(jointPosition) 
		<< float(jointPosition1)
		<< float(jointPosition2)
		<< float(jointPosition3)
		<< float(jointPosition4)
		<< float(jointPosition5)
		<< float(jointPosition6)
		<< float(jointPosition7)
		<< float(jointPosition8)
		<< int(jointPosition9)
		<< int(jointPosition10)
		<< int(jointPosition11)
		<< int(jointPosition12)
		<< int(jointPosition13)
		<< int(jointPosition14)
		<< int(jointPosition15)
		<< int(jointPosition16)
		<< int(jointPosition17)
	 << osc::EndMessage
		<< osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}

void OSCSendMessage::setMesseageToSendNames(string  jointMessage_, string jointName_, string jointName1_, string jointName2_
	, string jointName3_, string jointName4_, string jointName5_, string jointName6_, string jointName7_, string jointName8_,
	string jointName9_, string jointName10_, string jointName11_, string jointName12_, string jointName13_, string jointName14_,
	string jointName15_, string jointName16_, string jointName17_) {
	jointMessage = jointMessage_;
	jointName = jointName_;
	jointName1 = jointName1_;
	jointName2 = jointName2_;
	jointName3 = jointName3_;
	jointName4 = jointName4_;
	jointName5 = jointName5_;
	jointName6 = jointName6_;
	jointName7 = jointName7_;
	jointName8 = jointName8_;
	jointName9 = jointName9_;
	jointName10 = jointName10_;
	jointName11 = jointName11_;
	jointName12 = jointName12_;
	jointName13 = jointName13_;
	jointName14 = jointName14_;
	jointName15 = jointName15_;
	jointName16 = jointName16_;
	jointName17 = jointName17_;


	osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
	p << osc::BeginBundleImmediate
		<< osc::BeginMessage(jointMessage.c_str())
		<< jointName.c_str()
		<< jointName1.c_str()
		<< jointName2.c_str()
		<< jointName3.c_str()
		<< jointName4.c_str()
		<< jointName5.c_str()
		<< jointName6.c_str()
		<< jointName7.c_str()
		<< jointName8.c_str()
		<< jointName9.c_str()
		<< jointName10.c_str()
		<< jointName11.c_str()
		<< jointName12.c_str()
		<< jointName13.c_str()
		<< jointName14.c_str()
		<< jointName15.c_str()
		<< jointName16.c_str()
		<< jointName17.c_str()
		<< osc::EndMessage
		<< osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}

string OSCSendMessage::getMesseageToSendData() {
	return theMessage;
}
string OSCSendMessage::getMesseageToSendNames() {
	return theMessage;
}
OSCSendMessage::~OSCSendMessage()
{
}
