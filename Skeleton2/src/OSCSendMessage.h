#pragma once

#include <iostream>
#include<string>
using namespace std;

class OSCSendMessage
{
public:
	OSCSendMessage();
	string jointMessage;
	int jointPosition;
	int jointPosition1;
	int jointPosition2;
	int jointPosition3;
	int jointPosition4;
	int jointPosition5;
	int jointPosition6;
	int jointPosition7;
	int jointPosition8;
	int jointPosition9;
	int jointPosition10;
	int jointPosition11;
	int jointPosition12;
	int jointPosition13;
	int jointPosition14;
	int jointPosition15;
	int jointPosition16;
	int jointPosition17;
	
	string jointName;
	string jointName1;
	string jointName2;
	string jointName3;
	string jointName4;
	string jointName5;
	string jointName6;
	string jointName7;
	string jointName8;
	string jointName9;
	string jointName10;
	string jointName11;
	string jointName12;
	string jointName13;
	string jointName14;
	string jointName15;
	string jointName16;
	string jointName17;


	void setMesseageToSendData(string  jointMessage_, float jointPosition_, float jointPosition1_, float jointPosition2_,
		float jointPosition3_, float jointPosition4_, float jointPosition5_, float jointPosition6_, float jointPosition7_, 
		float jointPosition8_, int jointPosition9_, int jointPosition10_, int jointPosition11_, int jointPosition12_,
		int jointPosition13_, int jointPosition14_, int jointPosition15_, int jointPosition16_, int jointPosition17_);
	void setMesseageToSendNames(string  jointMessage_, string jointName_, string jointName1_, string jointName2_,
		string jointName3_, string jointName4_, string jointName5_, string jointName6_, string jointName7_,
		string jointName8_, string jointName9_, string jointName10_, string jointName11_, string jointName12_,
		string jointName13_, string jointName14_, string jointName15_, string jointName16_, string jointName17_);
	string theMessage;
	string getMesseageToSendData();
	string getMesseageToSendNames();
	~OSCSendMessage();

private:
};