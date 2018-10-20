#ifndef JY901_H
#define JY901_H

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};
struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};
struct SQuater
{
	short q0;
	short q1;
	short q2;
	short q3;
};
struct SSN
{
	short sSVNum;
	short sPDOP;
	short sHDOP;
	short sVDOP;
};
class CJY901 
{
public: 

	struct Data{
		int seq = 0;
		struct STime		stcTime;
		struct SAcc 		stcAcc;
		struct SGyro 		stcGyro;
		struct SAngle 		stcAngle;
		struct SMag 		stcMag;
		struct SDStatus 	stcDStatus;
		struct SPress 		stcPress;
		struct SLonLat 		stcLonLat;
		struct SGPSV 		stcGPSV;
		struct SQuater		stcQuater;
		struct SSN 			stcSN;
	};

	Data data;
	
    CJY901 () = default;
    void CopeSerialData(const unsigned char ucData[], unsigned int ucLength);

private:
	Data data_buffer;

	void publishData();

};

#endif /* JY901_H */
