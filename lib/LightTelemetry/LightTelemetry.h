#ifndef	LIGHTTELEMETRY_H
#define LIGHTTELEMETRY_H

#include <stdint.h>
#include <stddef.h>

#define LIGHTTELEMETRY_START1 0x24 //$ HEADER '$'
#define LIGHTTELEMETRY_START2 0x54 //T  HEADER 'T'
#define LIGHTTELEMETRY_GFRAME 0x47 //G GPS + Baro altitude data ( Lat, Lon, Speed, Alt, Sats, Sat fix)
#define LIGHTTELEMETRY_AFRAME 0x41 //A Attitude data ( Roll,Pitch, Heading )
#define LIGHTTELEMETRY_SFRAME 0x53 //S Sensors/Status data ( VBat, Consumed current, Rssi, Airspeed, Arm status, Failsafe status, Flight mode )
#define LIGHTTELEMETRY_OFRAME 0x4F //O  Origin data (home lon, home lat, homefix)
#define LIGHTTELEMETRY_NFRAME 0x4E //N  NAVIGATION data
#define LIGHTTELEMETRY_XFRAME 0x58 //X  EXTRA FRAME

#define LIGHTTELEMETRY_GFRAMELENGTH 18
#define LIGHTTELEMETRY_AFRAMELENGTH 10
#define LIGHTTELEMETRY_SFRAMELENGTH 11
#define LIGHTTELEMETRY_OFRAMELENGTH 18
#define LIGHTTELEMETRY_NFRAMELENGTH 10
#define LIGHTTELEMETRY_XFRAMELENGTH 10


class LTM_reader {

public:
//Global variables
static int32_t       LTM_pkt_ko;   //wrong LTM packet counter
static int32_t       LTM_pkt_ok;   //good LTM packet counter
static int32_t       uav_homelat; //home latitude
static int32_t       uav_homelon; //home longitude
static int32_t       uav_homealt; //home altitude
static int32_t       ground_course; //course over ground
static int32_t       ground_distance; //distance bettween two consecutive GPS points in actual course (unused) // TODO use this to calc SPEED
static uint8_t       uav_osd_on; // allways 1
static uint32_t      home_distance; // distante to home
static int32_t       home_heading; // heading to home
static uint8_t       uav_homefixstatus; // 1=homefix ok
static uint8_t       uav_gpsmode;
static uint8_t       uav_navmode;
static uint8_t       uav_navaction;
static uint8_t       uav_WPnumber; // Waypoint number
static uint8_t       ltm_naverror;
static uint8_t       ltm_flags;
static uint16_t      uav_HDOP; // GPS HDOP
static uint8_t       uav_HWstatus; // hardware error
static uint8_t       uav_spare1;
static uint8_t       uav_spare2;
static uint8_t       ltm_spare3;
static int32_t      uav_lat;                    // latitude
static int32_t      uav_lon;                    // longitude
static float        lonScaleDown;             // longitude scaling
static uint8_t      uav_satellites_visible;     // number of satellites
static uint8_t      uav_fix_type;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static int32_t      uav_alt;                    // altitude (dm)
static int32_t      rel_alt;                    // relative altitude to home
static uint16_t     uav_groundspeed;            // ground speed in km/h
static uint8_t      uav_groundspeedms;          // ground speed in m/s
static int16_t      uav_pitch;                  // attitude pitch
static int16_t      uav_roll;                   // attitude roll
static int16_t      uav_heading;                // attitude heading
static int16_t      uav_gpsheading;             // gps heading
static uint16_t     uav_bat;                    // battery voltage (mv)
static uint16_t     uav_amp;                    // consumed mah.
static uint16_t     uav_current;                // actual current
static uint8_t      uav_rssi;                   // radio RSSI (%)
static uint8_t      uav_linkquality;            // radio link quality
static uint8_t      uav_airspeed;               // Airspeed sensor (m/s)
static uint8_t      ltm_armfsmode;
static uint8_t      uav_arm;                    // 0: disarmed, 1: armed
static uint8_t      uav_failsafe;               // 0: normal,   1: failsafe
static uint8_t      uav_flightmode;            // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
// 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
// 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown


//long lastpacketreceived;

static bool gps_fix;
//boolean btholdstate  = false;
//boolean telemetry_ok = false;
//boolean home_pos     = false;
//boolean home_bear    = false;

private:
static uint8_t LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH - 4];
static uint8_t LTMreceiverIndex;
static uint8_t LTMcmd;
static uint8_t LTMrcvChecksum;
static uint8_t LTMreadIndex;
static uint8_t LTMframelength;

public:
static void read(uint8_t* buff, size_t size);

private:
LTM_reader(){};
static void check();
static uint8_t ltmread_u8()  {
  return LTMserialBuffer[LTMreadIndex++];
}

static uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8() << 8;
  return t;
}

static uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16() << 16;
  return t;
}

};

#endif	//LIGHTTELEMETRY_H