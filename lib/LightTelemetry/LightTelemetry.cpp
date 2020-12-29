/* #################################################################################################################
   LightTelemetry protocol (LTM)

   Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds).

   Protocol details: 3 different frames, little endian.
     G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
      0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0
       $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
     A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
       0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
        $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
     S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
       0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0
        $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
   ################################################################################################################# */

#include "LightTelemetry.h"
#include <math.h>
#include <string.h>



//Global variables
int32_t      LTM_reader::LTM_pkt_ko;   //wrong LTM packet counter
int32_t      LTM_reader::LTM_pkt_ok;   //good LTM packet counter
int32_t      LTM_reader::uav_homelat; //home latitude
int32_t      LTM_reader::uav_homelon; //home longitude
int32_t      LTM_reader::uav_homealt; //home altitude
int32_t      LTM_reader::ground_course; //course over ground
int32_t      LTM_reader::ground_distance; //distance bettween two consecutive GPS points in actual course (unused) // TODO use this to calc SPEED
uint8_t      LTM_reader::uav_osd_on; // allways 1
uint32_t     LTM_reader::home_distance; // distante to home
int32_t      LTM_reader::home_heading; // heading to home
uint8_t      LTM_reader::uav_homefixstatus; // 1=homefix ok
uint8_t      LTM_reader::uav_gpsmode;
uint8_t      LTM_reader::uav_navmode;
uint8_t      LTM_reader::uav_navaction;
uint8_t      LTM_reader::uav_WPnumber; // Waypoint number
uint8_t      LTM_reader::ltm_naverror;
uint8_t      LTM_reader::ltm_flags;
uint16_t     LTM_reader::uav_HDOP; // GPS HDOP
uint8_t      LTM_reader::uav_HWstatus; // hardware error
uint8_t      LTM_reader::uav_spare1;
uint8_t      LTM_reader::uav_spare2;
uint8_t      LTM_reader::ltm_spare3;
int32_t      LTM_reader::uav_lat;                    // latitude
int32_t      LTM_reader::uav_lon;                    // longitude
float        LTM_reader::lonScaleDown;             // longitude scaling
uint8_t      LTM_reader::uav_satellites_visible;     // number of satellites
uint8_t      LTM_reader::uav_fix_type;               // GPS lock 0-1=no fix, 2=2D, 3=3D
int32_t      LTM_reader::uav_alt;                    // altitude (dm)
int32_t      LTM_reader::rel_alt;                    // relative altitude to home
uint16_t     LTM_reader::uav_groundspeed;            // ground speed in km/h
uint8_t      LTM_reader::uav_groundspeedms;          // ground speed in m/s
int16_t      LTM_reader::uav_pitch;                  // attitude pitch
int16_t      LTM_reader::uav_roll;                   // attitude roll
int16_t      LTM_reader::uav_heading;                // attitude heading
int16_t      LTM_reader::uav_gpsheading;             // gps heading
uint16_t     LTM_reader::uav_bat;                    // battery voltage (mv)
uint16_t     LTM_reader::uav_amp;                    // consumed mah.
uint16_t     LTM_reader::uav_current;                // actual current
uint8_t      LTM_reader::uav_rssi;                   // radio RSSI (%)
uint8_t      LTM_reader::uav_linkquality;            // radio link quality
uint8_t      LTM_reader::uav_airspeed;               // Airspeed sensor (m/s)
uint8_t      LTM_reader::ltm_armfsmode;
uint8_t      LTM_reader::uav_arm;                    // 0: disarmed, 1: armed
uint8_t      LTM_reader::uav_failsafe;               // 0: normal,   1: failsafe
uint8_t      LTM_reader::uav_flightmode = 19;            // Flight mode(0-19): 0: Manual, 1: Rate, 2: Attitude/Angle, 3: Horizon, 4: Acro, 5: Stabilized1, 6: Stabilized2, 7: Stabilized3,
// 8: Altitude Hold, 9: Loiter/GPS Hold, 10: Auto/Waypoints, 11: Heading Hold / headFree, 12: Circle, 13: RTH, 14: FollowMe, 15: LAND,
// 16:FlybyWireA, 17: FlybywireB, 18: Cruise, 19: Unknown


//long lastpacketreceived;

bool LTM_reader::gps_fix;
//boolean btholdstate  = false;
//boolean telemetry_ok = false;
//boolean home_pos     = false;
//boolean home_bear    = false;

uint8_t LTM_reader::LTMserialBuffer[LIGHTTELEMETRY_GFRAMELENGTH - 4];
uint8_t LTM_reader::LTMreceiverIndex;
uint8_t LTM_reader::LTMcmd;
uint8_t LTM_reader::LTMrcvChecksum;
uint8_t LTM_reader::LTMreadIndex;
uint8_t LTM_reader::LTMframelength;


void LTM_reader::read(uint8_t* buff, size_t size) {
  
  uint8_t c;

  static enum _serial_state {
    IDLE,
    HEADER_START1,
    HEADER_START2,
    HEADER_MSGTYPE,
    HEADER_DATA
  }
  c_state = IDLE;

  for(int i=0; i<size; i++) {
    c = buff[i];

    if (c_state == IDLE) {
      c_state = (c == '$') ? HEADER_START1 : IDLE;
      
    }
    else if (c_state == HEADER_START1) {
      c_state = (c == 'T') ? HEADER_START2 : IDLE;
      
    }
    else if (c_state == HEADER_START2) {
      switch (c) {
        case 'G':
          LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          break;
        case 'A':
          LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'S':
          LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'O':
          LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'N':
          LTMframelength = LIGHTTELEMETRY_NFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          
          break;
        case 'X':
          LTMframelength = LIGHTTELEMETRY_XFRAMELENGTH;
          c_state = HEADER_MSGTYPE;
          ;
          break;
        default:
          c_state = IDLE;
      }
      LTMcmd = c;
      LTMreceiverIndex = 0;
    }
    else if (c_state == HEADER_MSGTYPE) {
      if (LTMreceiverIndex == 0) {
        LTMrcvChecksum = c;
      }
      else {
        LTMrcvChecksum ^= c;
      }
      if (LTMreceiverIndex == LTMframelength - 4) { // received checksum byte
        if (LTMrcvChecksum == 0) {
          //telemetry_ok = true;
          LTM_pkt_ok++;   //increase packet ok counter
          //lastpacketreceived = millis();
          check();
          c_state = IDLE;
        }
        else {                                                   // wrong checksum, drop packet
          LTM_pkt_ko++;       //increase packet dropped counter
          c_state = IDLE;
        }
      }
      else LTMserialBuffer[LTMreceiverIndex++] = c;
    }
  }
}

// --------------------------------------------------------------------------------------
// Decoded received commands
void LTM_reader::check() {
  
  LTMreadIndex = 0;

  if (LTMcmd == LIGHTTELEMETRY_GFRAME)
  {
    uav_lat = (int32_t)ltmread_u32();
    uav_lon = (int32_t)ltmread_u32();
    uav_groundspeedms = ltmread_u8();
    uav_groundspeed = (uint16_t) round((float)(uav_groundspeedms * 3.6f)); // convert to kmh
    uav_alt = (int32_t)ltmread_u32()/100;//convert to m
    uint8_t ltm_satsfix = ltmread_u8();
    uav_satellites_visible         = (ltm_satsfix >> 2) & 0xFF;
    uav_fix_type                   = ltm_satsfix & 0b00000011;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_GFRAMELENGTH - 4);
  }

  if (LTMcmd == LIGHTTELEMETRY_AFRAME)
  {
    uav_pitch = (int16_t)ltmread_u16();
    uav_roll =  (int16_t)ltmread_u16();
    uav_heading = (int16_t)ltmread_u16();
    if (uav_heading < 0 ) uav_heading = uav_heading + 360; //convert from -180/180 to 0/360°
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_AFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_SFRAME)
  {
    uav_bat = ltmread_u16();
    uav_amp = ltmread_u16();
    uav_rssi = (ltmread_u8()*100)/255;
    uav_airspeed = ltmread_u8()*100;
    ltm_armfsmode = ltmread_u8();
    uav_arm = ltm_armfsmode & 0b00000001;
    uav_failsafe = (ltm_armfsmode >> 1) & 0b00000001;
    uav_flightmode = (ltm_armfsmode >> 2) & 0b00111111;
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_SFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_OFRAME) // origin frame
  {
    uav_homelat = (int32_t)ltmread_u32();
    uav_homelon = (int32_t)ltmread_u32();
    uav_homealt = (int32_t)ltmread_u32();
    uav_osd_on = (int8_t)ltmread_u8(); // always 1
    uav_homefixstatus = (int8_t)ltmread_u8(); // home fix status
    
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_OFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_NFRAME)
  {
    uav_gpsmode = ltmread_u8();
    uav_navmode = ltmread_u8();
    uav_navaction = ltmread_u8();
    uav_WPnumber = ltmread_u8();
    ltm_naverror = ltmread_u8();
    ltm_flags = ltmread_u8();
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_NFRAMELENGTH - 4);
  }
  if (LTMcmd == LIGHTTELEMETRY_XFRAME)
  {
    uav_HDOP = ltmread_u16();
    uav_HWstatus = ltmread_u8();
    uav_spare1 = ltmread_u8();
    uav_spare2 = ltmread_u8();
    ltm_spare3 = ltmread_u8();
    memset(LTMserialBuffer, 0, LIGHTTELEMETRY_XFRAMELENGTH - 4);
  }
}