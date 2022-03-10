/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Sanjay Jaiman <sjaiman@parrallelwireless.com>
 */

#ifndef _LTE_SIMULATOR_PARAMS_H_
#define _LTE_SIMULATOR_PARAMS_H_

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/epc-helper.h"
#include "ns3/config-store.h"
#include "ns3/applications-module.h"

namespace ns3 {

#define DL_UDP_LOCAL_PORT 11234
#define DL_UDP_REMOTE_HOST_PORT 18000
#define DL_UDP_INTERPACKET_INTERVAL 0.0001
#define DL_UDP_MAX_PACKET_COUNT 1000000000
#define DL_UDP_SERVER_START_TIME 0.5
#define DL_UDP_CLIENT_START_TIME 1.0

#define UL_UDP_LOCAL_PORT 1234
#define UL_UDP_REMOTE_HOST_PORT 8000
#define UL_UDP_INTERPACKET_INTERVAL 0.0001
#define UL_UDP_MAX_PACKET_COUNT 1000000000
#define UL_UDP_SERVER_START_TIME 1.0
#define UL_UDP_CLIENT_START_TIME 1.5

#define STOP_TIME_WILL_GET_SET_LATER 0  // Will get set later


class UpdConnect {
public:
	UpdConnect(uint16_t UePort, uint16_t RemoteHostPort, double interPacketInterval, uint32_t MaxPacketCount,
      double ServerStartTimeSec, double ClientStartTimeSec) : 
		m_local_port(UePort), m_remote_port(RemoteHostPort), m_maxPacketCount(MaxPacketCount),
    m_serverStartTimeSec(ServerStartTimeSec), m_clientStartTimeSec(ClientStartTimeSec),
    m_serverStopTimeSec(STOP_TIME_WILL_GET_SET_LATER), m_clientStopTimeSec(STOP_TIME_WILL_GET_SET_LATER) {
    m_interPacketInterval = Seconds(interPacketInterval);
    };
  uint16_t get_local_port_and_increment() {++m_local_port; return (m_local_port-1);}
  uint16_t get_remote_port_and_increment() {++m_remote_port; return (m_remote_port-1);}
  Time     get_inter_packet_interval() {return m_interPacketInterval;};
  uint32_t get_max_packet_count() {return m_maxPacketCount;};
  double   get_server_start_time() {return m_serverStartTimeSec;};
  double   get_server_stop_time() {return m_serverStopTimeSec;};
  double   get_client_start_time() {return m_clientStartTimeSec;};
  double   get_client_stop_time() {return m_clientStopTimeSec;};
  void     set_simu_stop_time(double stop_time) {
    m_serverStopTimeSec = stop_time; m_clientStopTimeSec = stop_time - 0.1;
  };
private:
	uint16_t m_local_port;
	uint16_t m_remote_port;
	Time m_interPacketInterval;
	uint32_t m_maxPacketCount;
	double m_serverStartTimeSec;
	double m_clientStartTimeSec;
	double m_serverStopTimeSec;
	double m_clientStopTimeSec;
};

class SimulationParameters : public Object
{
public:
  /**
   * Constructor
   */
  SimulationParameters();

  /**
   * Destructor
   */
  virtual ~SimulationParameters () {};

  // inherited from Object
  /**
   *  Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId (void);
  static void simRunUpdates(Ptr<SimulationParameters> params);

  std::string getLogXmlFilename();
  std::string getConfigXmlFilename();
  template <class T> static bool getInitialVal(std::string& tidName, std::string paramName, T& value, uint32_t verbose);
  void configureSimulationParams(int argc, char *argv[], CommandLine cmd);
  uint32_t getUePositionAlloc(Ptr<ListPositionAllocator>& uePositionAlloc, NodeContainer enbNodes);
  std::string ue_pos_file() {return m_ue_positions_file;};
  std::string enb_pos_file() {return m_enb_positions_file;};
  std::string building_pos_file() {return m_building_positions_file;};
  bool use_buildings() {return m_use_buildings;}
  uint32_t get_new_ul_port() {return ulUdp.get_local_port_and_increment();};
  uint32_t get_new_dl_port() {return dlUdp.get_local_port_and_increment();};
  uint32_t get_new_ul_remote_port() {return ulUdp.get_remote_port_and_increment();};
  uint32_t get_new_dl_remote_port() {return dlUdp.get_remote_port_and_increment();};
  Time     get_ul_inter_packet_interval() {return ulUdp.get_inter_packet_interval();};
  Time     get_dl_inter_packet_interval() {return dlUdp.get_inter_packet_interval();};
  uint32_t get_ul_max_packet_count() {return ulUdp.get_max_packet_count();};
  uint32_t get_dl_max_packet_count() {return dlUdp.get_max_packet_count();};
  bool     test_mode() {return test;}
  bool     print_defaults() {return print_defaults_;}
  bool     print_logs() {return print_logs_;}
  bool     print_traces() {return print_traces_;}

  void print() {

    std::cout << "  UE positions file = " << ue_pos_file() << std::endl;
    std::cout << "  Enb positions file = " << enb_pos_file() << std::endl;
    std::cout << "  Building positions file = " << building_pos_file() << std::endl;
    std::cout << "  Simulation time = " << SimulationTimeSec << std::endl;
    std::cout << "  Simulation message inerval = " << simMsgIntervalSec << std::endl;
    std::cout << "  Test mode = " << ((test) ? "True" : "False") << std::endl;
  }


public:
//////////////////////////////////////////////////////////////////////////////////
// Simulation Parameters
//////////////////////////////////////////////////////////////////////////////////

	// Simulation Parameters
	double SimulationTimeSec;
	double simMsgIntervalSec;

	// Network Topology Parameters
	uint16_t m_numEnb;
	uint16_t m_numUes;
	double enbCoverageRadiusm;

	// change to enum
	uint16_t transMode;
  std::string   m_ue_rand_walk_min_max_speed;
	std::string   m_ue_positions_file;
	std::string   m_enb_positions_file;
	std::string   m_building_positions_file;
	// UDP DL/UL Application
	bool runDlUdpApp;
	bool runUlUdpApp;
	bool	m_use_buildings;

	UpdConnect dlUdp;
	UpdConnect ulUdp;
  uint32_t   verbose;
  bool       print_defaults_;
  bool       print_logs_;
  bool       print_traces_;
  bool       test;
};

};

#endif // _LTE_SIMULATOR_PARAMS_H_