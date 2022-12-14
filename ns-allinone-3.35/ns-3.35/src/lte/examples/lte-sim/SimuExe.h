/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Sanjay Jaiman <sanjayjaiman@yahoo.com>
 */

#ifndef _SIMU_EXE_H_
#define _SIMU_EXE_H_

#include "LteSimulatorHelper.h"

namespace ns3 {

class UePrintInfo {
public:
	UePrintInfo() : id_(-1) {};
	UePrintInfo(Ptr<Node> node, Vector ue_pos) : node_(node), pos(ue_pos) {id_ = node->GetId();};

public:
  std::string pos_to_string() {
		std::stringstream os;
		os << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
		return os.str();
  }
  std::string ip_address_to_string() {
		std::stringstream os;
		os << ip_address;
		return os.str();
  }
	std::string ue_id_to_string() {
		std::stringstream os;
		os << "UE" << id_;
		return os.str();
	}
	std::string toString() {
		std::stringstream os;
		os << "Id = " << ue_id_to_string() << "; " << pos_to_string() << "; IP = " << ip_address_to_string() << "; tput = " << tput;
		return os.str();
	}

public:
    Ptr<Node> node_;
	uint32_t id_;
	Vector pos;
	ns3::Ipv4Address ip_address;
	double tput;
} ;

class SimuExe : public Object {
public:
    SimuExe(Ptr<SimulationParameters> params, Ptr<UeConfig> ue_cfg_);
    void run();
    void print_ue_map();
    void print_enb_info(std::ostringstream& os) {enb_cfg->to_string_comma_seperated(os);};
    void print_ue_info(std::ostringstream& os);
    void print_buildings_info(std::ostringstream& os);
    void print_buildings_info_comma_seperated(std::ostringstream& os) {
        lte_simu_helper->print_buildings_info_comma_seperated(os);
    };
    void print_ue_tput(std::ostringstream& os);
    void print_ue_pos(std::ostringstream& os);
    uint32_t get_num_ues() {return ue_map.size();}
    void print_path_loss_model(std::ostringstream& os, bool comma_seperated=false);

private:
    static bool print_init_info;
    Ptr<UeConfig> ue_cfg;
    Ptr<SimulationParameters> sim_params;
    bool test_mode;
    Ptr<LteSimulatorHelper> lte_simu_helper;
    Ptr<LteHelper> lte_helper;
    Ptr<Node> pgw;
    Ptr<PointToPointEpcHelper> epc_helper;
    Ptr<Node> remote_host;
	std::map<uint32_t, UePrintInfo> ue_map;
    Ptr<EnbConfig> enb_cfg;

    Ipv4InterfaceContainer ifaces_internet_devs;
    Ipv4StaticRoutingHelper ipv4_routing_helper;
    Ptr<Ipv4StaticRouting> remote_host_static_routing;
    Ptr<Ipv4> remote_host_ip;
    Ipv4Address default_gateway_addess;
    NodeContainer enb_nodes;
    InternetStackHelper internet;

private:
    void init();
    Ptr<EnbConfig> get_enb_cfg() {return enb_cfg;}
    void printEnbInfo(NodeContainer);
    void printUeInfo(NodeContainer);
    void plotNetwork(NodeContainer, NodeContainer);
    void printVruNetworkTopo(Ipv4InterfaceContainer, NodeContainer, NodeContainer);
    void PrintGnuplottableBuildingListToFile (std::string filename);
    void get_ip_for_ues(NodeContainer& ue_active_nodes);
    void update_tput_ue_map(Ipv4Address ip_address, double tput);
};

};

#endif