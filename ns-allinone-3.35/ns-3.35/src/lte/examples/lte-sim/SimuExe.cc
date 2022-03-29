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
 * Author: Sanjay Jaiman <sjaiman@parallelwireless.com>
 */

#include "SimuExe.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/gnuplot.h"
#include "ns3/gnuplot-helper.h"
#include "ns3/netanim-module.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SimuExe");

NS_OBJECT_ENSURE_REGISTERED (SimuExe);

bool SimuExe::print_init_info = true;

std::string get_ue_desc_str(NodeContainer& ue_nodes) {
	std::stringstream os;
	for (NodeContainer::Iterator iter = ue_nodes.Begin() ; iter != ue_nodes.End(); iter++) {
		Ptr<Node> node = *iter;
		Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
		Vector ue_pos = mobility->GetPosition();
		uint32_t id = node->GetId();
		os << "\tUE" << id << ": pos = (" << ue_pos.x << ", " << ue_pos.y << ", "<< ue_pos.z << ")" << std::endl;
	}
	return os.str();
}

SimuExe::SimuExe(Ptr<SimulationParameters> params, Ptr<UeConfig> ue_cfg_) : ue_cfg(ue_cfg_), sim_params(params) {
	test_mode = params->test_mode();
	init();
};

void SimuExe::print_path_loss_model(std::ostringstream& os, bool comma_seperated) {
	std::string delim = ((comma_seperated) ? "," : "\t");
	os << "PathlossModel" << delim <<  lte_simu_helper->GetPropagationLossModel() << "\n" <<
	"SpectrumChannelModel" << delim <<  lte_simu_helper->GetSpectrumChannelModel () << "\n" <<
	"FadingModel" << delim <<  lte_simu_helper->GetFadingLossModel () << std::endl;
}

void SimuExe::init() {
	uint32_t verbosity = sim_params->verbose;
	IntegerValue intValue;
    GlobalValue::GetValueByName ("RandomNumberGeneratorSeed", intValue);
    int seed_ = intValue.Get ();
	ns3::RngSeedManager::SetSeed(seed_);
	// Tx Power
    Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (20));
	// Transmission mode
	Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(sim_params->transMode));
	//---------------------------------------------------------------------------
	// 16. Save config in a config.file
	Config::SetDefault("ns3::ConfigStore::Filename", StringValue(sim_params->getConfigXmlFilename()));
	Config::SetDefault("ns3::ConfigStore::FileFormat", StringValue("Xml"));
	Config::SetDefault("ns3::ConfigStore::Mode", StringValue ("Save"));

	//---------------------------------------------------------------------------
	// 3. Logging
	//LogComponentEnable ("LteHelper", LOG_LEVEL_ALL);
	//LogComponentEnable("Ipv4", LOG_LEVEL_ALL);
	//LogComponentEnable("UdpServer", LOG_LEVEL_ALL);
	//LogComponentEnable("UdpClient", LOG_LEVEL_ALL);
	//LogComponentEnable("UdpTraceClient", LOG_LEVEL_ALL);
	//LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);
	//LogComponentEnable("RrFfMacScheduler", LOG_LEVEL_INFO);
	//LogComponentEnable("PfFfMacScheduler", LOG_LEVEL_ALL);

	//---------------------------------------------------------------------------
	// 1. LTE helper
	lte_helper = CreateObject<LteHelper>();
	lte_simu_helper = CreateObject<LteSimulatorHelper>(lte_helper, sim_params);

	//---------------------------------------------------------------------------
	// 2. EPC helper
	epc_helper = CreateObject<PointToPointEpcHelper> ();
	lte_helper->SetEpcHelper (epc_helper);
	pgw = epc_helper->GetPgwNode();
	default_gateway_addess = epc_helper->GetUeDefaultGatewayAddress();
	//---------------------------------------------------------------------------
	// 3. Add eNodeBs
	
	enb_cfg = CreateObject<EnbConfig>();
	std::string enb_pos_file = sim_params->enb_pos_file();
	if (enb_pos_file == "") {
		std::cout << "ENB file not specified" << std::endl;
		exit(0);
	}
	enb_cfg->put_enbs(enb_pos_file);
	if (verbosity >= 1) {
		enb_cfg->Print();
	}
	Ptr<ListPositionAllocator> enbPositionAlloc = enb_cfg->get_enb_position_alloc();
	MobilityHelper enbMobility;
	enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	enbMobility.SetPositionAllocator(enbPositionAlloc);

	uint32_t num_enbs_allocated = enbPositionAlloc->GetSize();
	enb_nodes.Create(num_enbs_allocated);
	enbMobility.Install(enb_nodes);

	std::ostringstream os;

	lte_simu_helper->add_enb_nodes(enb_nodes, enb_cfg, os);  //Sets path loss model / fading as well

	lte_simu_helper->LogEnable(verbosity > 1);
	lte_simu_helper->verify_init_values(verbosity, sim_params->print_defaults());
    if (verbosity >= 1) {
	    lte_simu_helper->print_lte_helper_values();
    }

	BooleanValue bool_val;
	// Connect to trace sources in all eNodeB
	Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
					MakeCallback (&LteSimulatorHelper::ConnectionEstablishedCallback, lte_simu_helper));
	Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
					MakeCallback (&LteSimulatorHelper::HandoverStartCallback, lte_simu_helper));
	Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
					MakeCallback (&LteSimulatorHelper::EnbHandoverEndOkCallback, lte_simu_helper));
	Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/NotifyConnectionRelease",
					MakeCallback (&LteSimulatorHelper::NotifyConnectionReleaseCallback, lte_simu_helper));
	lte_simu_helper->GetAttribute("TestCellBringDown", bool_val);
	if (bool_val.Get()) {
  		Simulator::Schedule (Seconds (CELL_SHUT_DOWN_TIME),
                       &LteSimulatorHelper::CellShutdownCallback, lte_simu_helper);
	}

	//---------------------------------------------------------------------------
	// 4. Create a single RemoteHost
	// 5. Install IP stack and address

	NodeContainer remote_host_container;
	remote_host_container.Create(1);
	remote_host = remote_host_container.Get(0);
	internet.Install(remote_host_container);
	Ipv4AddressHelper ipv4Addh;
	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
	p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
	p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
	ipv4Addh.SetBase("10.0.0.0", "255.0.0.0");
	NetDeviceContainer internetDevices = p2ph.Install(pgw, remote_host);
	ifaces_internet_devs = ipv4Addh.Assign(internetDevices);
	std::vector<uint32_t> enb_ids = enb_cfg->get_indicies();

	//---------------------------------------------------------------------------
	// 6. UEs

	lte_simu_helper->install_ue_nodes(enb_cfg, ue_cfg, verbosity);
	ue_map.empty();
	
	NodeContainer ue_active_nodes = lte_simu_helper->get_active_ue_nodes();
	NodeContainer idle_ue_nodes = lte_simu_helper->get_idle_ue_nodes();
	NodeContainer all_ue_nodes = lte_simu_helper->get_ue_nodes();
//	std::cout << "******** num_ues_allocated = " << all_ue_nodes.GetN() << " ********" << std::endl;

	if (ue_active_nodes.GetN()) {
		os << "================================================================================" << std::endl;
		os << "\nNum Active UEs allocated = " << ue_active_nodes.GetN()<< "; UE cfg file = " << sim_params->ue_pos_file() << std::endl;
		os << "Active UEs - \n";
		os << get_ue_desc_str(ue_active_nodes);
		for (NodeContainer::Iterator iter = ue_active_nodes.Begin() ; iter != ue_active_nodes.End(); iter++) {
			Ptr<Node> node = *iter;
			Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
			Vector ue_pos = mobility->GetPosition();
			uint32_t id = node->GetId();
//			os << "\tUE" << id << ": pos = (" << ue_pos.x << ", " << ue_pos.y << ", "<< ue_pos.z << ")" << std::endl;
			ue_map[id] = UePrintInfo(node, ue_pos);
		}
	}


	std::string active_ue_str;
	os << "\nNum ENBs allocated = " << num_enbs_allocated << "; ENB cfg file = " << sim_params->enb_pos_file() << std::endl;
	for (NodeContainer::Iterator iter = enb_nodes.Begin() ; iter != enb_nodes.End(); iter++) {
		Ptr<Node> node = *iter;
		uint32_t id = node->GetId();
		Vector enb_pos = enbPositionAlloc->GetNext();
		os << "\tENB" << id << " pos = (" << enb_pos.x << ", " << enb_pos.y << ", "<< enb_pos.z << ")" << std::endl;
	}

	if (sim_params->use_buildings()) {
		BuildingsHelper::Install (enb_nodes);
		BuildingsHelper::Install (all_ue_nodes);
	//		std::cout << "**** BuildingsHelper::Install DONE" << std::endl;
		lte_simu_helper->add_buildings();

		// The following call will pull pull all the building objects from a static list inside BuildingList class
		// and mobility from enb_nodes object.  It will put the buildings into the enb mobility object.

		if (verbosity >= 1) {
			os << "================================================================================" << std::endl;
			lte_simu_helper->print_buildings_info(os);
			PrintGnuplottableBuildingListToFile ("buildings.txt");
		}
	}

	os << "================================================================================\n";
	print_path_loss_model(os);
	os << "================================================================================\n";
	os << std::endl;

	if (idle_ue_nodes.GetN()) {
		os << "Idle UEs - \n";
		os << get_ue_desc_str(idle_ue_nodes);
	}
	if (print_init_info) {
		std::cout << os.str();
		print_init_info = false;
	}
	if (test_mode) {
		std::cout << active_ue_str;
	}

	os.str(std::string(""));
	//---------------------------------------------------------------------------
	// 11. UE - Assign IP
	// UEs: IP stack

	if (!remote_host_ip) {
		remote_host_ip = remote_host->GetObject<Ipv4>();
	} 
	
	NS_ASSERT(remote_host_ip != nullptr);

	//---------------------------------------------------------------------------
	// 10. IP Routing
	if (! remote_host_static_routing) {
		remote_host_static_routing = ipv4_routing_helper.GetStaticRouting(remote_host_ip);
	}
	remote_host_static_routing->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
				Ipv4Mask("255.0.0.0"), 1);

	std::vector<uint32_t> enb_vec = enb_cfg->get_indicies();
	NetDeviceContainer net_;
	std::map<uint32_t, NetDeviceContainer> ue_container_per_enb_id_map;

//  Attaching UEs to their closest ENBs
	std::map<uint32_t, NodeContainer> enb_to_UEs_map;
	lte_simu_helper->find_ues_for_closest_enbid(enb_cfg, all_ue_nodes, enb_to_UEs_map, os);
	for (std::map<uint32_t, NodeContainer>::iterator iter = enb_to_UEs_map.begin(); iter != enb_to_UEs_map.end(); iter++) {
		uint32_t enb_id = iter->first;
		NodeContainer ue_nodes = iter->second;
		uint32_t num = ue_nodes.GetN();
//		std::cout << "ENB" << enb_id << " : Num UEs to attach:: " << num << std::endl;
		if (num) {
//				std::cout << "-- Attaching " << num << " UEs to ENB" << enb_id << std::endl;
			ue_container_per_enb_id_map[enb_id].Add(lte_helper->InstallUeDevice(ue_nodes));
			net_.Add(ue_container_per_enb_id_map[enb_id]);
		}
		else {
			std::cout << "-- NO Ues to attach to ENB" << enb_id << std::endl;
		}
		lte_simu_helper->printUeToEnbID(ue_container_per_enb_id_map[enb_id], ue_nodes, os);
	}

/*  This should do the same as above
		for (std::vector<uint32_t>::iterator iter = enb_vec.begin(); iter != enb_vec.end(); iter++) {
			uint32_t enb_id = *iter;

			NodeContainer ue_nodes;
			uint32_t num = lte_simu_helper->sift_ues_for_enbid(enb_id, all_ue_nodes, ue_nodes);
			if (num) {
				ue_container_per_enb_id_map[enb_id].Add(lte_helper->InstallUeDevice(ue_nodes));
				net_.Add(ue_container_per_enb_id_map[enb_id]);
			}
			else {
				std::cout << "NO Ues to attach to ENB" << enb_id << std::endl;
			}
		}
		*/
	//	os << "-- Net devices --" << std::endl;

	internet.Install(all_ue_nodes);
	NS_ASSERT_MSG(net_.GetN() != 0, "No Net devices found");
	Ipv4InterfaceContainer ueIfaces = epc_helper->AssignUeIpv4Address(net_);
	// Assign IP address to UEs, and install applications
	for (NodeContainer::Iterator iter = all_ue_nodes.Begin() ; iter != all_ue_nodes.End(); iter++) {
		Ptr<Node> ueNode = *iter;
		// Set the default gateway for the UE
		Ptr<Ipv4StaticRouting> ueStaticRouting =
				ipv4_routing_helper.GetStaticRouting(ueNode->GetObject<Ipv4>());
		ueStaticRouting->SetDefaultRoute(default_gateway_addess, 1);
	}

	get_ip_for_ues(ue_active_nodes);

	if (sim_params->use_buildings()) {
		lte_simu_helper->GetAttribute("TestPropagationLoss", bool_val);
//		std::cout << "**** VAL from GetAttribute  TestPropagationLoss = " << bool_val.Get() << std::endl;
		if (bool_val.Get()) {
			Ptr<MobilityModel> mmEnb = enb_nodes.Get(0)->GetObject<MobilityModel> ();
			lte_simu_helper->testPropagationLoss (mmEnb, all_ue_nodes);
		}
    }

	//---------------------------------------------------------------------------
	// 12. Attach UEs to the coses eNB
	for (std::map<uint32_t, NetDeviceContainer>::iterator iter = ue_container_per_enb_id_map.begin(); iter != ue_container_per_enb_id_map.end(); iter++) {
		uint32_t enb_id = iter->first;
//			std::cout << "-- Attaching to ENB" << enb_id << std::endl;
		NetDeviceContainer ue_net_dev_container = iter->second;
//			std::cout << "\tNum UEs = " << ue_net_dev_container.GetN() << std::endl;
		Ptr<LteEnbNetDevice> enb_dev_ptr = lte_simu_helper->get_ENB_net_device(enb_id);  // Each container contains 1 ENB
		NS_ASSERT_MSG(enb_dev_ptr != nullptr, "NO ATTACH to ENB; enb_dev_ptr is NULL");
//			std::cout << "\tCell ID = " << enb_dev_ptr->GetCellId() << std::endl;
		lte_helper->Attach(ue_net_dev_container, enb_dev_ptr);
	}


	//-------------------------------------------------------------------------------------
	// 14. UDP Application - DL

	if (sim_params->runDlUdpApp) {
		ApplicationContainer dl_client_apps;
		ApplicationContainer dl_server_apps;
		Time     inter_packet_interval = sim_params->get_dl_inter_packet_interval();
		uint32_t max_packet_count = sim_params->get_dl_max_packet_count();
		os << "\n================================================================================" << std::endl;
		os << "Setting up DL client/server Apps::\n";
		for (NodeContainer::Iterator iter = ue_active_nodes.Begin();
			iter != ue_active_nodes.End(); ++iter) {
			Ptr<Node> ueNode = *iter;
			uint32_t uePort = sim_params->get_new_dl_port();
			uint32_t remote_host_port = sim_params->get_new_dl_remote_port() ;
			lte_simu_helper->add_to_apps(remote_host, ueNode, dl_client_apps, dl_server_apps, remote_host_port, uePort, inter_packet_interval, max_packet_count, os);
		}
		double serv_start_time = sim_params->dlUdp.get_server_start_time();
		double serv_stop_time = sim_params->dlUdp.get_server_stop_time();
		double client_start_time = sim_params->dlUdp.get_client_start_time();
		double client_stop_time = sim_params->dlUdp.get_client_stop_time();
		os << "\tDL Serv start time " << serv_start_time << std::endl;
		os << "\tDL Serv stop time " << serv_stop_time << std::endl;
		os << "\tDL Client start time " << client_start_time << std::endl;
		os << "\tDL Client stop time " << client_stop_time << std::endl;
		dl_server_apps.Start(Seconds(serv_start_time));
		dl_server_apps.Stop(Seconds(serv_stop_time));
		dl_client_apps.Start(Seconds(client_start_time));
		dl_client_apps.Stop(Seconds(client_stop_time));
	}

	// UDP Application - UL
	if (sim_params->runUlUdpApp) {
		ApplicationContainer ul_client_apps;
		ApplicationContainer ul_server_apps;
		Time     inter_packet_interval = sim_params->get_ul_inter_packet_interval();
		uint32_t max_packet_count = sim_params->get_ul_max_packet_count();
		os << "\n================================================================================" << std::endl;
		os << "Setting up UL client/server Apps::\n";
		for (NodeContainer::Iterator iter = ue_active_nodes.Begin();
			iter != ue_active_nodes.End(); ++iter) {
			Ptr<Node> ueNode = *iter;
			uint32_t uePort = sim_params->get_new_ul_port();
			uint32_t remote_host_port = sim_params->get_new_ul_remote_port();
			lte_simu_helper->add_to_apps(ueNode, remote_host, ul_client_apps, ul_server_apps, uePort, remote_host_port, inter_packet_interval, max_packet_count, os);
		}
		uint32_t serv_start_time = sim_params->ulUdp.get_server_start_time();
		uint32_t serv_stop_time = sim_params->ulUdp.get_server_stop_time();
		uint32_t client_start_time = sim_params->ulUdp.get_client_start_time();
		uint32_t client_stop_time = sim_params->ulUdp.get_client_stop_time();
		os << "\tUL Serv start time " << serv_start_time << std::endl;
		os << "\tUL Serv stop time " << serv_stop_time << std::endl;
		os << "\tUL Client start time " << client_start_time << std::endl;
		os << "\tUL Client start time " << client_stop_time << std::endl;
		ul_server_apps.Start(Seconds(serv_start_time));
		ul_server_apps.Stop(Seconds(serv_stop_time));
		ul_client_apps.Start(Seconds(client_start_time));
		ul_client_apps.Stop(Seconds(client_stop_time));
	}

	if (verbosity >= 1) {
		std::cout << os.str();
	}

	if (! test_mode) {
		//---------------------------------------------------------------------------
		// 17. tmp debug working log
		printEnbInfo(enb_nodes);
		printUeInfo(ue_active_nodes);
		printVruNetworkTopo(ifaces_internet_devs, enb_nodes, ue_active_nodes);
	}

}

void SimuExe::run() {

	//---------------------------------------------------------------------------
	// 15. Install FlowMonitor
	std::ostringstream os;
	Ptr<ListPositionAllocator> enbPositionAlloc = enb_cfg->get_enb_position_alloc();
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

	Config::RegisterRootNamespaceObject (enbPositionAlloc);

	if (sim_params->print_logs()) {
		ConfigStore outputConfig;
		outputConfig.ConfigureDefaults();
		outputConfig.ConfigureAttributes();
	}

	if (sim_params->print_traces()) {
		//-------------------------------------------------------------------------------------
		// 13. Enable Traces
		lte_helper->EnableTraces();
		lte_helper->EnablePhyTraces();
		lte_helper->EnableMacTraces();

		lte_helper->EnableDlMacTraces();
		lte_helper->EnableDlPhyTraces();
		lte_helper->EnableDlRxPhyTraces();
		lte_helper->EnableDlTxPhyTraces();
		lte_helper->EnableUlMacTraces();
		lte_helper->EnableUlPhyTraces();
		lte_helper->EnableUlRxPhyTraces();
		lte_helper->EnableUlTxPhyTraces();
	}

	//---------------------------------------------------------------------------
	// 18. Simulator
	if (! test_mode) {
		std::cout << "Starting Simulation..." << std::endl;
	}
	Simulator::Stop(Seconds(sim_params->SimulationTimeSec));

	Simulator::Schedule(Seconds(sim_params->simMsgIntervalSec), &SimulationParameters::simRunUpdates, sim_params);

	if (sim_params->print_logs()) {
		AnimationInterface anim ("ulla.xml");
		anim.SetConstantPosition(enb_nodes.Get(0),0,0,0);
		anim.SetConstantPosition(remote_host,0,0,0);
	}
	Simulator::Run();

	//---------------------------------------------------------------------------
	// 19. Flow Monitor
	monitor->CheckForLostPackets ();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());

	if (sim_params->print_logs()) {
		// Save to xml file
		std::string xmlFileName = sim_params->getLogXmlFilename();
		flowmon.SerializeToXmlFile(xmlFileName, true, true);
	}
	//---------------------------------------------------------------------------
	// 20. Stat debug output
	os.str(std::string());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
	{
		if (i->first > 0) {
			Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
			double rx_delta_time = i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstRxPacket.GetSeconds();
			double tx_delta_time = i->second.timeLastTxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds();
			double tput = (rx_delta_time > 0) ? ((i->second.rxBytes * 8.0 / rx_delta_time) / (1000 * 1000)) : 0;
			double tx_offered = (tx_delta_time > 0) ? ((i->second.txBytes * 8.0 / tx_delta_time) / (1000 * 1000)) : 0;
			Ipv4Address src_ip = t.sourceAddress;
			Ipv4Address dst_ip = t.destinationAddress;		
			update_tput_ue_map(dst_ip, tput);
			os << "Flow " << i->first << " (" <<src_ip << " -> " << dst_ip << ") " << std::endl;
			os << "\tTx: Tx Packets[" << i->second.txPackets
					<< "] Tx Bytes[" << i->second.txBytes << "]"
					<< " TxOffered["
					<< tx_offered << "]Mbps" << std::endl;
			os << "\tRx: Rx Packets[" << i->second.rxPackets << "] Rx Bytes[" << i->second.rxBytes << "] Throughput["
					<< tput << "]Mbps" << std::endl;
		}
	}
	if (! test_mode) {
		std::cout << os.str();
	}
	//---------------------------------------------------------------------------
	// Clean up
	Simulator::Destroy();
}

/**
 * Print network topology
 */
void SimuExe::printVruNetworkTopo(Ipv4InterfaceContainer ifaces, NodeContainer enb_nodes,
		NodeContainer ue_nodes) {
	std::cout << std::endl;
	std::cout << "RemoteHost <----p2p----->  _______EPC_________ " << std::endl;
	std::cout << "                             P-GW      S-GW    <------------> eNB <-------------> UE" << std::endl;
	std::cout << ifaces.GetAddress(0) << "                   10.0.0.PGW  10.0.0.6" << std::endl;
	std::cout << ifaces.GetAddress(1);

	// eNB
	for (NodeContainer::Iterator nodei = enb_nodes.Begin();
			nodei != enb_nodes.End(); ++nodei) {
		Ptr<Ipv4> ipv4 = (*nodei)->GetObject<Ipv4>();
		std::cout << "                                                    "
				<< (ipv4->GetAddress(1, 0)).GetLocal() << std::endl;
	}

	// UE
	for (NodeContainer::Iterator nodei = ue_nodes.Begin();
			nodei != ue_nodes.End(); ++nodei) {
		Ptr<Ipv4> ipv4 = (*nodei)->GetObject<Ipv4>();
		std::cout << "                                                                               "
				<< (ipv4->GetAddress(1, 0)).GetLocal() << std::endl;
	}

	std::cout << std::endl;
	std::cout << " no of UEs: " << ue_nodes.GetN() << std::endl;
	std::cout << std::endl
			<< "================================================================================"
			<< std::endl;

}


/**
 * Print eNB information
 */
void SimuExe::printEnbInfo(NodeContainer enb_nodes) {

	std::cout << std::endl << "================================================================================" << std::endl;
	std::cout << "eNB Info" << std::endl
			<< "------------------------------------------------------------" << std::endl;

	//
	for (NodeContainer::Iterator nodei = enb_nodes.Begin();
			nodei != enb_nodes.End(); ++nodei) {
		Ptr<MobilityModel> mobility = (*nodei)->GetObject<MobilityModel>();
		Vector pos = mobility->GetPosition();

		Ptr<Ipv4> ipv4 = (*nodei)->GetObject<Ipv4>();

		Ptr<LteEnbNetDevice> enbDev = (*nodei)->GetDevice(0)->GetObject<LteEnbNetDevice>();
		Ptr<LteEnbRrc> enbRrc = enbDev->GetRrc();

		Ptr<LteEnbPhy> phy_ = enbDev->GetPhy();
		double tx_pwr = phy_->GetTxPower();
		std::cout  << std::endl;

		std::cout << "eNB" << (*nodei)->GetId() << " = (" << pos.x << "," << pos.y
				<< "," << pos.z << ")" << "\n\tTx POWER = " << tx_pwr << "\n\tInterfaces: lo "
				<< (ipv4->GetAddress(0, 0)).GetLocal() << " eth "
				<< (ipv4->GetAddress(1, 0)).GetLocal() << std::endl;

	}
	std::cout << std::flush;
}

void SimuExe::print_ue_map() {
	for (std::map<uint32_t, UePrintInfo>::iterator iter = ue_map.begin(); iter != ue_map.end(); iter++) {
		uint32_t id = iter->first;
		UePrintInfo& ue_print_info = iter->second;
		std::cout << "UE" << id << "::" << ue_print_info.toString() << std::endl;
	}	
}
void SimuExe::print_buildings_info(std::ostringstream& os) {
	lte_simu_helper->print_buildings_info(os);
}

void SimuExe::print_ue_info(std::ostringstream& os) {
	for (std::map<uint32_t, UePrintInfo>::iterator iter = ue_map.begin(); iter != ue_map.end(); iter++) {
//		uint32_t id = iter->first;
		UePrintInfo& ue_print_info = iter->second;
		os << "Id = " << ue_print_info.ue_id_to_string() << std::endl;
		os << "IP = " << ue_print_info.ip_address_to_string() << std::endl;
		break;
	}
}

void SimuExe::print_ue_pos(std::ostringstream& os) {
	for (std::map<uint32_t, UePrintInfo>::iterator iter = ue_map.begin(); iter != ue_map.end(); iter++) {
//		uint32_t id = iter->first;
		UePrintInfo& ue_print_info = iter->second;
		os << ue_print_info.pos_to_string() << "\t";
	}
}

void SimuExe::print_ue_tput(std::ostringstream& os) {
	for (std::map<uint32_t, UePrintInfo>::iterator iter = ue_map.begin(); iter != ue_map.end(); iter++) {
//		uint32_t id = iter->first;
		UePrintInfo& ue_print_info = iter->second;
		os << ue_print_info.tput << ",";
		break;
	}
}

void SimuExe::update_tput_ue_map(Ipv4Address ip_address, double tput) {
	for (std::map<uint32_t, UePrintInfo>::iterator iter = ue_map.begin(); iter != ue_map.end(); iter++) {
		uint32_t id = iter->first;
		UePrintInfo& ue_print_info = iter->second;
		if (ip_address == ue_print_info.ip_address) {
			NS_ASSERT(ue_print_info.id_ == id);
			ue_print_info.tput = tput;
		}
	}
}


void SimuExe::get_ip_for_ues(NodeContainer& ue_nodes) {
	
	for (NodeContainer::Iterator nodei = ue_nodes.Begin();
			nodei != ue_nodes.End(); ++nodei) {
		Ptr<Node> node = *nodei;
		uint32_t id = node->GetId();
//		std::cout << "get_ip_for_ues: ID = " << id << std::endl;
		Ptr<LteUeNetDevice> lte_net_dev = (*nodei)->GetDevice(0)->GetObject<LteUeNetDevice>();
		NS_ASSERT_MSG(lte_net_dev != nullptr, "lte_net_dev NULL");
		Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
		Ipv4Address ip = ipv4->GetAddress(1, 0).GetLocal();
		ue_map[id].ip_address = ip;
	}	
}

/**
 * Print UE information
 */
void SimuExe::printUeInfo(NodeContainer ue_nodes) {

	std::cout << "UE Info" << std::endl
			<< "------------------------------------------------------------" << std::endl;

	//
	for (NodeContainer::Iterator nodei = ue_nodes.Begin();
			nodei != ue_nodes.End(); ++nodei) {
		Ptr<Node> node = *nodei;
		Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
		Ptr<ConstantPositionMobilityModel> cp_mm = node->GetObject<ConstantPositionMobilityModel> ();
		Ptr<RandomWalk2dMobilityModel> rw_mm = node->GetObject<RandomWalk2dMobilityModel> ();
		Ptr<ConstantVelocityMobilityModel> cv_mm = node->GetObject<ConstantVelocityMobilityModel> ();
		Ptr<LteUeNetDevice> lte_net_dev = (*nodei)->GetDevice(0)->GetObject<LteUeNetDevice>();
		Vector pos = mobility->GetPosition();
		Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
		std::string mm_name = ((cp_mm) ? "ConstantPosition" : 
								((rw_mm) ? "RandomWalk2d" : 
								((cv_mm) ? "ConstantVelocity" : "Unknown")));
		std::cout << "UE" << node->GetId() << " = (" << pos.x << "," << pos.y << "," << pos.z << ") " << mm_name << "\n\t"
				<< "IMSI: " << lte_net_dev->GetImsi() << "\n\t"
				<< "Interfaces: lo " << (ipv4->GetAddress(0, 0)).GetLocal() 
				<< " eth " << (ipv4->GetAddress(1, 0)).GetLocal() << std::endl;
	}
	std::cout << "================================================================================" << std::endl;
}


/**
 * Plot network topology with gnuplot
 */
void SimuExe::plotNetwork(NodeContainer enb_nodes, NodeContainer ue_nodes) {
	std::string plotfilename = "pw_network_topology_plot.plt";

	// Plot instantiate
	Gnuplot plot("check11.png");
	plot.SetTitle("check plot");
	// Make graphic file - used by Gnuplot
	plot.SetTerminal("png");
	plot.SetLegend("X Axis", "Y Axis");
	plot.AppendExtra("set xrange [-200:+200]");
	plot.AppendExtra("set yrange [-200:+200]");

	// Instantiate the eNB data
	Gnuplot2dDataset enbSet;
	enbSet.SetTitle("eNB");
	enbSet.SetStyle(Gnuplot2dDataset::POINTS);

	// Plot eNB
	for (NodeContainer::Iterator nodei = enb_nodes.Begin();
			nodei != enb_nodes.End(); ++nodei) {
		Ptr<MobilityModel> mobility = (*nodei)->GetObject<MobilityModel>();
		Vector pos = mobility->GetPosition();
		enbSet.Add(pos.x, pos.y);
	}
	// Add to the plot
	plot.AddDataset(enbSet);

	// Instantiate UE data
	Gnuplot2dDataset ueSet;
	ueSet.SetTitle("UE");
	ueSet.SetStyle(Gnuplot2dDataset::POINTS);
	for (NodeContainer::Iterator nodei = ue_nodes.Begin();
			nodei != ue_nodes.End(); ++nodei) {
		Ptr<MobilityModel> mobility = (*nodei)->GetObject<MobilityModel>();
		Vector pos = mobility->GetPosition();
		ueSet.Add(pos.x, pos.y);
	}
	// Add to the plot
	plot.AddDataset(ueSet);

	// Open/Write plot file
	std::ofstream plotFile(plotfilename.c_str());
	plot.GenerateOutput(plotFile);
	plotFile.close();
}

void
SimuExe::PrintGnuplottableBuildingListToFile (std::string filename)
{
  std::ofstream outFile;
  outFile.open (filename.c_str (), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open ())
    {
      NS_LOG_ERROR ("Can't open file " << filename);
      return;
    }
  uint32_t index = 0;
  for (BuildingList::Iterator it = BuildingList::Begin (); it != BuildingList::End (); ++it)
    {
      ++index;
      Box box = (*it)->GetBoundaries ();
      outFile << "set object " << index
              << " rect from " << box.xMin  << "," << box.yMin
              << " to "   << box.xMax  << "," << box.yMax
              << std::endl;
    }
}

}
