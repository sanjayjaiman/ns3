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

#include "SimulationParams.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SimulationParameters");

NS_OBJECT_ENSURE_REGISTERED (SimulationParameters);


static ns3::GlobalValue g_rand_num_generator_seed ("RandomNumberGeneratorSeed",
                                            "Random Number Generator Seed",
                                            ns3::IntegerValue (3),
                                            ns3::MakeIntegerChecker<uint32_t> ());

static ns3::GlobalValue g_macroEnbTxPowerDbm ("macroEnbTxPowerDbm",
                                             "TX power [dBm] used by macro eNBs",
                                             ns3::DoubleValue (46.0),
                                             ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_interSiteDistance ("interSiteDistance",
                                            "min distance between two nearby macro cell sites",
                                            ns3::DoubleValue (500),
                                            ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_enbCoverageRadiusm ("EnbCoverageRadiusM",
                                            "Radius of EnbCoverage",
                                            ns3::DoubleValue (50),    //248
                                            ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_simulationTimeSec ("SimulationTimeSec",
                                            "Total simulation time in seconds",
                                            ns3::DoubleValue (15),
                                            ns3::MakeDoubleChecker<double> ());

static ns3::GlobalValue g_runDlTest ("SimulationRunDlTest",
									"Run Downlink tput test",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_runUlTest ("SimulationRunUlTest",
									"Run Uplink tput test",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_printDefaults ("PrintDefaultParams",
									"Print defaults",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_printLteTraces ("PrintLteTraces",
									"Print Lte Traces",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_printLogs ("PrintLogs",
									"Print logs",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_test_mode ("SimulationTestMode",
									"Run simulator in test mode",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());

static ns3::GlobalValue g_num_ues_in_test ("SimulationTestModeNumUEs",
                                            "Nums UEs in test",
                                            ns3::IntegerValue (1),
                                            ns3::MakeIntegerChecker<uint32_t> ());



static ns3::GlobalValue g_simulation_test_output_filename ("SimulationTestOutputFile",
                                            "SimulationTestOutputFile",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_simulation_test_x_Range ("SimulationTest_X_Range",
                                            "UE x min | max value",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_simulation_test_y_Range ("SimulationTest_Y_Range",
                                            "UE y min | max value",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_ueRandWalkMinMaxSpeed ("UeRandWalkMinMaxSpeed",
                                            "UE Random Walk Min-Max Speed",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_UePositionsFile ("UePositionsFile",
                                            "Ue Positions Specified or not",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_EnbPositionsFile ("EnbPositionsFile",
                                            "ENB Positions Specified or not",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_BuildingPositionsFile ("BuildingPositionsFile",
                                            "Building Positions Specified or not",
                                            ns3::StringValue (""),
                                            ns3::MakeStringChecker ());

static ns3::GlobalValue g_run_in_test_mode ("SimulationTestMode",
									"Run in test mode",
									ns3::BooleanValue (false),
									ns3::MakeBooleanChecker ());
TypeId
SimulationParameters::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SimulationParameters")
    .SetParent<Object> ()
    .SetGroupName ("SimulationParameters")
    .AddConstructor<SimulationParameters> ()
  ;
  return tid;
};


SimulationParameters::SimulationParameters() : m_numEnb(1), m_numUes(2),
		m_ue_rand_walk_min_max_speed(""),m_ue_positions_file(""), m_enb_positions_file(""), m_building_positions_file(""),
		m_use_buildings(false),
		dlUdp(DL_UDP_LOCAL_PORT, DL_UDP_REMOTE_HOST_PORT, DL_UDP_INTERPACKET_INTERVAL, DL_UDP_MAX_PACKET_COUNT,
				DL_UDP_SERVER_START_TIME, DL_UDP_CLIENT_START_TIME),
		ulUdp(UL_UDP_LOCAL_PORT, UL_UDP_REMOTE_HOST_PORT, UL_UDP_INTERPACKET_INTERVAL, UL_UDP_MAX_PACKET_COUNT,
				UL_UDP_SERVER_START_TIME, UL_UDP_CLIENT_START_TIME),
		verbose(0), test(false) {
	DoubleValue double_value;
	BooleanValue boolean_value;
	GlobalValue::GetValueByName ("SimulationTestMode", boolean_value);
	test = boolean_value.Get ();
	GlobalValue::GetValueByName ("SimulationRunDlTest", boolean_value);
	runDlUdpApp = boolean_value.Get ();
	GlobalValue::GetValueByName ("SimulationRunUlTest", boolean_value);
	runUlUdpApp = boolean_value.Get ();
	GlobalValue::GetValueByName ("SimulationTimeSec", double_value);
	GlobalValue::GetValueByName ("PrintDefaultParams", boolean_value);
	print_defaults_ = boolean_value.Get ();
	GlobalValue::GetValueByName ("PrintLogs", boolean_value);
	print_logs_ = boolean_value.Get ();
	GlobalValue::GetValueByName ("PrintLteTraces", boolean_value);
	print_traces_ = boolean_value.Get ();
	double sim_time_secs = double_value.Get ();
	dlUdp.set_simu_stop_time(sim_time_secs);
	ulUdp.set_simu_stop_time(sim_time_secs);
};
static uint32_t simbuffer_len = 10000;

/**
 * Flow Monitor File Name
 */
std::string SimulationParameters::getLogXmlFilename() {
	time_t rawtime;
	struct tm* timeinfo;
	char buffer[80];
	bzero(buffer, 80);
	char simbuffer[simbuffer_len];
	bzero(simbuffer, simbuffer_len);
	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S%Z", timeinfo);
	snprintf(simbuffer, simbuffer_len, "vruSimLog%s_eNB%d_%.0fm_UE%d_%d.xml", buffer, m_numEnb,
			enbCoverageRadiusm, m_numUes, transMode);

	std::string filename(simbuffer);

	return (filename);
}


/**
 * Simulation Config File Name
 */
std::string SimulationParameters::getConfigXmlFilename() {
	time_t rawtime;
	struct tm* timeinfo;
	char buffer[80];
	char simbuffer[simbuffer_len];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S%Z", timeinfo);
	snprintf(simbuffer, simbuffer_len, "vruSimConfig%s_eNB%d_%.0fm_UE%d_%d.xml", buffer, m_numEnb,
			enbCoverageRadiusm, m_numUes, transMode);

	std::string filename(simbuffer);

	return (filename);
}


/**
 * Simulation Run Update
 */
void SimulationParameters::simRunUpdates(Ptr<SimulationParameters> params) {
	time_t rawtime;
	struct tm* timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S %Z", timeinfo);
	std::string str(buffer);

	if (! params->test_mode()) {
		std::cout << str << ": " << Simulator::Now().GetSeconds() << "/"
			<< params->SimulationTimeSec << " seconds completed" << std::endl;
	}

	Simulator::Schedule(Seconds(params->simMsgIntervalSec), &simRunUpdates,
			params);
}

template <class T>
bool SimulationParameters::getInitialVal(std::string& tidName, std::string paramName, T& value, uint32_t verbose) {
  	TypeId tid;
 	if (TypeId::LookupByNameFailSafe (tidName, &tid)) {
		for (uint32_t j = 0; j < tid.GetAttributeN (); j++) {
			struct TypeId::AttributeInformation info = tid.GetAttribute (j);
			if (info.name == paramName) {
				Ptr<const AttributeValue> initialValue = info.initialValue;
				std::istringstream iss;
  			iss.str (initialValue->SerializeToString(info.checker));
  			iss >> (value);
				if (verbose) {
					std::cout << "*** Got Val of " << tidName << ":" << paramName << " = " << value << std::endl;
				}
				return true;
			}
		}
	}
	return false;
}
/*
 * Configure Simulation Parameter
 */
void SimulationParameters::configureSimulationParams(int argc, char *argv[], CommandLine cmd)
{

	DoubleValue double_value;
	IntegerValue integerValue;
	StringValue strValue;

	GlobalValue::GetValueByName ("SimulationTimeSec", double_value);
	double sim_time_secs = double_value.Get ();

	GlobalValue::GetValueByName ("EnbCoverageRadiusM", double_value);
	enbCoverageRadiusm = double_value.Get ();

	GlobalValue::GetValueByName ("UeRandWalkMinMaxSpeed", strValue);
	m_ue_rand_walk_min_max_speed = strValue.Get ();

	GlobalValue::GetValueByName ("UePositionsFile", strValue);
	m_ue_positions_file = strValue.Get ();

	GlobalValue::GetValueByName ("EnbPositionsFile", strValue);
	m_enb_positions_file = strValue.Get ();


	GlobalValue::GetValueByName ("BuildingPositionsFile", strValue);
	m_building_positions_file = strValue.Get ();

	cmd.Usage ("lte [--verbose=[1|2]] [--sim_time=<double>] [--use_buildings=true|false] [--test");
  	cmd.AddValue ("verbose", "turn on log components", verbose);
	cmd.AddValue ("sim_time", "Total simulation time in seconds", sim_time_secs);
	cmd.AddValue ("use_buildings", "Add builings from building_positions.txt", m_use_buildings);
  	cmd.AddValue ("test", "turn on log components", test);

	/* Following is to override tilt values of all antennas if provided from command line */

	cmd.Parse (argc, argv);

	SimulationTimeSec = sim_time_secs;
	simMsgIntervalSec = 1.0;
	
	//
	transMode = 2;		// *0=SISO 1=SISO *2=TxDiversity 3=MIMO_Spatial_Multiplexity OL 4=MIMO_Spatial_Multplexing_CL

};

/**
 * Get eNB UE position allocator
 */
uint32_t SimulationParameters::getUePositionAlloc(Ptr<ListPositionAllocator>& uePositionAlloc, NodeContainer enbNodes) {
	//Ues for cell edge
	uint8_t numUeCellEdge = (m_numUes > 3) ? 2 : 0;
	double enbCoverageEdge = enbCoverageRadiusm * 2;

	// Random number generator: seed?
	Ptr<UniformRandomVariable> randvar = CreateObject<UniformRandomVariable>();
	uint32_t numUes = 0;
	// Each eNB
	for (NodeContainer::Iterator enbi = enbNodes.Begin();
			enbi != enbNodes.End(); ++enbi) {

		Ptr<MobilityModel> mobility = (*enbi)->GetObject<MobilityModel>();
		Vector position = mobility->GetPosition();
		for (uint16_t uei = 0; uei < m_numUes-numUeCellEdge; uei++) {
			double x_ = position.x + (enbCoverageRadiusm/2) * cos(randvar->GetValue() * 2 * M_PI);
			double y_ = position.y + (enbCoverageRadiusm/2) * sin(randvar->GetValue() * 2 * M_PI);
			uePositionAlloc->Add(Vector(x_, y_, 0));
			numUes++;
		}
		for (uint16_t uei = 0; uei < numUeCellEdge; uei++) {
			double x_ = position.x + (enbCoverageEdge/2) * cos(randvar->GetValue() * 2 * M_PI);
			double y_ = position.y + (enbCoverageEdge/2) * sin(randvar->GetValue() * 2 * M_PI);
					uePositionAlloc->Add(Vector(x_, y_, 0));
			numUes++;
		}
	}
	return numUes;
}



}
