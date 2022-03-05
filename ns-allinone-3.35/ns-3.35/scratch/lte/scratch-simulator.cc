/**
 * VRU SIMULATION
 *
 * To identify stack/scheduler system limitation with high no of UEs
 *
 * Nuraj Pradhan & Sanjay Jaiman
 */

#include "SimuExe.h"
#include "RunTest.h"

using namespace ns3;

// #define CONNECT_SINGLE_ENB

// Logging keyword
NS_LOG_COMPONENT_DEFINE ("vru_simulation.cc");

static std::string loadfile = "scratch/lte/input-defaults.txt";

int main(int argc, char *argv[]) {
	std::cout << std::endl << "VRU Simulation" << std::endl;
	
	//---------------------------------------------------------------------------
	// 1. Read from config file
	
	Config::SetDefault ("ns3::ConfigStore::Filename", StringValue (loadfile));
	Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Load"));
	Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
//    Config::SetDefault ("ns3::UrbanMacroCellPropagationLossModel::LosEnabled", BooleanValue (true));
//    Config::SetDefault ("ns3::Hybrid3gppPropagationLossModel::ShadowingEnabled", BooleanValue (false));

	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults();
	inputConfig.ConfigureAttributes();
	
	//---------------------------------------------------------------------------
	// 2. Overlay commandline params
   	Ptr<SimulationParameters> sim_params = CreateObject<SimulationParameters>();
	CommandLine cmd (__FILE__);
	sim_params->configureSimulationParams(argc, argv, cmd);
	bool test_mode =  sim_params->test_mode();

	if (test_mode) {
//		sim_params->verbose = 0;
		StringValue strValue;
		GlobalValue::GetValueByName ("SimulationTestOutputFile", strValue);
		RunTest test_class(sim_params, strValue.Get ());
		test_class.run();
	}
	else {
		uint32_t verbosity = sim_params->verbose;
		if (verbosity >= 1) {
			std::cout << "\nInit params::" << std::endl;
			sim_params->print();
		}
		Ptr<UeConfig> ue_cfg = CreateObject<UeConfig>();

		std::string ue_pos_file = sim_params->ue_pos_file();
		if (ue_pos_file == "") {
			std::cout << "UE file not specified" << std::endl;
			exit(0);
		}

		ue_cfg->add_ues(ue_pos_file);
		ue_cfg->init();
#ifdef CONNECT_SINGLE_ENB
		SimuExe sim_exe(sim_params, ue_cfg, true);
#else
		SimuExe sim_exe(sim_params, ue_cfg);
#endif
		sim_exe.run();
	}
}


