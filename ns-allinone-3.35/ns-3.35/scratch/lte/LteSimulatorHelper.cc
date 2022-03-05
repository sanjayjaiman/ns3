/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011,12 CTTC
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
 * Author: Sanjay Jaiman
 */

#include "LteSimulatorHelper.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteSimulatorHelper");

NS_OBJECT_ENSURE_REGISTERED (LteSimulatorHelper);


#define PRINT_LOG_MSG_CONDITIONALLY(os) {\
  if (sim_params->test_mode()) \
   NS_LOG_DEBUG(os.str()); \
  else \
  	std::cout << os.str() << std::endl; \
  }

TypeId 
UeEnbIdCfg::GetTypeId (void) {
  static TypeId tid = TypeId ("ns3::UeEnbIdCfg")
    .SetParent<Object> ()
    .SetGroupName ("Mobility")
  ;
  return tid;
}

void put_enb_id(Ptr<Node> object, uint32_t enb_id) {
  Ptr<UeEnbIdCfg> ue_enb_id = object->GetObject<UeEnbIdCfg> ();
//	std::cout << "-- UE" << object->GetId() << " wil attach to ENB" << enb_id << std::endl;
    if (ue_enb_id == 0) {
      ue_enb_id = CreateObject<UeEnbIdCfg>();
      ue_enb_id->set_enb_id(enb_id);
      object->AggregateObject (ue_enb_id);   // ---- SANJAY:: Pass ENB ID to the Node for later retrieval
  }
}


TypeId
LteSimulatorHelper::GetTypeId () {
  static TypeId tid = TypeId ("ns3::LteSimulatorHelper")
    .SetParent<Object> ()
    .SetGroupName ("LteSimulator")
    .AddConstructor<LteSimulatorHelper> ()
    .AddAttribute ("PropagationLossModel",
                   "Propagation Loss Model",
                   StringValue (""),
                   MakeStringAccessor (&LteSimulatorHelper::SetPropagationLossModel,
                                       &LteSimulatorHelper::GetPropagationLossModel),
                   MakeStringChecker ())
    .AddAttribute ("AntennaModel",
                   "Antenna Model",
                   StringValue (""),
                   MakeStringAccessor (&LteSimulatorHelper::SetAntennaModel,
                                       &LteSimulatorHelper::GetAntennaModel),
                   MakeStringChecker ())
    .AddAttribute ("BuildingsPropagationLossModel",
                   "Propagation Loss Model for buildings",
                   StringValue (""),
                   MakeStringAccessor (&LteSimulatorHelper::SetBuildingsPropagationLossModel,
                                       &LteSimulatorHelper::GetBuildingsPropagationLossModel),
                   MakeStringChecker ())
    .AddAttribute ("FadingLossModel",
                   "Fading Loss Model",
                   StringValue (""),
                   MakeStringAccessor (&LteSimulatorHelper::SetFadingLossModel,
                                       &LteSimulatorHelper::GetFadingLossModel),
                   MakeStringChecker ())
    .AddAttribute ("SpectrumChannelModel",
                   "Spectrum Channel Model",
                   StringValue (""),
                   MakeStringAccessor (&LteSimulatorHelper::SetSpectrumChannelModel,
                                       &LteSimulatorHelper::GetSpectrumChannelModel),
                   MakeStringChecker ())
	.AddAttribute ("TestCellBringDown",
                   "Test Cell Bring Down",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteSimulatorHelper::m_test_cell_bring_down),
                   MakeBooleanChecker ())
    .AddAttribute ("TestPropagationLoss",
                   "",
                   BooleanValue (false),
                   MakeBooleanAccessor (&LteSimulatorHelper::m_test_propagation_loss),
                   MakeBooleanChecker ())
  ;
  return tid;
};

void
LteSimulatorHelper::SetPropagationLossModel (std::string type) {
  NS_LOG_FUNCTION (this << type);
  m_popagationLossModel = type;
}

std::string
LteSimulatorHelper::GetPropagationLossModel () const {
  return m_popagationLossModel;
}

void
LteSimulatorHelper::SetAntennaModel (std::string type) {
  NS_LOG_FUNCTION (this << type);
  m_antennaModel = type;
}

std::string
LteSimulatorHelper::GetAntennaModel () const {
  return m_antennaModel;
}


void
LteSimulatorHelper::SetBuildingsPropagationLossModel (std::string type) {
  NS_LOG_FUNCTION (this << type);
  m_buildingsPopagationLossModel = type;
}

std::string
LteSimulatorHelper::GetBuildingsPropagationLossModel () const {
  return m_buildingsPopagationLossModel;
}

void
LteSimulatorHelper::SetFadingLossModel (std::string type) {
  NS_LOG_FUNCTION (this << type);
  m_fading_loss_model = type;
}

std::string
LteSimulatorHelper::GetFadingLossModel () const {
  return m_fading_loss_model;
}

void
LteSimulatorHelper::SetSpectrumChannelModel (std::string type) {
  NS_LOG_FUNCTION (this << type);
  m_spectrum_channel_model = type;
}

std::string
LteSimulatorHelper::GetSpectrumChannelModel () const {
  return m_spectrum_channel_model;
}

TypeId
LteSimulatorHelper::get_path_loss_model() {
	if (sim_params->use_buildings()) {
		return TypeId::LookupByName (m_buildingsPopagationLossModel);
	}
	return TypeId::LookupByName (m_popagationLossModel);
}

void
LteSimulatorHelper::set_path_loss_model() {
	m_lteHelper->SetPathlossModelType(get_path_loss_model());
}

template <class myType>
void LteSimulatorHelper::print_attr (std::string params[], uint32_t size, std::ostringstream& os) {
	myType st_value;
	for (uint32_t i = 0; i < size; i++) {
		std::cout << "\t" << params[i] << ":" << std::endl;
		m_lteHelper->GetAttribute(params[i], st_value);
		os << "\t" << params[i] << " = " << st_value.Get() << std::endl;
		std::cout << "\t" << st_value.Get() << std::endl;
	}
}

void LteSimulatorHelper::print_lte_helper_values() {
	std::string helper_string_params[] = {
		"Scheduler", "FfrAlgorithm", "FadingModel", "EnbComponentCarrierManager", "UeComponentCarrierManager",
		"NumberOfComponentCarriers",
	};
	std::string helper_tid_params[] = { 
		"PathlossModel", 
	};
	std::string helper_bool_params[] = {"UseIdealRrc", "AnrEnabled", "UsePdschForCqiGeneration", "UseCa"};
	//, "DisableEnbPhy"
	uint32_t siz_str_params = sizeof(helper_string_params) / sizeof(std::string);
	uint32_t siz_bool_params = sizeof(helper_bool_params) / sizeof(std::string);
	uint32_t siz_tid_params = sizeof(helper_tid_params) / sizeof(std::string);

	std::ostringstream os;
	os << "LTE Helper params\n";
	print_attr<StringValue>(helper_string_params, siz_str_params, os);
	print_attr<BooleanValue>(helper_bool_params, siz_bool_params, os);
//	print_attr<IntegerValue>(helper_int_params, siz_int_params, os);
	TypeIdValue type_value;
	for (uint32_t i = 0; i < siz_tid_params; i++) {
		m_lteHelper->GetAttribute(helper_tid_params[i], type_value);
		TypeId tid = type_value.Get();
		os << "\t" <<helper_tid_params[i] << " = " <<  tid.GetName() << std::endl;
	}

	std::cout << os.str() << std::endl;
};

void LteSimulatorHelper::set_fading_spectrum_and_scheduler () {
//	m_lteHelper->SetSpectrumChannelType ("ns3::MultiModelSpectrumChannel");
	m_lteHelper->SetSpectrumChannelType (m_spectrum_channel_model);

//	m_lteHelper->SetAttribute ("FadingModel", StringValue ("ns3::TraceFadingLossModel"));
	m_lteHelper->SetAttribute ("FadingModel", StringValue (m_fading_loss_model));
	std::ifstream ifTraceFile;
	ifTraceFile.open ("../../src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad", std::ifstream::in);
	if (ifTraceFile.good ())  {
	    // script launched by test.py
	    m_lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("../../src/lte/model/fading-traces/fading_trace_EVA_60kmph.fad"));
	}
	else {
	    // script launched as an example
	    m_lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_EVA_60kmph.fad"));
	}

	m_lteHelper->SetFadingModelAttribute ("TraceLength", TimeValue (Seconds (10.0)));
	m_lteHelper->SetFadingModelAttribute ("SamplesNum", UintegerValue (10000));
	m_lteHelper->SetFadingModelAttribute ("WindowSize", TimeValue (Seconds (0.5)));
	m_lteHelper->SetFadingModelAttribute ("RbNum", UintegerValue (100));
	m_lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");
};

void  LteSimulatorHelper::add_buildings() {
//	SetPropagationLossModel("ns3::ThreeGppUmaPropagationLossModel");
	SetPropagationLossModel(m_buildingsPopagationLossModel);
	buildingPositionAlloc = CreateObject<BuildingPositionAllocator>();
    m_lteHelper->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0));
    m_lteHelper->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (1));
    m_lteHelper->SetPathlossModelAttribute ("ShadowSigmaIndoor", DoubleValue (1.5));

		// use always LOS model
//		m_lteHelper->SetPathlossModelAttribute ("Los2NlosThr", DoubleValue (1e6));
	std::string building_pos_file = sim_params->building_pos_file();
	if (building_pos_file != "") {
		buildingPositionAlloc->Add(building_pos_file);
	}
	else {
		std::cout << "Buildings file not specified" << std::endl;
		exit(0);
	}

//		for (uint32_t j = 0; j < all_ue_nodes.GetN(); j++) {
//				Ptr<Node> ueNode = all_ue_nodes.Get(j);
//				Ptr<MobilityModel> a = ueNode->GetObject<MobilityModel> ();
//				if (a != nullptr) {
//					std::cout << "--- A found" << std::endl;
//				}
//      			Ptr<MobilityBuildingInfo> buildingInfoA = ueNode->GetObject<MobilityBuildingInfo> ();
//				if (a != nullptr) {
//					std::cout << "--- buildingInfoA found; Num floors = " << buildingInfoA->GetFloorNumber() << std::endl;
//				}
//      			buildingInfoA->MakeConsistent (a);
//		}

}

void  LteSimulatorHelper::print_buildings_info_comma_seperated(std::ostringstream& os) {
	if (buildingPositionAlloc == nullptr)
		return;
//	os << "Propagation model," << m_buildingsPopagationLossModel << std::endl;
	os << "Num Buildings, " << buildingPositionAlloc->GetSize() << std::endl;
	buildingPositionAlloc->print(os, ",");
}

void  LteSimulatorHelper::print_buildings_info(std::ostringstream& os) {
	if (buildingPositionAlloc == nullptr)
		return;
//	os << "Propagation model = " << m_buildingsPopagationLossModel << std::endl;
	os << "Building file = " << sim_params->building_pos_file() << std::endl;
	os << "Num Buildings = " << buildingPositionAlloc->GetSize() << std::endl;
	buildingPositionAlloc->print(os);
	NS_ASSERT_MSG(buildingPositionAlloc->GetSize() == BuildingList::GetNBuildings(), "Num buldings not correct in BuildingList");
//	std::stringstream ss;
//	ss << "***** One more check - Num Buildings = " << BuildingList::GetNBuildings() << std::endl;
//	uint32_t i = 0;
//	for (BuildingList::Iterator iter = BuildingList::Begin(); iter != BuildingList::End(); iter++) {
//		Ptr<Building> bld_ =  BuildingList::GetBuilding (i);
//		ss << "Building " << i << "::" << std::endl;
//		ns3::Box box_ = bld_->GetBoundaries();
//		ss << "\t(xmin, xmax, ymin, ymax) = (" << box_.xMin << ", " << box_.xMax << ", " << box_.yMin << ", " << box_.yMax << ")" << std::endl;
//		i++;
//	}
//	std::cout << ss.str();
}


// does not work for this situation yet. Do not use
void LteSimulatorHelper::testPropagationLoss (Ptr<MobilityModel>& mmEnb, NodeContainer& ueNodes) {
	std::ofstream outFile;
	Ptr<Hybrid3gppPropagationLossModel> propagationLossModel = CreateObject<Hybrid3gppPropagationLossModel> ();
	propagationLossModel->SetAttribute ("ShadowSigmaOutdoor", DoubleValue (1));
	propagationLossModel->SetAttribute ("ShadowSigmaIndoor", DoubleValue (1.5));
	propagationLossModel->SetAttribute ("ShadowSigmaExtWalls", DoubleValue (0.0));
	propagationLossModel->SetAttribute ("ShadowingEnabled", BooleanValue (true));

	outFile.open ("buildings-shadowing-profiler.out");
	if (!outFile.is_open ()) {
		NS_FATAL_ERROR ("Can't open output file");
	}
	uint32_t i = 0;
	for (NodeContainer::Iterator nodei = ueNodes.Begin();
			nodei != ueNodes.End(); nodei++) {
		Ptr<MobilityModel> mmUe = (*nodei)->GetObject<MobilityModel>();
    	double loss = propagationLossModel->GetLoss (mmEnb, mmUe);
		std::cout << "***** " << i << "\tLoss = " << loss << std::endl;
      	outFile << i++ << "\t" << loss << std::endl;
    }
}


//////////////////////////////////////////////////////////////////////////////////

/**
 *  Log Eanble
 */
void LteSimulatorHelper::LogEnable(bool verbose) {
	LogLevel logLevel = (LogLevel)(LOG_ERROR);
	if (verbose) {
		logLevel = (LogLevel)(LOG_PREFIX_ALL | LOG_LEVEL_INFO);
		LogComponentEnableAll(logLevel);
	}

//	
//	LogLevel logLevel = (LogLevel)(LOG_LEVEL_ERROR);
//	
//	LogComponentEnable ("pw_PfFfMacScheduler", logLevel);
//	LogComponentEnable ("LteUePhy", logLevel);
	//LogComponentEnable ("pw_PfFfMacScheduler", logLevel);
	//LogComponentEnable ("LteUePowerControl", logLevel);
	//LogComponentEnable ("LteUeMac", logLevel);

//	LogComponentEnable ("LteEnbPhy", logLevel);
//	LogComponentEnable ("EpcHelper", logLevel);
//	LogComponentEnable ("EpcEnbApplication", logLevel);
//	LogComponentEnable ("EpcX2", logLevel);
//	LogComponentEnable ("EpcSgwPgwApplication", logLevel);
//
//	LogComponentEnable ("LteEnbRrc", logLevel);
//	LogComponentEnable ("LteEnbNetDevice", logLevel);
//	LogComponentEnable ("LteUeRrc", logLevel);
//	LogComponentEnable ("LteUeNetDevice", logLevel);
//	LogComponentEnable ("A2A4RsrqHandoverAlgorithm", logLevel);
//	LogComponentEnable ("A3RsrpHandoverAlgorithm", logLevel);
}

uint32_t LteSimulatorHelper::get_ul_earfcn(uint32_t dl_earfcn) {
	typedef struct dl_conv_struct {
		uint32_t dl_earfcn;
		uint32_t ul_earfcn;
	} dl_conv_struct_t;

	dl_conv_struct dl_conv_table[] = {
		{300, 18300},
		{6400, 24400},
		{9585, 27585},
	};

	for (uint32_t i = 0; i < sizeof(dl_conv_table) / sizeof(dl_conv_struct_t); i++) {
		if (dl_conv_table[i].dl_earfcn == dl_earfcn) {
			return dl_conv_table[i].ul_earfcn;
		}
	}
	return 0;
}

void LteSimulatorHelper::enb_set_power(NetDeviceContainer& enbDev, uint32_t i, double tx_pwr, uint32_t verbose) {
	for (NetDeviceContainer::Iterator iter = enbDev.Begin(); iter != enbDev.End(); iter++) {
		Ptr<LteEnbNetDevice> enb_net_dev_p = ns3::DynamicCast<LteEnbNetDevice>(*iter);
		Ptr<LteEnbPhy> phy_ = enb_net_dev_p->GetPhy();
		phy_->SetTxPower(tx_pwr);
		std::cout << "****** ENB" << i << " Set Tx POWER : " << tx_pwr << std::endl;
	}
	if (verbose >= 1) {
		for (NetDeviceContainer::Iterator iter = enbDev.Begin(); iter != enbDev.End(); iter++) {
			Ptr<LteEnbNetDevice> enb_net_dev_p = ns3::DynamicCast<LteEnbNetDevice>(*iter);
			Ptr<LteEnbPhy> phy_ = enb_net_dev_p->GetPhy();
			double tx_pwr = phy_->GetTxPower();
			std::cout << "\tTx POWER = " << tx_pwr << std::endl;
		}
	}
}

void LteSimulatorHelper::add_enb_positions(Ptr<ListPositionAllocator>& enbPositionAlloc, Ptr<LteSimulatorHelper>& simu_helper_, uint32_t numEnb) {
	//	enbPositionAlloc->Add(Vector(0.0, 0.0, 0.0));
	uint32_t distance = 100;
	UintegerValue enb_height;
	std::string tidName = "ns3::LteSimulatorHelper";
	uint32_t ht_ = 5;

	switch (numEnb) {
		case 1:
// This way of accessing the default values also works
//			simu_helper_->GetAttribute("Enb1Height", enb_height);
//			enbPositionAlloc->Add (Vector (0.0,  0.0, enb_height.Get()));
//			std::cout << "**** VAL from GetAttribute  Enb1Height = " << enb_height.Get() << std::endl;
			enbPositionAlloc->Add (Vector (0.0, 0.0, ht_));                       // eNB1
			break;
		case 2:             
			enbPositionAlloc->Add (Vector (distance,  0.0, ht_));                 // eNB2
			break;
		case 3:           
			enbPositionAlloc->Add (Vector (distance * 0.5, distance * 0.866, ht_));  // eNB3
			break;
		default:
			std::cout << "Unsupported number of enbs (" << numEnb << ")" << std::endl;
			exit(0);
	}
}

/*
bool LteSimulatorHelper::get_initial_value(TypeId tid, std::string paramName, std::string& type, std::string& value) {
	std::string tidName = tid.GetName();
	std::ostringstream os;
	for (uint32_t j = 0; j < tid.GetAttributeN (); j++) {
		struct TypeId::AttributeInformation info = tid.GetAttribute (j);
		Ptr<const AttributeValue> initialValue = info.initialValue;
		Ptr<const AttributeAccessor> accessor = info.accessor;
		Ptr<AttributeValue> v = info.checker->Create ();
//		os << "----" << info.name << "\n";
		if (info.checker->HasUnderlyingTypeInformation ()) {
//        type = info.checker->GetUnderlyingTypeInformation ();
			type = info.checker->GetValueTypeName();
		}
//		if (info.name == paramName && info.accessor->HasGetter()) {
		if (info.name == paramName) {
			value = initialValue->SerializeToString(info.checker);
//			std::cout << "**** " << type << "; " << value << std::endl;
			return true;
		}
	}
	return false;
}
*/

bool LteSimulatorHelper::get_initial_value(TypeId tid, std::string paramName, std::string& type, std::string& value) {
	std::string tidName = tid.GetName();
	std::ostringstream os;
	struct TypeId::AttributeInformation info;
	if (tid.LookupAttributeByName (paramName, &info)) {
		Ptr<const AttributeValue> initialValue = info.initialValue;
		Ptr<const AttributeAccessor> accessor = info.accessor;
		Ptr<AttributeValue> v = info.checker->Create ();
		if (info.checker->HasUnderlyingTypeInformation ()) {
			type = info.checker->GetValueTypeName();
		}
		value = initialValue->SerializeToString(info.checker);
		return true;
	}
	return false;
}

void LteSimulatorHelper::print_helper_defaults(TypeId tid, std::ostringstream& os) {
	std::string tidName = tid.GetName();
	
	
	for (uint32_t j = 0; j < tid.GetAttributeN (); j++) {
		std::string type, value;
		struct TypeId::AttributeInformation info = tid.GetAttribute (j);
		if (get_initial_value(tid, info.name, type, value)) {
			os << "\t" << tidName << "::" << info.name << "(" << type << ") = " << value << std::endl;
		}
		else {
			os << "\t" << tidName << "::" << info.name << "(" << type << ") NOT FOUND "<< std::endl;
		}
	}
}

void LteSimulatorHelper::verify_init_values(uint32_t verbose, bool print_defaults) {

	std::ostringstream os;

	ObjAndParam_t objs_[] = {
		{"ns3::Parabolic3dAntennaModel", "HorizontalBeamwidth"},
		{"ns3::Parabolic3dAntennaModel", "Orientation"},
		{"ns3::Parabolic3dAntennaModel", "MaxHorizontalAttenuation"},
		{"ns3::Parabolic3dAntennaModel", "VerticalBeamwidth"},
		{"ns3::Parabolic3dAntennaModel", "ElectricalTilt"},
		{"ns3::Parabolic3dAntennaModel", "MechanicalTilt"},
		{"ns3::Parabolic3dAntennaModel", "MaxVerticalAttenuation"},
		{"ns3::Parabolic3dAntennaModel", "BoresightGain"},
		{"ns3::LteEnbRrc", "SrsPeriodicity"},
		{"ns3::pw_PfFfMacScheduler", "EfficiencyState"},
		{"ns3::LteSimulatorHelper", "PropagationLossModel"},
		{"ns3::LteSimulatorHelper", "BuildingsPropagationLossModel"},
		{"ns3::LteSimulatorHelper", "FadingLossModel"},
		{"ns3::LteSimulatorHelper", "SpectrumChannelModel"},
		{"ns3::LteSimulatorHelper", "TestCellBringDown"},
	};
	if (print_defaults) {
		os << "*** Deafult params ***" << std::endl;
		for (uint32_t i = 0; i < sizeof(objs_) / sizeof(ObjAndParam_t); i++) {
			std::string tidName = objs_[i].obj;
			std::string paramName = objs_[i].param;
	//		LteSimulatorHelper::verify_value(os, tidName, paramName);

			TypeId tid;
			if (! TypeId::LookupByNameFailSafe (tidName, &tid)) {
				os << "\t" << tidName << ":: NOT Found" << std::endl;
				continue;
			}
			std::string type, value;
			os << "\t" << tidName << "::" << paramName;
			if (get_initial_value(tid, paramName, type, value)) {
				os << "(" << type << ") = " << value << std::endl;
			}
			else {
				os << "\t" << tidName << "::" << paramName <<" NOT Found" << std::endl;
			}
		}
		os << "\n";
		//	TypeIdValue ti_value =   TypeIdValue(TypeId::LookupByName ("ns3::LteHelper"));
		//	TypeId tid = ti_value.Get();
		TypeId tid = TypeId::LookupByName ("ns3::LteHelper");
		std::string type;
		std::string value;
		if (LteSimulatorHelper::get_initial_value(tid, "PathlossModel", type, value)) {
			os << "\tPathlossModel (" << type << ") = " << value << std::endl;
		}
		std::cout << "*** print_helper_defaults ***" << std::endl;
		LteSimulatorHelper::print_helper_defaults(tid, os);

		if (verbose) {
			std::cout << os.str();
		}
	}
}

void LteSimulatorHelper::add_to_apps(Ptr<Node>& local_node, Ptr<Node>& remote_node,
				ApplicationContainer& client_apps, ApplicationContainer& server_apps,
				uint32_t local_port, uint32_t remote_port, Time& inter_packet_interval, uint32_t max_packet_count, std::ostringstream& os) {
	Ptr<Ipv4> local_ip = local_node->GetObject<Ipv4>();
	Ptr<Ipv4> remote_ip = remote_node->GetObject<Ipv4>();
	NS_ASSERT(local_ip != nullptr);
	NS_ASSERT(remote_ip != nullptr);

	Ipv4Address remote_host_addr = (remote_ip->GetAddress(1, 0)).GetLocal();
	Ipv4Address local_host_addr = (local_ip->GetAddress(1, 0)).GetLocal();
	os << "\tLocal addr/port: ";
	local_host_addr.Print(os); 
	os << "/" << local_port << "\n";
	os << "\tRemote host/port: ";
	remote_host_addr.Print(os); 
	os << "/" << remote_port  << "\n";
	os << "\tInter Packet Interval: " << inter_packet_interval << " \n";
	os << "\tMax Packet Count: " <<  max_packet_count << " \n";
	// UDP Server
	// DO WE NEED PacketSinkHelper??
	PacketSinkHelper packet_sink_helper("ns3::UdpSocketFactory",
			InetSocketAddress(Ipv4Address::GetAny(), local_port));
	server_apps.Add(packet_sink_helper.Install(local_node));
	// UDP Client
	UdpClientHelper udp_client_helper(remote_host_addr, remote_port);
	udp_client_helper.SetAttribute("Interval", TimeValue(inter_packet_interval));
	udp_client_helper.SetAttribute("MaxPackets", UintegerValue(max_packet_count));
	client_apps.Add(udp_client_helper.Install(local_node));
}

NodeContainer LteSimulatorHelper::get_ue_nodes() {
	NodeContainer net_;
	net_.Add(ue_nodes_container_map[ACTIVE_CONSTANT_POS]);
	net_.Add(ue_nodes_container_map[IDLE_CONSTANT_POS]);
	net_.Add(ue_nodes_container_map[ACTIVE_RAND_WALK]);
	net_.Add(ue_nodes_container_map[IDLE_RAND_WALK]);
	net_.Add(ue_nodes_container_map[ACTIVE_CONST_VEL]);
	net_.Add(ue_nodes_container_map[IDLE_CONST_VEL]);
    return net_;
};
NodeContainer LteSimulatorHelper::get_active_ue_nodes() {
	return NodeContainer(ue_nodes_container_map[ACTIVE_CONSTANT_POS], ue_nodes_container_map[ACTIVE_RAND_WALK], ue_nodes_container_map[ACTIVE_CONST_VEL]);
};
NodeContainer LteSimulatorHelper::get_idle_ue_nodes() {
	return NodeContainer(ue_nodes_container_map[IDLE_CONSTANT_POS], ue_nodes_container_map[IDLE_RAND_WALK], ue_nodes_container_map[IDLE_CONST_VEL]);
};
void
LteSimulatorHelper::printUeToEnbID (NetDeviceContainer devices, NodeContainer c, std::ostringstream& os)
{
  NS_LOG_FUNCTION (this);
  os << "UE to ENB attach mappings:" << std::endl;
  for (NetDeviceContainer::Iterator iter = devices.Begin (); iter != devices.End (); ++iter) {
	Ptr<NetDevice> device = *iter;
	Ptr<Node> node = device->GetNode();
	Ptr<UeEnbIdCfg> enb_id_cfg = node->GetObject<UeEnbIdCfg> ();
	uint32_t enb_id = -1;
	uint32_t ue_id = node->GetId();
	if (enb_id_cfg != nullptr) {
		enb_id = enb_id_cfg->id();
	}
	if (c.Contains(ue_id)) {
		os << "\tUE" << ue_id << " attaches to ENB" << enb_id << std::endl;
	}
  }
}

NetDeviceContainer LteSimulatorHelper::get_ue_container_for_enbid(uint32_t enb_id) {
  	std::map<uint16_t, NetDeviceContainer>::iterator iter = m_enb_net_dev_container_map.find(enb_id);
	NS_ASSERT_MSG(iter != m_enb_net_dev_container_map.end(), "ENB" << enb_id << "not found in m_enb_net_dev_container_map");
	NetDeviceContainer container_ = iter->second;
//	std::cout << "Num ENBS = " << container_.GetN() << std::endl;
	NS_ASSERT_MSG(container_.GetN() != 0, "No ENB found in container");
	return container_;
}

void LteSimulatorHelper::find_ues_for_closest_enbid(Ptr<EnbConfig> enb_cfg, NodeContainer &ue_container,
			std::map<uint32_t, NodeContainer> &out, std::ostringstream& os) {
	std::vector<uint32_t> enb_ids = enb_cfg->get_indicies();
	std::map<uint32_t, Vector> enb_pos_map;
	for (std::vector<uint32_t>::iterator iter = enb_ids.begin(); iter != enb_ids.end(); iter++) {
		uint32_t enb_id = *iter;
		Vector   enb_pos_vec;
		if (enb_cfg->get_pos_vector(enb_id, enb_pos_vec)) {
			enb_pos_map[enb_id] = enb_pos_vec;
		}
	}
	os << "================================================================================\n";
	os << " Distance to ENBs::" << std::endl;
  	for (NodeContainer::Iterator iter = ue_container.Begin (); iter != ue_container.End (); ++iter)  {
		Ptr<Node> object = (*iter);
		Ptr<MobilityModel> mmUe = object->GetObject<MobilityModel>();
		Vector ue_pos = mmUe->GetPosition();
		os << "UE" << object->GetId() << " (" << ue_pos.x << "," << ue_pos.y << "," << ue_pos.z << "):" << std::endl;
		std::map<uint32_t, double> enb_distance_map;
		for (std::map<uint32_t, Vector>::iterator iter = enb_pos_map.begin(); iter != enb_pos_map.end(); iter++) {
			uint32_t enb_id = iter->first;
			Vector enb_pos = iter->second;
			double x_2 = pow(double(enb_pos.x - ue_pos.x), 2);
			double y_2 = pow(double(enb_pos.y - ue_pos.y), 2);
			enb_distance_map[enb_id] = sqrt(x_2 + y_2);
			os << "\tENB" << enb_id << " ("  << enb_pos.x << "," << enb_pos.y << "," << enb_pos.z << ") = " << enb_distance_map[enb_id] << std::endl;
		}
		uint32_t min_enb_id;
		double min_val;
		uint32_t i = 0;
		for (std::map<uint32_t, double>::iterator iter = enb_distance_map.begin(); iter != enb_distance_map.end(); iter++) {
			if (i == 0 || iter->second < min_val) {
				min_val = iter->second;
				min_enb_id = iter->first;
			}
			i++;
		}
		out[min_enb_id].Add(object);
		put_enb_id(object, min_enb_id);  // We are not using it but for every UE we can determine which ENB are going to attach to
  	}
}


uint32_t LteSimulatorHelper::sift_ues_for_enbid(uint32_t enb_id, NodeContainer &container, NodeContainer &out) {
  	for (NodeContainer::Iterator iter = container.Begin (); iter != container.End (); ++iter)  {
		Ptr<Node> object = (*iter);
		Ptr<UeEnbIdCfg> enb_id_cfg = object->GetObject<UeEnbIdCfg> ();
		if (enb_id_cfg->id() == enb_id) {
//			std::cout << "Found ENB ID = " << enb_id_cfg->id() << "; Adding node" << object->GetId() << std::endl;
			out.Add(object);
		}
  	}
	return out.GetN();
}


Ptr<LteEnbNetDevice> LteSimulatorHelper::get_ENB_net_device(uint32_t enb_id) {
	NetDeviceContainer container_ = get_ue_container_for_enbid(enb_id);
//	std::cout << "-- Found ENB Net dev for ENBID " << enb_id << std::endl;
	Ptr<NetDevice> net_dev = container_.Get(0);  // Only One ENB per container
	Ptr<LteEnbNetDevice> enb_dev_ptr = net_dev->GetObject<LteEnbNetDevice>();
	NS_ASSERT_MSG(enb_dev_ptr != nullptr, "No ENB container found");
	return enb_dev_ptr;
};


void LteSimulatorHelper::install_ue_nodes(Ptr<EnbConfig>& enb_cfg, Ptr<UeConfig>& ue_cfg, uint32_t verbosity) {
	std::stringstream os;

	Ptr<ListPositionAllocator> ue_const_position_active_alloc = ue_cfg->get_const_position_active_alloc();
	Ptr<ListPositionAllocator> ue_const_position_idle_alloc = ue_cfg->get_const_position_idle_alloc();
	Ptr<ListPositionAllocator> ue_rand_walk_active_alloc = ue_cfg->get_rand_walk_active_alloc();
	Ptr<ListPositionAllocator> ue_rand_walk_idle_alloc = ue_cfg->get_rand_walk_idle_alloc();
	Ptr<ListPositionAllocator> ue_constant_velocity_actve_alloc = ue_cfg->get_const_velocity_active_alloc();
	Ptr<ListPositionAllocator> ue_constant_velocity_idle_alloc = ue_cfg->get_const_velocity_idle_alloc();

	uint32_t num_const_pos_active_nodes = ue_cfg->get_num_const_pos_active_ues();
	uint32_t num_const_pos_idle_nodes = ue_cfg->get_num_const_pos_idle_ues();
	uint32_t num_rand_walk_active_nodes = ue_cfg->get_num_rand_walk_active_ues();
	uint32_t num_rand_walk_idle_nodes = ue_cfg->get_num_rand_walk_idle_ues();
	uint32_t num_const_velocity_active_nodes = ue_cfg->get_num_const_vel_active_ues();
	uint32_t num_const_velocity_idle_nodes = ue_cfg->get_num_const_vel_idle_ues();
  
/*
	uint32_t num_const_pos_active_nodes = ue_const_position_active_alloc->GetSize();
	uint32_t num_const_pos_idle_nodes = ue_const_position_idle_alloc->GetSize();
	uint32_t num_rand_walk_active_nodes = ue_rand_walk_active_alloc->GetSize();
	uint32_t num_rand_walk_idle_nodes = ue_rand_walk_idle_alloc->GetSize();
	uint32_t num_const_velocity_active_nodes = ue_constant_velocity_actve_alloc->GetSize();
	uint32_t num_const_velocity_idle_nodes = ue_constant_velocity_idle_alloc->GetSize();
*/
	os << "================================================================================\n";
	os << "Number of active UEs = " << num_const_pos_active_nodes + num_rand_walk_active_nodes + num_const_velocity_active_nodes
	<< "\n\t\'Const Pos active\' UEs = " << num_const_pos_active_nodes 
	<< "\n\t\'Randwalk active\' UEs = " << num_rand_walk_active_nodes 
	<< "\n\t\'Const velocity active\' UEs = " << num_const_velocity_active_nodes 
	<< "\nNumber of idle UEs =" << num_const_pos_idle_nodes + num_rand_walk_idle_nodes + num_const_velocity_idle_nodes
	<< "\n\t\'Const Idle\' UEs = " << num_const_pos_idle_nodes
	<< "\n\t\'Randwalk idle\' UEs = " << num_rand_walk_idle_nodes 
	<< "\n\t\'Const velocity idle\' UEs = " << num_const_velocity_idle_nodes 
	<< std::endl;

	const char* speed_str = sim_params->m_ue_rand_walk_min_max_speed.c_str();

	if (num_const_pos_active_nodes) {
		ue_nodes_container_map[ACTIVE_CONSTANT_POS].Create(num_const_pos_active_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[ACTIVE_CONSTANT_POS], ue_const_position_active_alloc, 
				enb_cfg, ue_cfg, CONST_POS_ACTIVE_MASK));
	}
	if (num_const_pos_idle_nodes) {
		ue_nodes_container_map[IDLE_CONSTANT_POS].Create(num_const_pos_idle_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[IDLE_CONSTANT_POS], ue_const_position_idle_alloc, 
				enb_cfg, ue_cfg, CONST_POS_IDLE_MASK));
	}
	if (num_rand_walk_active_nodes) {
		ue_nodes_container_map[ACTIVE_RAND_WALK].Create(num_rand_walk_active_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[ACTIVE_RAND_WALK], ue_rand_walk_active_alloc, 
				enb_cfg, ue_cfg, RAND_WALK_ACTIVE_MASK, speed_str));
	}
	if (num_rand_walk_idle_nodes) {
		ue_nodes_container_map[IDLE_RAND_WALK].Create(num_rand_walk_idle_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[IDLE_RAND_WALK], ue_rand_walk_idle_alloc, 
				enb_cfg, ue_cfg, RAND_WALK_IDLE_MASK, speed_str));
	}
	if (num_const_velocity_active_nodes) {
		ue_nodes_container_map[ACTIVE_CONST_VEL].Create(num_const_velocity_active_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[ACTIVE_CONST_VEL], ue_constant_velocity_actve_alloc, 
				enb_cfg, ue_cfg, CONST_VELOCITY_ACTIVE_MASK));
	}
	if (num_const_velocity_idle_nodes) {
		ue_nodes_container_map[IDLE_CONST_VEL].Create(num_const_velocity_idle_nodes);
		ue_mobility_helpers.push_back(UeMobilityHelper::UeMobilityFactory(ue_nodes_container_map[IDLE_CONST_VEL], ue_constant_velocity_idle_alloc, 
				enb_cfg, ue_cfg, CONST_VELOCITY_IDLE_MASK));
	}

	if (verbosity >= 1) {
		std::cout << os.str();
	}
}

void LteSimulatorHelper::NotifyConnectionReleaseCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
  NS_LOG_FUNCTION (this << context);
  std::ostringstream os;
//  if (imsi != 0) {
  	os << "Connection Released: IMSI = " << imsi << "; CellId = " << cellId << ";rnti = " << rnti;
//  }
  PRINT_LOG_MSG_CONDITIONALLY(os);
}

void LteSimulatorHelper::ConnectionEstablishedCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti) {
  NS_LOG_FUNCTION (this << context);
  std::ostringstream os;
  os << "Connection Established: IMSI = " << imsi << "; CellId = " << cellId << ";rnti = " << rnti;
  PRINT_LOG_MSG_CONDITIONALLY(os);
}

void
LteSimulatorHelper::HandoverStartCallback (std::string context, uint64_t imsi,
                                                  uint16_t sourceCellId, uint16_t rnti,
                                                  uint16_t targetCellId)
{
  NS_LOG_FUNCTION (this << context);
  std::ostringstream os;
  os << "IMSI = " << imsi << "; CellId = " << sourceCellId << ";rnti = " << rnti << ";targetCellId = " << targetCellId;
  os << "Handover started:\n\t" << os.str() << std::endl;
  PRINT_LOG_MSG_CONDITIONALLY(os);
  m_hasHandoverOccurred = true;
  m_enbHandoverStart = Simulator::Now ();
}

void
LteSimulatorHelper::EnbHandoverEndOkCallback (std::string context,
                                                    uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_ASSERT (m_enbHandoverStart > Seconds (0));
  Time delay = Simulator::Now () - m_enbHandoverStart;
  std::ostringstream os;
  os << "IMSI = " << imsi << ";CellId = " << cellid << ";rnti = " << rnti << "; eNodeB delay = " << delay.As (Time::S);
  os << "Handover Ended:\n\t" << os.str() << std::endl;
  PRINT_LOG_MSG_CONDITIONALLY(os);
}

void
LteSimulatorHelper::CellShutdownCallback () {
  NS_LOG_FUNCTION (this);
  std::map<uint16_t, NetDeviceContainer>::iterator iter = m_enb_net_dev_container_map.begin();
  for (; iter != m_enb_net_dev_container_map.end(); iter++) {
	uint16_t id = iter->first;
	NetDeviceContainer container_ = iter->second;
	Ptr<NetDevice> net_dev = container_.Get(0); // Only One ENB per container
	Ptr<LteEnbNetDevice> enb_dev = net_dev->GetObject<LteEnbNetDevice>();
	NS_ASSERT(enb_dev != nullptr);
	uint32_t cell_id = enb_dev->GetCellId ();
  	if (enb_dev != 0) {
      // set the Tx power to 1 dBm
      NS_LOG_INFO ("ENB" << id << ": Shutting down cell " << cell_id);
      Ptr<LteEnbPhy> phy = enb_dev->GetPhy ();
      phy->SetTxPower (1);
    }
  }
}

bool
LteSimulatorHelper::setup_3d_parabolic_antenna_for_enb(uint32_t i, Ptr<EnbConfig>& enb_cfg, std::ostringstream& os) {
    NS_LOG_FUNCTION (this);

	double horix_beam_width;;
	double orientation;
	double e_tilt;
	double m_tilt;
	double vert_beamwidth;
	double horiz_max_attenuation;
	double vert_max_attenuation;
	double bore_sight_gain;
	
	if (enb_cfg->get_e_tilt(i, e_tilt) &&
		enb_cfg->get_m_tilt(i, m_tilt) &&
		enb_cfg->get_orientation(i, orientation) &&
		enb_cfg->get_horiz_beamwidth(i, horix_beam_width) &&
		enb_cfg->get_vert_beamwidth(i, vert_beamwidth) &&
		enb_cfg->get_horiz_max_attenuation(i, horiz_max_attenuation) &&
		enb_cfg->get_vert_max_attenuation(i, vert_max_attenuation) &&
		enb_cfg->get_vert_bore_sight_gain(i, bore_sight_gain)) {

		os << "\te_tilt = " << e_tilt << std::endl;
		os << "\tm_tilt = " << m_tilt << std::endl;
		os << "\torientation = " << orientation << std::endl;
		os << "\thorix_beam_width = " << horix_beam_width << std::endl;
		os << "\tvert_beamwidth = " << vert_beamwidth << std::endl;
		os << "\thoriz_max_attenuation = " << horiz_max_attenuation << std::endl;
		os << "\tvert_max_attenuation = " << vert_max_attenuation << std::endl;
		os << "\tvert_max_attenuation = " << vert_max_attenuation << std::endl;
		os << "\tbore_sight_gain = " << bore_sight_gain << std::endl;

		m_lteHelper->SetEnbAntennaModelAttribute ("HorizontalBeamwidth", DoubleValue (horix_beam_width));
		m_lteHelper->SetEnbAntennaModelAttribute ("Orientation",   DoubleValue (orientation));
		m_lteHelper->SetEnbAntennaModelAttribute ("MaxHorizontalAttenuation", DoubleValue (horiz_max_attenuation));
		m_lteHelper->SetEnbAntennaModelAttribute ("VerticalBeamwidth", DoubleValue (vert_beamwidth));
		m_lteHelper->SetEnbAntennaModelAttribute ("ElectricalTilt", DoubleValue (e_tilt));
		m_lteHelper->SetEnbAntennaModelAttribute ("MechanicalTilt", DoubleValue (m_tilt));
		m_lteHelper->SetEnbAntennaModelAttribute ("MaxVerticalAttenuation", DoubleValue (vert_max_attenuation));
//		m_lteHelper->SetEnbAntennaModelAttribute ("BoresightGain", DoubleValue (bore_sight_gain));
		return true;
	}
	return false;
}

void
LteSimulatorHelper::add_enb_nodes(NodeContainer& enb_nodes, Ptr<EnbConfig>& enb_cfg, std::ostringstream& os) {
//	uint32_t num_enbs_allocated = enb_nodes.GetN();
//	std::cout << "Total ENBS = " << num_enbs_allocated << std::endl;
	enb_indicies = enb_cfg->get_indicies();
	NS_ASSERT(enb_indicies.size() == enb_nodes.GetN());
	uint32_t i = 0;
	m_lteHelper->SetEnbAntennaModelType (m_antennaModel);
	for (NodeContainer::Iterator iter = enb_nodes.Begin(); iter < enb_nodes.End(); iter++) {
		Ptr<Node> enb = *iter;
		uint32_t id = enb_indicies[i];

		uint32_t dl_earfcn;
//		uint32_t id = enb->GetId();
		double horix_beam_width;;
		double orientation;
		double e_tilt;
		double m_tilt;
		double vert_beamwidth;
		double horiz_max_attenuation;
		double vert_max_attenuation;
		double bore_sight_gain;


		os << "Adding ENB" << id << "::" << std::endl;
		if (enb_cfg->get_dl_earfcn(id, dl_earfcn)) {
			uint32_t ul_earfcn = get_ul_earfcn(dl_earfcn);
			if (ul_earfcn == 0) {
				std::cout << "Unsupported dl_earfcn " << dl_earfcn << std::endl;
				exit(-1);
			}
			os << "\tDL/UL Earfcn = " << dl_earfcn << " / " << ul_earfcn << std::endl;
			m_lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (dl_earfcn));
			m_lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (ul_earfcn));
		}
		uint32_t bw_rb;
		if (enb_cfg->get_bandwidth_in_rb(id, bw_rb)) {
			// Set LTE BW
			m_lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue(bw_rb));
			m_lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue(bw_rb));
		}

		// RR scheduler
		//m_lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
		// PF Scheduler
		// m_lteHelper->SetSchedulerType ("ns3::pw_PfFfMacScheduler");
		std::string antenna_type = m_lteHelper->GetEnbAntennaModelType ();  //Antenna type has to be same for all ENBs
		NetDeviceContainer container_ = m_lteHelper->InstallEnbDevice(enb);
		m_enb_net_dev_container_map[id] = container_;
//		std::cout << "**** Putting ENB" << id << " in m_enb_net_dev_container_map; Num devs in container = " << container_.GetN() << std::endl;
		double tx_pwr;
		enb_cfg->get_tx_pwr(id, tx_pwr);
		Ptr<NetDevice> net_dev = container_.Get(0); // Only One ENB per container
		Ptr<LteEnbNetDevice> enb_net_dev_p = ns3::DynamicCast<LteEnbNetDevice>(net_dev);
		Ptr<LteEnbPhy> phy_ = enb_net_dev_p->GetPhy();
		os << "\tNet dev = " << enb_net_dev_p << "; Phy = " << phy_ << "; Tx PWR = " << tx_pwr << std::endl;
		os << "\tAntenna type = " << m_lteHelper->GetEnbAntennaModelType() << std::endl;

		phy_->SetTxPower(tx_pwr);
		Ptr<LteSpectrumPhy> dl_spec_phy_ = phy_->GetDownlinkSpectrumPhy();
		Ptr<AntennaModel> antenna_ = dl_spec_phy_->GetRxAntenna ();

		if (m_antennaModel == "ns3::Parabolic3dAntennaModel") {
			if (enb_cfg->get_e_tilt(i, e_tilt) &&
				enb_cfg->get_m_tilt(i, m_tilt) &&
				enb_cfg->get_orientation(i, orientation) &&
				enb_cfg->get_horiz_beamwidth(i, horix_beam_width) &&
				enb_cfg->get_vert_beamwidth(i, vert_beamwidth) &&
				enb_cfg->get_horiz_max_attenuation(i, horiz_max_attenuation) &&
				enb_cfg->get_vert_max_attenuation(i, vert_max_attenuation) &&
				enb_cfg->get_vert_bore_sight_gain(i, bore_sight_gain)) {
				os << "\tE-ElectricalTilt = " << e_tilt << "\n";
				os << "\tHorizontalBeamwidth = " << horix_beam_width << "\n";
				os << "\tOrientation = " << orientation << "\n";
				os << "\tMaxHorizontalAttenuation = " << horiz_max_attenuation << "\n";
				os << "\tVerticalBeamwidth = " << vert_beamwidth << "\n";
				os << "\tMechanicalTilt = " << m_tilt << "\n";
				os << "\tMaxVerticalAttenuation = " << vert_max_attenuation << "\n";

				antenna_->SetAttribute("ElectricalTilt", DoubleValue(e_tilt));
				antenna_->SetAttribute("HorizontalBeamwidth", DoubleValue (horix_beam_width));
				antenna_->SetAttribute("Orientation",   DoubleValue (orientation));
				antenna_->SetAttribute("MaxHorizontalAttenuation", DoubleValue (horiz_max_attenuation));
				antenna_->SetAttribute("VerticalBeamwidth", DoubleValue (vert_beamwidth));
				antenna_->SetAttribute("ElectricalTilt", DoubleValue (e_tilt));
				antenna_->SetAttribute("MechanicalTilt", DoubleValue (m_tilt));
				antenna_->SetAttribute("MaxVerticalAttenuation", DoubleValue (vert_max_attenuation));
			}
		}
		else if (m_antennaModel == "ns3::CosineAntennaModel") {
			double beamwidth = 50;
			double orientation = 0;
			double max_gain_ = 0.0;

			if (enb_cfg->get_orientation(i, orientation) &&
				enb_cfg->get_horiz_beamwidth(i, beamwidth)) {
				os << "\torientation = " << orientation << std::endl;
				os << "\tbeamwidth = " << beamwidth << std::endl;
				os << "\tmax_gain = " << max_gain_ << std::endl;		
				antenna_->SetAttribute ("Orientation", DoubleValue (orientation));
				antenna_->SetAttribute ("Beamwidth", DoubleValue (beamwidth));
				antenna_->SetAttribute ("MaxGain", DoubleValue (max_gain_));
			}
		}
		else {
			os << "\tUnsupported Antenna Model = " << m_antennaModel << std::endl;
		}
		os << "\n";
		i++;
	}
	NS_ASSERT (i == enb_nodes.GetN());
}

void
LteSimulatorHelper::print_enb_to_cell_id_mapping(std::ostringstream& os) {
	os << "EnbID to CellID mapping::\n";
  	std::map<uint16_t, NetDeviceContainer>::iterator iter = m_enb_net_dev_container_map.begin();
  	for (; iter != m_enb_net_dev_container_map.end(); iter++) {
		uint16_t id = iter->first;
		NetDeviceContainer container_ = iter->second;
		Ptr<NetDevice> net_dev = container_.Get(0); // Only One ENB per container
		Ptr<LteEnbNetDevice> enb_dev_ptr = net_dev->GetObject<LteEnbNetDevice>();
		uint32_t cell_id = enb_dev_ptr->GetCellId();
		os << "\tENB" << id << ": Cell ID " << cell_id << std::endl;
	}
}

Vector
LteSimulatorHelper::get_position_of_enb(uint32_t enb_id) {
	NetDeviceContainer container_ = m_enb_net_dev_container_map[enb_id];
	Ptr<NetDevice> net_dev = container_.Get(0); // Only One ENB per container
	Ptr<MobilityModel> mmUe = net_dev->GetObject<MobilityModel>();
	Vector ps_ = mmUe->GetPosition();
	return ps_;
}

}
