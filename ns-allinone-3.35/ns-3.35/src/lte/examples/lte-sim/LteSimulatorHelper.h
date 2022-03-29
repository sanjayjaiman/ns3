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

#ifndef _LTE_SIMULATOR_HELPER_H_
#define _LTE_SIMULATOR_HELPER_H_

#include <ns3/log.h>
#include <ns3/double.h>
#include <ns3/integer.h>
#include <ns3/uinteger.h>
#include <ns3/boolean.h>
#include <ns3/string.h>
#include "ns3/pointer.h"

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <ctime>
#include "ns3/object.h"
#include "ue_mobility_helper.h"

#include <ns3/hybrid-3gpp-propagation-loss-model.h>
#include <ns3/okumura-hata-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/three-gpp-propagation-loss-model.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>

#include <ns3/parabolic-antenna-model.h>
#include <ns3/three-gpp-antenna-array-model.h>
#include <ns3/cosine-antenna-model.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/log.h>
#include <ns3/double.h>
#include <ns3/parabolic-3d-antenna-model.h>
#include <ns3/building-list.h>
#include "BuildingPositionAlloc.h"
#include "EnbConfig.h"
#include "UeConfig.h"
#include "SimulationParams.h"

namespace ns3 {

#define MAX_UES  10
#define CELL_SHUT_DOWN_TIME 2


class UeEnbIdCfg : public Object {
public:
  static TypeId GetTypeId (void);
  UeEnbIdCfg() : enb_id(-1) {};
  UeEnbIdCfg(uint32_t id) : enb_id(id) {};
  void set_enb_id(uint32_t id) {enb_id = id;};
  void set_pos(Vector pos_) {pos = pos_;};
  uint32_t id() {return enb_id;}
private:
    uint32_t enb_id;
    Vector   pos;
};

typedef struct ObjAndParam {
  std::string obj;
  std::string param;
} ObjAndParam_t;


class LteSimulatorHelper : public Object {
public:
  /**
   * Constructor
   */
  LteSimulatorHelper()  {
    m_lteHelper = CreateObject<LteHelper>();
    sim_params = CreateObject<SimulationParameters>();
  };
  LteSimulatorHelper(Ptr<LteHelper> lteHelper, Ptr<SimulationParameters> sim_params_) : 
      m_lteHelper(lteHelper), sim_params(sim_params_) {
  };

  /**
   * Destructor
   */
  virtual ~LteSimulatorHelper () {};

  // inherited from Object
  /**
   *  Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId (void);
  void          set_fading_spectrum_and_scheduler ();
  void          add_buildings();
  void          testPropagationLoss (Ptr<MobilityModel>& mmEnb, NodeContainer& ueNodes);
  void          add_enb_nodes(NodeContainer& enb_nodes, Ptr<EnbConfig>& enb_cfg, std::ostringstream& os);
  void          print_enb_to_cell_id_mapping(std::ostringstream& os);
  void          install_ue_nodes(Ptr<EnbConfig>& enb_cfg, Ptr<UeConfig>& ue_cfg, uint32_t verbosity);
  void          print_buildings_info(std::ostringstream& os);
  void          print_buildings_info_comma_seperated(std::ostringstream& os);
  Vector        get_position_of_enb(uint32_t enb_id);
  TypeId        get_path_loss_model();
  NodeContainer get_ue_nodes();
  NodeContainer get_active_ue_nodes();
  NodeContainer get_idle_ue_nodes();
  void printUeToEnbID (NetDeviceContainer devices, NodeContainer c, std::ostringstream& os);
  void verify_init_values(uint32_t verbose, bool print_defaults);
  static bool get_initial_value(TypeId tid, std::string paramName, std::string& type, std::string& value);
  static void print_helper_defaults(TypeId tid, std::ostringstream& os);

  void print_lte_helper_values();
  void add_to_apps(Ptr<Node>& local_node, Ptr<Node>& remote_node,
				ApplicationContainer& client_apps, ApplicationContainer& server_apps,
				uint32_t local_port, uint32_t remote_port, Time& inter_packet_interval, uint32_t max_packet_count, std::ostringstream& os);
  void add_enb_positions(Ptr<ListPositionAllocator>& enbPositionAlloc, Ptr<LteSimulatorHelper>& simu_helper_, uint32_t numEnb);
  uint32_t get_ul_earfcn(uint32_t dl_earfcn);
  void LogEnable(bool verbose);

  void ConnectionEstablishedCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);
  void NotifyConnectionReleaseCallback (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti);
  void HandoverStartCallback (std::string context, uint64_t imsi,
                              uint16_t sourceCellId, uint16_t rnti,
                              uint16_t targetCellId);
  void EnbHandoverEndOkCallback (std::string context, uint64_t imsi,
                                 uint16_t cellid, uint16_t rnti);
  void CellShutdownCallback ();
  void SetAntennaModel (std::string type);
  std::string GetAntennaModel () const;
  void SetPropagationLossModel (std::string type);
  std::string GetPropagationLossModel () const;
  void SetBuildingsPropagationLossModel (std::string type);
  std::string GetBuildingsPropagationLossModel () const;
  void SetFadingLossModel (std::string type);
  std::string GetFadingLossModel () const;
  void SetSpectrumChannelModel (std::string type);
  std::string GetSpectrumChannelModel () const;
  Ptr<LteEnbNetDevice> get_ENB_net_device(uint32_t enb_id);
  uint32_t sift_ues_for_enbid(uint32_t enb_id, NodeContainer &container, NodeContainer &out);
  void     find_ues_for_closest_enbid(Ptr<EnbConfig> enb_cfg, NodeContainer &ue_container, std::map<uint32_t, NodeContainer> &out, std::ostringstream& os);
  NetDeviceContainer get_ue_container_for_enbid(uint32_t enb_id);
 
private:
  void print_enbs_power(NetDeviceContainer& enbDev);
  bool setup_3d_parabolic_antenna_for_enb(uint32_t i, Ptr<EnbConfig>& enb_cfg, std::ostringstream& os);
  template <class myType> void print_attr (std::string params[], uint32_t size, std::ostringstream& os);

public:
  std::string       m_popagationLossModel;
  std::string       m_buildingsPopagationLossModel;

private:
  std::vector<uint32_t> enb_indicies;
  std::map<uint32_t, std::vector<Ptr<UeDesc>>> enb_id_to_ue_nodes_map;
  std::map<uint16_t, NetDeviceContainer> m_enb_net_dev_container_map;
  std::string       m_antennaModel;
  std::string       m_fading_loss_model;
  std::string       m_spectrum_channel_model;
  bool              m_test_propagation_loss;
  bool              m_test_cell_bring_down;
  Ptr<LteHelper>    m_lteHelper;
  Ptr<SimulationParameters> sim_params;
  Ptr<BuildingPositionAllocator> buildingPositionAlloc;
  bool m_hasHandoverOccurred; ///< has handover occurred?
  Time m_enbHandoverStart; ///< ENB handover start time

  typedef enum {
    ACTIVE_CONSTANT_POS = 1,
    IDLE_CONSTANT_POS,
    ACTIVE_RAND_WALK,
    IDLE_RAND_WALK,
    ACTIVE_CONST_VEL,
    IDLE_CONST_VEL,
  } ue_node_type;
//  uint16_t m_targetCellId; ///< target cell ID
	std::map<ue_node_type, NodeContainer> ue_nodes_container_map;

  std::vector<Ptr<UeMobilityHelper >> ue_mobility_helpers;

};



};

#endif // _LTE_SIMULATOR_HELPER_H_