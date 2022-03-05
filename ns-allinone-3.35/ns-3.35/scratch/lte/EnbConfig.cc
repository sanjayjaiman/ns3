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

#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/csv-reader.h"
#include "EnbConfig.h"

#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("EnbConfig");

NS_OBJECT_ENSURE_REGISTERED (EnbConfig);

TypeId
EnbConfig::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::EnbConfig")
    .SetParent<Object> ()
    .SetGroupName ("Mobility")
    .AddConstructor<EnbConfig> ()
    ;
  return tid;
}

EnbConfig::EnbConfig () {
  enbPositionAlloc = CreateObject<ListPositionAllocator>();
};

EnbConfig::EnbConfig (std::string enb_pos_file) {
  enbPositionAlloc = CreateObject<ListPositionAllocator>();
  put_enbs(enb_pos_file);
}
void
EnbConfig::Install(std::vector<EnbInfo>& enb_vector) {
	uint32_t i = 0;
  for (std::vector<EnbInfo>::iterator iter = enb_vector.begin(); iter != enb_vector.end(); iter++) {
    EnbInfo enb_ = *iter;
    Vector& b = enb_.pos;
    NS_LOG_INFO ("Adding Enb" << i << ":(" << b.x << ", " << b.y << ", " <<b.z << ")");
    enbPositionAlloc->Add(b);
    i++;
  }
}


void
EnbConfig::put_enbs (const std::string filePath, char delimiter /* = ',' */)
{
  NS_LOG_FUNCTION (this << filePath << std::string ("'") + delimiter + "'");

  CsvReader csv (filePath, delimiter);
  uint32_t i = 0;
  
  while (csv.FetchNextRow ())  {
    std::ostringstream os;
    uint32_t col_count = csv.ColumnCount ();
    if (col_count == 1 || col_count == 0) {
//        NS_LOG_INFO ("** Comment line **"); 
        // comment line
        continue;
    }
    Vector pos_;
    double hbw, o, hma, vbw, et, mt, vma, bsg, tx_pwr;
    uint32_t dl_earfcn, bw_rb;
    uint32_t id_;

    NS_ASSERT_MSG ((col_count == 15), "File:" << filePath <<"::Invalid number of tokens in line ");

    bool ok = csv.GetValue (0, id_);
    os << "id " << id_ << ", pos = (";
    ok = csv.GetValue (1, pos_.x);
    os << pos_.x << ",";
    NS_ASSERT_MSG (ok, "failed reading x");
    ok = csv.GetValue (2, pos_.y);
    os << pos_.y << ",";
    NS_ASSERT_MSG (ok, "failed reading y");
    ok = csv.GetValue (3, pos_.z);
    os << pos_.z << "), ";
    NS_ASSERT_MSG (ok, "failed reading hbw");
    ok = csv.GetValue (4, hbw);
    os << "hbw = " << hbw << ", ";
    NS_ASSERT_MSG (ok, "failed reading o");
    ok = csv.GetValue (5, o);
    os << "o = " << o << ", ";
    NS_ASSERT_MSG (ok, "failed reading o");
    ok = csv.GetValue (6, hma);
    os << "hma = " << hma << ", ";
    NS_ASSERT_MSG (ok, "failed reading hma");
    ok = csv.GetValue (7, vbw);
    os << "vbw = " << vbw << ", ";
    NS_ASSERT_MSG (ok, "failed reading vbw");
    ok = csv.GetValue (8, et);
    os << "et = " << et << ", ";
    NS_ASSERT_MSG (ok, "failed reading et");
    ok = csv.GetValue (9, mt);
    os << "mt = " << mt << ", ";
    NS_ASSERT_MSG (ok, "failed reading mt");
    ok = csv.GetValue (10, vma);
    os << "vma = " << vma << ", ";
    NS_ASSERT_MSG (ok, "failed reading vma");
    ok = csv.GetValue (11, bsg);
    os << "bsg = " << bsg << ", ";
    NS_ASSERT_MSG (ok, "failed reading bsg");
    ok = csv.GetValue (12, dl_earfcn);
    os << "dl_earfcn = " << dl_earfcn << ", ";
    NS_ASSERT_MSG (ok, "failed reading dl_earfcn");
    ok = csv.GetValue (13, bw_rb);
    os << "bw_rb = " << bw_rb << ", ";
    NS_ASSERT_MSG (ok, "failed reading bw_rb");
    ok = csv.GetValue (14, tx_pwr);
    os << "tx_pwr = " << tx_pwr << ", ";
    NS_ASSERT_MSG (ok, "failed reading tx_pwr");
    enb_vector_.push_back(EnbInfo(id_, pos_, hbw, o, hma, vbw, et, mt, vma, bsg, dl_earfcn, bw_rb, tx_pwr));
    NS_LOG_INFO ("Row#" << i << ": " << os.str());
    i++;
  }  // while FetchNextRow
  NS_LOG_INFO ("Read: " << i << " rows");
  Install(enb_vector_);
}

uint32_t
EnbConfig::GetSize (void) const
{
  return enb_vector_.size();
}

void
EnbConfig::to_string_comma_seperated (std::ostringstream& os) {
  EnbInfo::to_string_comma_seperated_header(os);
  os << std::endl;
  for (std::vector<EnbInfo>::iterator iter = enb_vector_.begin(); iter !=  enb_vector_.end(); iter++) {
    EnbInfo enb_cfg = *iter;
    enb_cfg.to_string_comma_seperated(os);
    os << std::endl;
  }
}

void
EnbConfig::print (std::ostringstream& os)
{
		uint32_t num_enbs_allocated = GetSize();
    os << "\n================================================================================" << std::endl;
    os << "ENB Config:: Num Enbs = " << num_enbs_allocated << std::endl;
		for (std::vector<EnbInfo>::iterator iter = enb_vector_.begin(); iter !=  enb_vector_.end(); iter++) {
			EnbInfo enb_cfg = *iter;
      enb_cfg.to_string(os);
		}
    os << "================================================================================" << std::endl;
}

std::vector<uint32_t> EnbConfig::get_indicies() {
  std::vector<uint32_t> ret;
	for (std::vector<EnbInfo>::iterator iter = enb_vector_.begin(); iter !=  enb_vector_.end(); iter++) {
      EnbInfo enb_struct = *iter;
      ret.push_back(enb_struct.id);
  }
  return ret;
}

bool EnbConfig::get_enb_struct(EnbInfo& ret, uint32_t index) {		
		for (std::vector<EnbInfo>::iterator iter = enb_vector_.begin(); iter !=  enb_vector_.end(); iter++) {
      EnbInfo enb_struct = *iter;
      uint32_t i = enb_struct.id;
      if (index == i) {
			  ret = *iter;
        return true;
      }
    }
    return false;
}
bool EnbConfig::get_tx_pwr(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.tx_pwr;
  return true;
}
bool EnbConfig::get_dl_earfcn(uint32_t index, uint32_t& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.dl_earfcn;
  return true;
}
bool EnbConfig::get_bandwidth_in_rb(uint32_t index, uint32_t& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.bw_rb;
  return true;
};
bool EnbConfig::get_e_tilt(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.ElecTilt;
  return true;
};
bool EnbConfig::get_m_tilt(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.MechTilt;
  return true;
};
bool EnbConfig::get_orientation(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.Orientation;
  return true;
};
bool EnbConfig::get_horiz_beamwidth(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.HorizBeamwidth;
  return true;
};
bool EnbConfig::get_vert_beamwidth(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.VertBeamwidth;
  return true;
};
bool EnbConfig::get_pos_vector(uint32_t index, Vector& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.pos;
  return true;
};
bool EnbConfig::get_horiz_max_attenuation(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.HorizontalMaxAttenuation;
  return true;
};
bool EnbConfig::get_vert_max_attenuation(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.VerticalMaxAttenuation;
  return true;
};

bool EnbConfig::get_vert_bore_sight_gain(uint32_t index, double& val) {
  EnbInfo enb_struct;
  if (! get_enb_struct(enb_struct, index))
    return false;
  val = enb_struct.BoresightGain;
  return true;  
}

}