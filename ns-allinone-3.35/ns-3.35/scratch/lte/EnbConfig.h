/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
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
 * Author: Sanjay Jaiman <sjaiman@parallelwireless,com>
 */
#ifndef ENB_CONFIG_H
#define ENB_CONFIG_H

#include <iostream>
#include <vector>
#include <memory>
#include "ns3/object.h"
#include "ns3/vector.h"
#include "ns3/node-container.h"
#include <ns3/mobility-building-info.h>
#include <ns3/constant-position-mobility-model.h>
#include "ns3/mobility-module.h"

namespace ns3 {

class EnbInfo {
public:
  EnbInfo() {};
  EnbInfo(uint32_t id_, Vector pos_, double hbw, double o, double hma, double vbw, double et, double mt, double vma, double bsg, uint32_t dl_earfcn_, uint32_t bw_rb_, double tx_pwr_) :
    id(id_), pos(pos_), HorizBeamwidth(hbw), Orientation(o), HorizontalMaxAttenuation(hma), VertBeamwidth(vbw), ElecTilt(et),
    MechTilt(mt), VerticalMaxAttenuation(vma), BoresightGain(bsg), dl_earfcn(dl_earfcn_), bw_rb(bw_rb_), tx_pwr(tx_pwr_) {};
  void to_string(std::ostringstream& os) {
    	os << "ID = " << id << std::endl;
      os << "\tpos = (" << pos.x << "," << pos.y << "," << pos.z << ");" << std::endl;
      os << "\tHorizBeamwidth = " <<  HorizBeamwidth << std::endl;
      os << "\tOrientation = " <<  Orientation << std::endl;
      os << "\tHorizontalMaxAttenuation = " <<  HorizontalMaxAttenuation << std::endl;
      os << "\tVertBeamwidth = " <<  VertBeamwidth  << std::endl;
      os << "\tElecTilt = " <<  ElecTilt  << std::endl;
      os << "\tMechTilt = " <<  MechTilt  << std::endl;
      os << "\tVerticallMaxAttenuation = " <<  VerticalMaxAttenuation  << std::endl;
      os << "\tBoresightGain = " <<  BoresightGain  << std::endl;
      os << "\tdl_earfcn = " <<  dl_earfcn  << std::endl;
      os << "\tBandwidth (rb) = " <<  bw_rb  << std::endl;
      os << "\tTX Power = " <<  tx_pwr  << std::endl;
  };
  static void to_string_comma_seperated_header(std::ostringstream& os) {
    	os << "ID(x/y/z),HorizBeamwidth,Orientation,HorizontalMaxAttenuation,VertBeamwidth,";
      os << "ElecTilt,MechTilt,VerticallMaxAttenuation,BoresightGain,dl_earfcn,Bandwidth,";
      os << "TX Power";
  };
  void to_string_comma_seperated(std::ostringstream& os) {
    	os << id;
      os << "(" << pos.x << "/" << pos.y << "/" << pos.z << ")";
      os << "," <<  HorizBeamwidth;
      os << "," <<  Orientation;
      os << "," <<  HorizontalMaxAttenuation;
      os << "," <<  VertBeamwidth ;
      os << "," <<  ElecTilt ;
      os << "," <<  MechTilt ;
      os << "," <<  VerticalMaxAttenuation ;
      os << "," <<  BoresightGain ;
      os << "," <<  dl_earfcn ;
      os << "," <<  bw_rb ;
      os << "," <<  tx_pwr ;
  };
public:
  uint32_t id;
  Vector pos;
  double HorizBeamwidth;
  double Orientation;
  double HorizontalMaxAttenuation;
  double VertBeamwidth;
  double ElecTilt;
  double MechTilt;
  double VerticalMaxAttenuation;
  double BoresightGain;
  uint32_t dl_earfcn;
  uint32_t bw_rb;
  double tx_pwr;
};


class EnbConfig : public Object
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  EnbConfig ();
  EnbConfig (std::string enb_pos_file);
  virtual ~EnbConfig () {};

  /**
   * \brief Add the positions listed in a file.
   * The file should be a simple text file, with one position per line,
   * either X and Y, or X, Y and Z, in meters.  The delimiter can
   * be any character, such as ',' or '\t'; the default is a comma ','.
   *
   * The file is read using CsvReader, which explains how comments
   * and whitespace are handled.
   *
   * \param [in] filePath The path to the input file.
   * \param [in] delimiter The delimiter character; see CsvReader.
   */
  void put_enbs (const std::string filePath, char delimiter = ',');

  uint32_t GetSize (void) const;
  void Print() {
    std::ostringstream os;
		print(os);
		std::cout << os.str();
  };
  std::vector<uint32_t> get_indicies();
  void   to_string_comma_seperated (std::ostringstream& os);
  bool   get_tx_pwr(uint32_t index, double& val);
  bool   get_dl_earfcn(uint32_t index, uint32_t& val);
  bool   get_bandwidth_in_rb(uint32_t index, uint32_t& val);
  bool   get_e_tilt(uint32_t index, double& val);
  bool   get_m_tilt(uint32_t index, double& val);
  bool   get_orientation(uint32_t index, double& val);
  bool   get_horiz_beamwidth(uint32_t index, double& val);
  bool   get_vert_beamwidth(uint32_t index, double& val);
  bool   get_pos_vector(uint32_t index, Vector& val);
  bool   get_horiz_max_attenuation(uint32_t index, double& val);
  bool   get_vert_max_attenuation(uint32_t index, double& val);
  bool   get_vert_bore_sight_gain(uint32_t index, double& val);
  Ptr<ListPositionAllocator> get_enb_position_alloc() {return enbPositionAlloc;}
private:
  std::vector<EnbInfo> enb_vector_;
  Ptr<ListPositionAllocator> enbPositionAlloc;
  void Install(std::vector<EnbInfo>& enb_vector);
  void print (std::ostringstream& os);
  bool get_enb_struct(EnbInfo& ret, uint32_t index);
};


} // namespace ns3

#endif /* ENB_CONFIG_H */
