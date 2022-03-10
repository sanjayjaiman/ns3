/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 *         sjaiman@parallelwirelless.com
 */

#ifndef PARABOLIC_3D_ANTENNA_HELPER_H
#define PARABOLIC_3D_ANTENNA_HELPER_H

#include <ns3/log.h>
#include <ns3/double.h>
#include <ns3/parabolic-3d-antenna-model.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include "ns3/string.h"
#include "ns3/double.h"

namespace ns3 {

#define GAIN_TOL .01

/*
 */
class Parabolic3dAntennaHelper : public Object
{
public:
  /**
   * Constructor
   */
  Parabolic3dAntennaHelper();
  Parabolic3dAntennaHelper (Angles a,
    double horiz_beamwidth,
    double vert_beamwidth,
    double elec_tilt,
    double mech_tilt,
    double orientation,
    double max_horiz_attenuation,
    double max_vert_attenuation,
    double expectedGainDb);
  Parabolic3dAntennaHelper (double phi, double theta,
    double horiz_beamwidth,
    double vert_beamwidth,
    double elec_tilt,
    double mech_tilt,
    double orientation,
    double max_horiz_attenuation,
    double max_vert_attenuation,
    double expectedGainDb);

  /**
   * Destructor
   */
  virtual ~Parabolic3dAntennaHelper ();

  // inherited from Object
  /**
   *  Register this type.
   *  \return The object TypeId.
   */
  static TypeId GetTypeId (void);
  virtual void DoDispose ();
  TypeId GetInstanceTypeId () const;
  bool test();
  virtual void BuildNameString (std::ostringstream& os) {
    os <<  "\tAzimuth = " << m_phi  << "\n\t"
      << "Expected gain = " << m_expectedGain << ", Horiz beamdwidth = " << m_horiz_beamwidth << " deg" << "; Vert beamdwidth = " << m_vert_beamwidth << " deg" << "\n\t"
      << "Elec tilt = " << m_elec_tilt << " deg" << ", Mech tilt = " << m_mech_tilt << " deg " << ", orientation = " << m_phi << "\n\t"
      << "Horizontal maxAttenuation = " << m_max_horiz_attenuation << " dB" << ", Vertical maxAttenuation=" << m_max_vert_attenuation << " dB" << std::endl;
  };
  void Print(std::ostringstream& os) {BuildNameString (os);};
  void CreateAntenna(void);

private:
  double m_expectedGain;
  double m_horiz_beamwidth;
  double m_vert_beamwidth;
  double m_elec_tilt;
  double m_mech_tilt;
  double m_phi;
  double m_max_horiz_attenuation;
  double m_max_vert_attenuation;
  Ptr<AntennaModel> m_antenna;
};

}; // namespace ns3

#endif // PARABOLIC_3D_ANTENNA_HELPER_H
