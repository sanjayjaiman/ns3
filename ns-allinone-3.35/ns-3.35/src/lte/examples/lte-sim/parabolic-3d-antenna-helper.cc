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


#include "parabolic-3d-antenna-helper.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Parabolic3dAntennaHelper");

NS_OBJECT_ENSURE_REGISTERED (Parabolic3dAntennaHelper);


Parabolic3dAntennaHelper::Parabolic3dAntennaHelper ()
{
  NS_LOG_FUNCTION (this);
   m_antenna = CreateObject<Parabolic3dAntennaModel> ();
}

Parabolic3dAntennaHelper::Parabolic3dAntennaHelper (
    Angles a,
    double horiz_beamwidth,
    double vert_beamwidth,
    double elec_tilt,
    double mech_tilt,
    double orientation,
    double max_horiz_attenuation,
    double max_vert_attenuation,
    double expectedGainDb) :
      m_expectedGain (expectedGainDb),
      m_horiz_beamwidth(horiz_beamwidth),
      m_vert_beamwidth(vert_beamwidth),
      m_elec_tilt(elec_tilt),
      m_mech_tilt(mech_tilt),
      m_phi(orientation), 
      m_max_horiz_attenuation(max_horiz_attenuation),
      m_max_vert_attenuation(max_vert_attenuation) {
  m_antenna = CreateObject<Parabolic3dAntennaModel> ();  
}

Parabolic3dAntennaHelper::Parabolic3dAntennaHelper (double phi, double theta,
    double horiz_beamwidth,
    double vert_beamwidth,
    double elec_tilt,
    double mech_tilt,
    double orientation,
    double max_horiz_attenuation,
    double max_vert_attenuation,
    double expectedGainDb):
    Parabolic3dAntennaHelper(Angles (DegreesToRadians(phi),DegreesToRadians (theta)),
      horiz_beamwidth, vert_beamwidth, elec_tilt, mech_tilt, orientation,
      max_horiz_attenuation, max_vert_attenuation, expectedGainDb) {
    m_phi = phi;
}

Parabolic3dAntennaHelper::~Parabolic3dAntennaHelper ()
{
  NS_LOG_FUNCTION (this);
}


TypeId
Parabolic3dAntennaHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Parabolic3dAntennaHelper")
    .SetParent<Object> ()
    .SetGroupName("Parabolic3dAntenna")
    .AddConstructor<Parabolic3dAntennaHelper> ()
    .AddAttribute ("Azimuth",
                  "Azimuth",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_phi),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("HorizBeamwidth",
                  "Horizontal Beamwidth",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_horiz_beamwidth),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("VertBeamwidth",
                  "Vertical Beamwidth",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_vert_beamwidth),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("ElecTilt",
                  "Electrical Tilt",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_elec_tilt),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("MechTilt",
                  "Mechanical Tilt",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_mech_tilt),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("Orientation",
                  "Orientation",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_phi),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("HorizontalMaxAttenuation",
                  "horizontal maxAttenuation",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_max_horiz_attenuation),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("VerticallMaxAttenuation",
                  "vertical maxAttenuation",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_max_vert_attenuation),
                  MakeDoubleChecker<double> ())
    .AddAttribute ("ExpectedGain",
                  "Expected Gain",
                  DoubleValue (0.0),
                  MakeDoubleAccessor (&Parabolic3dAntennaHelper::m_expectedGain),
                  MakeDoubleChecker<double> ())
    ;
  return tid;
}

TypeId
Parabolic3dAntennaHelper::GetInstanceTypeId (void) const
{
  return GetTypeId ();
};

void Parabolic3dAntennaHelper::CreateAntenna (void) {
    m_antenna->SetAttribute ("HorizontalBeamwidth", DoubleValue (m_horiz_beamwidth));
    m_antenna->SetAttribute ("Orientation", DoubleValue (m_phi));
    m_antenna->SetAttribute ("MaxHorizontalAttenuation", DoubleValue (m_max_horiz_attenuation));
    m_antenna->SetAttribute ("VerticalBeamwidth", DoubleValue (m_vert_beamwidth));
    m_antenna->SetAttribute ("ElectricalTilt", DoubleValue (m_elec_tilt));
    m_antenna->SetAttribute ("MechanicalTilt", DoubleValue (m_mech_tilt));
    m_antenna->SetAttribute ("MaxVerticalAttenuation", DoubleValue (m_max_vert_attenuation));
};

bool Parabolic3dAntennaHelper::test() {
    double actual = m_antenna->GetGainDb (Angles (DegreesToRadians(m_phi),DegreesToRadians (m_elec_tilt)));
    double limit =  m_expectedGain;
    double tol = GAIN_TOL;
    std::ostringstream os;
    BuildNameString (os);
    std::cout << "\n*** TEST ***\n" << os.str() <<
    "\nActual Gain = " << actual << "; Expected = " << limit << "; tol = " << tol << std::endl;
    if ((actual) > (limit) + (tol) || (actual) < (limit) - (tol)) {
      std::cout << "wrong value of the radiation pattern" << std::endl;
      return false;
    }
    return true;
};


void
Parabolic3dAntennaHelper::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  Object::DoDispose ();
}

}