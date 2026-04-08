/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include "emergencyVehicleAlert.h"
#include "socketClient.h"
#include <cmath>
#include <unordered_map>
#include "json.hpp"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"
#define DEG_2_RAD(val) ((val) * M_PI / 180.0)

using namespace std;
const double distance_offset = 5.06614e+06;
const int denm_transmit_distance = 200;
const int cam_transmit_distance = 200;
const double attack_range = 200;
const double attack_procedure_start_time = 15.0; // seconds

string attacker_id = "";
string victim_m_id;
std::unordered_set<std::string> veh_set = {};
int denm_veh_min_speed = 5;
int denm_veh_max_speed = 20;
int denm_veh_accel = -3;
using json = nlohmann::json;

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("emergencyVehicleAlert");

NS_OBJECT_ENSURE_REGISTERED (emergencyVehicleAlert);

// Function to compute the distance between two objects, given their Lon/Lat
double
appUtil_haversineDist (double lat_a, double lon_a, double lat_b, double lon_b)
{
  // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
  // return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
  double distance =
      12742000.0 *
      asin (sqrt (sin (DEG_2_RAD (lat_b - lat_a) / 2) * sin (DEG_2_RAD (lat_b - lat_a) / 2) +
                  cos (DEG_2_RAD (lat_a)) * cos (DEG_2_RAD (lat_b)) *
                      sin (DEG_2_RAD (lon_b - lon_a) / 2) * sin (DEG_2_RAD (lon_b - lon_a) / 2)));
  return distance;
}

// Function to compute the absolute difference between two angles (angles must be between -180 and 180)
double
appUtil_angDiff (double ang1, double ang2)
{
  double angDiff;
  angDiff = ang1 - ang2;

  if (angDiff > 180)
    {
      angDiff -= 360;
    }
  else if (angDiff < -180)
    {
      angDiff += 360;
    }
  return abs (angDiff);
}

TypeId
emergencyVehicleAlert::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::emergencyVehicleAlert")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<emergencyVehicleAlert> ()
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&emergencyVehicleAlert::m_real_time),
                         MakeBooleanChecker ())
          .AddAttribute ("IpAddr", "IpAddr", Ipv4AddressValue ("10.0.0.1"),
                         MakeIpv4AddressAccessor (&emergencyVehicleAlert::m_ipAddress),
                         MakeIpv4AddressChecker ())
          .AddAttribute (
              "PrintSummary", "To print summary at the end of simulation", BooleanValue (false),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_print_summary), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&emergencyVehicleAlert::m_csv_name),
                         MakeStringChecker ())
          .AddAttribute ("Model", "Physical and MAC layer communication model", StringValue (""),
                         MakeStringAccessor (&emergencyVehicleAlert::m_model), MakeStringChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&emergencyVehicleAlert::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&emergencyVehicleAlert::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute (
              "SendCAM", "To enable/disable the transmission of CAM messages", BooleanValue (true),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cam), MakeBooleanChecker ())
          .AddAttribute (
              "SendCPM", "To enable/disable the transmission of CPM messages", BooleanValue (true),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cpm), MakeBooleanChecker ())
          .AddAttribute (
              "sim_type", "Type of simulation: AI_mode or Random_mode", StringValue ("AI_mode"),
              MakeStringAccessor (&emergencyVehicleAlert::sim_type), MakeStringChecker ())
          .AddAttribute ("attack_min_duration", "Minimum duration of the attack in seconds",
                         DoubleValue (0.0),
                         MakeDoubleAccessor (&emergencyVehicleAlert::attack_min_duration),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("attack_max_duration", "Maximum duration of the attack in seconds",
                         DoubleValue (10.0),
                         MakeDoubleAccessor (&emergencyVehicleAlert::attack_max_duration),
                         MakeDoubleChecker<double> ())
          .AddAttribute ("random_index", "Random index for the random mode attack",
                         IntegerValue (1),
                         MakeIntegerAccessor (&emergencyVehicleAlert::random_index),
                         MakeIntegerChecker<int> (0));
  return tid;
}

emergencyVehicleAlert::emergencyVehicleAlert ()
{
  NS_LOG_FUNCTION (this);
  m_client = nullptr;
  m_print_summary = true;
  m_already_print = false;
  m_send_cam = false;
  m_send_denm = true;
  is_monitoring = false;

  m_denm_sent = 0;
  m_cam_received = 0;
  m_cpm_received = 0;
  m_denm_received = 0;
  m_denm_intertime = 0;

  m_distance_threshold =
      0; // Distance used in GeoNet to determine the radius of the circumference arounf the emergency vehicle where the DENMs are valid
  m_heading_threshold =
      45; // Max heading angle difference between the normal vehicles and the emergenecy vehicle, that triggers a reaction in the normal vehicles
}

emergencyVehicleAlert::~emergencyVehicleAlert ()
{
  NS_LOG_FUNCTION (this);
}

void
emergencyVehicleAlert::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
emergencyVehicleAlert::vehicleTypeInit ()
{
  std::ifstream f ("/home/mcalab/Desktop/Andy/VaN3Twin/ns-3-dev/src/automotive/examples/"
                   "sumo_files_v2v_map/vehicle_types.json");

  // 檢查檔案是否成功開啟
  if (!f.is_open ())
    {
      std::cerr << "Cannot open file vehicle_types.json" << std::endl;
      std::cerr << "無法開啟檔案 vehicle_types.json" << std::endl;
      return;
    }

  // 2. 解析 JSON
  json data;
  try
    {
      data = json::parse (f);
    }
  catch (json::parse_error &e)
    {
      std::cerr << "JSON parse error in vehicleTypeInit: " << e.what () << std::endl;
      return;
    }
  catch (const std::exception &e)
    {
      std::cerr << "Unexpected error during JSON parsing: " << e.what () << std::endl;
      return;
    }

  try
    {
      attacker_id = data["attacker_id"];
    }
  catch (const std::exception &e)
    {
      std::cout << "[WARN] Failed to read attacker_id from JSON: " << e.what () << std::endl;
      attacker_id = "";
    }

  try
    {
      for (const auto &ev : data["emergency_vehicles"])
        {
          std::string id = ev["id"];
          veh_set.insert (id);
        }
    }
  catch (const std::exception &e)
    {
      std::cout << "[WARN] Failed to read emergency_vehicles from JSON: " << e.what () << std::endl;
    }

  std::cout << "[INFO] vehicleTypeInit completed. Attacker: " << attacker_id
            << ", Emergency vehicles: " << veh_set.size () << std::endl;
}

void
emergencyVehicleAlert::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  /*
     * In this example, the vehicle can be either of type "passenger" or of type "emergency" (see cars.rou.xml in SUMO folder inside examples/sumo_files_v2v_map)
     * All the vehicles broadcast CAM messages. When a "passenger" car receives a CAM from an "emergency" vehicle, it checks the distance between them and
     * the difference in heading, and if it considers it to be close, it takes proper actions to facilitate the takeover maneuver.
     */

  /* Save the vehicles informations */
  // libsumo::TraCIPosition testPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (1239, 41);
  // std::cout << "座標 (1239, 41) 轉換為經緯度: Longitude=" << testPos.x << ", Latitude=" << testPos.y
  //           << std::endl;
  vehicleTypeInit ();
  m_id = m_client->GetVehicleId (this->GetNode ());

  try
    {
      m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
      m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);

      string LaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);
      string vehicleType = m_client->TraCIAPI::vehicle.getTypeID (m_id);

      m_client->TraCIAPI::vehicletype.setMinGap (vehicleType, 0.1);
      m_client->TraCIAPI::vehicletype.setTau (vehicleType, 0.1);
      m_client->TraCIAPI::vehicletype.setDecel (vehicleType, 6.0);
      m_client->TraCIAPI::vehicletype.setAccel (vehicleType, 3.0);
      double currentMinGap = m_client->TraCIAPI::vehicle.getMinGap (m_id);
    }
  catch (const std::exception &e)
    {
      std::cerr << "Failed to initialize vehicle " << m_id << ": " << e.what () << std::endl;
      return;
    }

  VDP *traci_vdp = new VDPTraCI (m_client, m_id);
  //Create LDM and sensor object
  m_LDM = CreateObject<LDM> ();
  m_LDM->setStationID (m_id);
  m_LDM->setTraCIclient (m_client);
  m_LDM->setVDP (traci_vdp);

  m_sensor = CreateObject<SUMOSensor> ();
  m_sensor->setStationID (m_id);
  m_sensor->setTraCIclient (m_client);
  m_sensor->setVDP (traci_vdp);
  m_sensor->setLDM (m_LDM);

  // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_denService.setBTP (m_btp);
  m_caService.setBTP (m_btp);
  m_cpService.setBTP (m_btp);
  m_caService.setLDM (m_LDM);
  m_cpService.setLDM (m_LDM);

  /* Create the Sockets for TX and RX */
  TypeId tid;
  if (m_model == "80211p")
    tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  else if (m_model == "cv2x" || m_model == "nrv2x")
    tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  else
    NS_FATAL_ERROR (
        "No communication model set - check simulation script - valid models: '80211p' or 'lte'");
  m_socket = Socket::CreateSocket (GetNode (), tid);

  if (m_model == "80211p")
    {
      /* Bind the socket to local address */
      PacketSocketAddress local = getGNAddress (GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress ());
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
      // Set the socketAddress for broadcast
      PacketSocketAddress remote = getGNAddress (GetNode ()->GetDevice (0)->GetIfIndex (),
                                                 GetNode ()->GetDevice (0)->GetBroadcast ());
      m_socket->Connect (remote);
    }
  else // m_model=="cv2x"
    {
      /* The C-V2X model requires the socket to be bind to "any" IPv4 address, and to be connected to the
         * IP address of the transmitting node. Then, the model will take care of broadcasting the packets.
        */
      if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
        }
      m_socket->Connect (InetSocketAddress (m_ipAddress, 19));
    }

  /* Set Station Type in DENBasicService */
  StationType_t stationtype;
  stationtype = StationType_passengerCar;
  if (m_id == attacker_id)
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 232; // Red
      alertColor.g = 126; // Green
      alertColor.b = 4; // Blue
      alertColor.a = 255; // Alpha (不透明)
      m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
      m_type = "Attacker";
      m_send_cam = false;
      QueryAllVehiclesAndLeaders ();
      AttackerProcedureTrigger ();
    }
  else if (veh_set.count (m_id) == 0)
    {
      Simulator::Schedule (Seconds (attack_procedure_start_time + 1),
                           &emergencyVehicleAlert::monitorVehicleRoutePeriodic, this);
    }

  /* Set sockets, callback and station properties in DENBasicService */
  m_denService.setSocketTx (m_socket);
  m_denService.setSocketRx (m_socket);
  m_denService.setStationProperties (stol (m_id.substr (3)), (long) stationtype);
  m_denService.addDENRxCallback (
      bind (&emergencyVehicleAlert::receiveDENM, this, placeholders::_1, placeholders::_2));
  m_denService.setRealTime (m_real_time);

  /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.setStationProperties (stol (m_id.substr (3)), (long) stationtype);
  m_caService.addCARxCallback (
      bind (&emergencyVehicleAlert::receiveCAM, this, placeholders::_1, placeholders::_2));
  m_caService.setRealTime (m_real_time);

  /* Set sockets, callback, station properties and TraCI VDP in CPBasicService */
  m_cpService.setSocketTx (m_socket);
  m_cpService.setSocketRx (m_socket);
  m_cpService.setStationProperties (stol (m_id.substr (3)), (long) stationtype);
  m_cpService.addCPRxCallback (
      bind (&emergencyVehicleAlert::receiveCPM, this, placeholders::_1, placeholders::_2));
  m_cpService.setRealTime (m_real_time);
  m_cpService.setTraCIclient (m_client);

  /* IF CPMv1 facility is needed
    m_cpService_v1.setBTP (m_btp);
    m_cpService_v1.setLDM(m_LDM);
    m_cpService_v1.setSocketTx (m_socket);
    m_cpService_v1.setSocketRx (m_socket);
    m_cpService_v1.setVDP(traci_vdp);
    m_cpService_v1.setTraCIclient(m_client);
    m_cpService_v1.setRealTime(m_real_time);
    m_cpService_v1.setStationProperties(stol(m_id.substr (3)), (long)stationtype);
    m_cpService_v1.addCPRxCallback(bind(&emergencyVehicleAlert::receiveCPMV1,this,placeholders::_1,placeholders::_2));
    m_cpService_v1.startCpmDissemination ();
    */

  /* Set TraCI VDP for GeoNet object */
  m_caService.setVDP (traci_vdp);
  m_denService.setVDP (traci_vdp);
  m_cpService.setVDP (traci_vdp);

  /* Schedule CAM dissemination */
  if (m_send_cam == true)
    {

      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();

      m_caService.startCamDissemination (desync);
    }
  if (veh_set.count (m_id) > 0 && m_send_denm == true)
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 128;
      alertColor.g = 0;
      alertColor.b = 128;
      alertColor.a = 255;

      m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
      // std::cout << "[Debug] " << m_id << " color set to alert color." << std::endl;
      TriggerEmergencyDenm ();
    }

  /* Schedule CPM dissemination */
  if (m_send_cpm == true)
    {
      m_cpService.startCpmDissemination ();
    }

  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name + "-" + m_id + "-CAM.csv", ofstream::trunc);
      m_csv_ofstream_cam
          << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << endl;
    }
}

void
emergencyVehicleAlert::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_speed_ev);
  Simulator::Cancel (m_send_cam_ev);
  Simulator::Cancel (m_update_denm_ev);
  Simulator::Cancel (m_range_check_ev);
  Simulator::Cancel (m_attacker_procedure_ev);

  uint64_t cam_sent, cpm_sent;

  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.close ();
    }

  cam_sent = m_caService.terminateDissemination ();
  cpm_sent = m_cpService.terminateDissemination ();
  m_denService.cleanup ();
  m_LDM->cleanup ();
  m_sensor->cleanup ();

  if (m_print_summary && !m_already_print)
    {
      cout << "INFO-" << m_id << ",CAM-SENT:" << cam_sent << ",CAM-RECEIVED:" << m_cam_received
           << ",CPM-SENT: " << cpm_sent << ",CPM-RECEIVED: " << m_cpm_received << endl;
      m_already_print = true;
    }
  Simulator::Cancel (m_updateDenmEvent);
}

void
emergencyVehicleAlert::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
emergencyVehicleAlert::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  std::string vehicle_id = "veh" + std::to_string (asn1cpp::getField (cam->header.stationId, long));
  double sender_lat =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.latitude, double) /
      DOT_ONE_MICRO;
  double sender_lon =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.longitude,
                         double) /
      DOT_ONE_MICRO;
  if (m_id == attacker_id)
    {
      double speed_mps =
          asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                 .basicVehicleContainerHighFrequency.speed.speedValue,
                             double) /
          CENTI;
      double acceleration_mps2 =
          asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                 .basicVehicleContainerHighFrequency.longitudinalAcceleration.value,
                             double) /
          DECI;
      int heading = static_cast<int> (
          asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                 .basicVehicleContainerHighFrequency.heading.headingValue,
                             double) /
          DECI);
      long timestamp_ms = Simulator::Now ().GetMilliSeconds ();

      int lane_ID = -1;
      std::string leaderID = "";
      double gap = -1.0;

      try
        {
          std::pair<std::string, double> leaderInfo =
              m_client->TraCIAPI::vehicle.getLeader (vehicle_id, denm_transmit_distance);
          leaderID = leaderInfo.first;
          gap = leaderInfo.second;

          // 獲取車道索引
          lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (vehicle_id);
        }
      catch (const std::exception &e)
        {
          // 如果車輛已消失，忽略此 CAM 封包，不要崩潰
          std::cout << "[DEBUG] Sender vehicle " << vehicle_id << " no longer exists: " << e.what ()
                    << std::endl;
          return;
        }
      json CAM_json = {{"message_type", "CAM"},     {"vehicle_id", vehicle_id},
                       {"latitude", sender_lat},    {"longitude", sender_lon},
                       {"speed_mps", speed_mps},    {"acceleration_mps2", acceleration_mps2},
                       {"heading", heading},        {"lane_ID", lane_ID},
                       {"timestamp", timestamp_ms}, {"leader_ID", leaderID},
                       {"gap_to_leader", gap}};

      json_queue.push (CAM_json);
    }
}

void
emergencyVehicleAlert::receiveCPMV1 (asn1cpp::Seq<CPMV1> cpm, Address from)
{
  /* Implement CPM strategy here */
  m_cpm_received++;
  (void) from;
  bool POs_ok;
  auto PObjects = asn1cpp::getSeqOpt (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                      PerceivedObjectContainer, &POs_ok);
  if (POs_ok)
    {
      int PObjects_size =
          asn1cpp::sequenceof::getSize (cpm->cpm.cpmParameters.perceivedObjectContainer);
      for (int i = 0; i < PObjects_size; i++)
        {
          LDM::returnedVehicleData_t PO_data;
          auto PO_seq = asn1cpp::makeSeq (PerceivedObjectV1);
          PO_seq = asn1cpp::sequenceof::getSeq (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                                PerceivedObjectV1, i);
          //If PO is already in local copy of vLDM
          if (m_LDM->lookup (asn1cpp::getField (PO_seq->objectID, long), PO_data) == LDM::LDM_OK)
            {
              //Add the new perception to the LDM
              vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
              if (find (associatedCVs.begin (), associatedCVs.end (),
                        asn1cpp::getField (cpm->header.stationId, long)) == associatedCVs.end ())
                associatedCVs.push_back (asn1cpp::getField (cpm->header.stationId, long));
              PO_data.vehData.associatedCVs = OptionalDataItem<vector<long>> (associatedCVs);
              m_LDM->insert (PO_data.vehData);
            }
          else
            {
              //Translate CPM data to LDM format
              m_LDM->insert (translateCPMV1data (cpm, i));
            }
        }
    }
}

vehicleData_t
emergencyVehicleAlert::translateCPMV1data (asn1cpp::Seq<CPMV1> cpm, int objectIndex)
{
  vehicleData_t retval;
  auto PO_seq = asn1cpp::makeSeq (PerceivedObjectV1);
  using namespace boost::geometry::strategy::transform;
  PO_seq = asn1cpp::sequenceof::getSeq (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                        PerceivedObjectV1, objectIndex);
  retval.detected = true;
  retval.stationID = asn1cpp::getField (PO_seq->objectID, long);
  retval.ID = to_string (retval.stationID);
  retval.vehicleLength = asn1cpp::getField (PO_seq->planarObjectDimension1->value, long);
  retval.vehicleWidth = asn1cpp::getField (PO_seq->planarObjectDimension2->value, long);
  retval.heading = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                          .originatingVehicleContainer.heading.headingValue,
                                      double) /
                       10 +
                   asn1cpp::getField (PO_seq->yawAngle->value, double) / 10;
  if (retval.heading > 360.0)
    retval.heading -= 360.0;

  retval.speed_ms = (double) (asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                                     .originatingVehicleContainer.speed.speedValue,
                                                 long) +
                              asn1cpp::getField (PO_seq->xSpeed.value, long)) /
                    CENTI;

  double fromLon =
      asn1cpp::getField (cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude,
                         double) /
      DOT_ONE_MICRO;
  double fromLat =
      asn1cpp::getField (cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude,
                         double) /
      DOT_ONE_MICRO;

  libsumo::TraCIPosition objectPosition =
      m_client->TraCIAPI::simulation.convertLonLattoXY (fromLon, fromLat);

  point_type objPoint (asn1cpp::getField (PO_seq->xDistance.value, double) / CENTI,
                       asn1cpp::getField (PO_seq->yDistance.value, double) / CENTI);
  double fromAngle = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                            .originatingVehicleContainer.heading.headingValue,
                                        double) /
                     10;
  rotate_transformer<boost::geometry::degree, double, 2, 2> rotate (fromAngle - 90);
  boost::geometry::transform (objPoint, objPoint,
                              rotate); // Transform points to the reference (x,y) axises
  objectPosition.x += boost::geometry::get<0> (objPoint);
  objectPosition.y += boost::geometry::get<1> (objPoint);

  libsumo::TraCIPosition objectPosition2 = objectPosition;
  objectPosition =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x, objectPosition.y);

  retval.lon = objectPosition.x;
  retval.lat = objectPosition.y;

  point_type speedPoint (asn1cpp::getField (PO_seq->xSpeed.value, double) / CENTI,
                         asn1cpp::getField (PO_seq->ySpeed.value, double) / CENTI);
  boost::geometry::transform (speedPoint, speedPoint,
                              rotate); // Transform points to the reference (x,y) axises
  retval.speed_ms = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                           .originatingVehicleContainer.speed.speedValue,
                                       double) /
                        CENTI +
                    boost::geometry::get<0> (speedPoint);

  retval.camTimestamp = asn1cpp::getField (cpm->cpm.generationDeltaTime, long);
  retval.timestamp_us = Simulator::Now ().GetMicroSeconds () -
                        (asn1cpp::getField (PO_seq->timeOfMeasurement, long) * 1000);
  retval.stationType = StationType_passengerCar;
  retval.perceivedBy.setData (asn1cpp::getField (cpm->header.stationId, long));
  retval.confidence = asn1cpp::getField (PO_seq->objectConfidence, long);
  return retval;
}

void
emergencyVehicleAlert::receiveDENM (denData denm, Address from)
{

  auto mgmt = denm.getDenmMgmtData_asn_types ();
  long timestamp_ms = Simulator::Now ().GetMilliSeconds ();
  uint32_t senderId = mgmt.stationID;
  string senderVehicleId = "veh" + to_string (senderId);

  // 先检查接收者自己是否还在场景中
  string myLaneIndex;
  try
    {
      myLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);
    }
  catch (const std::exception &e)
    {
      std::cout << "[DEBUG] Receiver vehicle " << m_id << " no longer exists: " << e.what ()
                << std::endl;
      return;
    }

  string senderLaneIndex = "";
  libsumo::TraCIPosition pos;
  double speed_mps;
  double acceleration_mps2;
  double angle;
  int lane_ID;

  try
    {
      senderLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (senderVehicleId);
      pos = m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
      pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
      speed_mps = m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);
      acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (senderVehicleId);
      angle = m_client->TraCIAPI::vehicle.getAngle (senderVehicleId);
      lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (senderVehicleId);
    }
  catch (...)
    {
      std::cout << "[Debug] Exception caught when getting position of " << senderVehicleId
                << std::endl;
      return;
    }
  int heading = static_cast<int> (angle);

  json DENM_sender_json = {
      {"message_type", "DENM"}, {"vehicle_id", senderVehicleId},
      {"latitude", pos.y},      {"longitude", pos.x},
      {"speed_mps", speed_mps}, {"acceleration_mps2", acceleration_mps2},
      {"heading", heading},     {"timestamp", timestamp_ms},
      {"lane_ID", lane_ID},
  };
  json_queue.push (DENM_sender_json);

  libsumo::TraCIPosition myPos;
  try
    {
      myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      myPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);
    }
  catch (const std::exception &e)
    {
      std::cout << "[DEBUG] Failed to get receiver position: " << e.what () << std::endl;
      return;
    }
  libsumo::TraCIPosition attackerPos;

  try
    {
      attackerPos = m_client->TraCIAPI::vehicle.getPosition (attacker_id);
      attackerPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (attackerPos.x, attackerPos.y);
    }
  catch (const exception &e)
    {
      cerr << "Warning: Cannot access attacker vehicle " << attacker_id << ": " << e.what ()
           << endl;
      return;
    }
  double distance_to_attacker =
      appUtil_haversineDist (myPos.y, myPos.x, attackerPos.y, attackerPos.x);

  if (senderId == stol (m_id.substr (3)) ||
      ((senderVehicleId == victim_m_id) && (m_type != "Attacker") &&
       (distance_to_attacker <= attack_range)))
    {
      if (((myLaneIndex.back () == senderLaneIndex.back ()) &&
           (senderLaneIndex.substr (0, 2) == "E5") && (myLaneIndex.substr (0, 2) == "E0")) ||
          myLaneIndex == senderLaneIndex)
        {
          shouldEnterForkRoad = 0;
        }
      return;
    }
  else if ((myLaneIndex.back () != senderLaneIndex.back ()) && m_type != "Attacker" &&
           veh_set.count (m_id) == 0)
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 0; // Red
      alertColor.g = 255; // Green
      alertColor.b = 0; // Blue
      alertColor.a = 255; // Alpha (不透明)
      try
        {
          m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
        }
      catch (const std::exception &e)
        {
          std::cout << "[DEBUG] Failed to set vehicle color in first condition: " << e.what ()
                    << std::endl;
        }
      return;
    }
  else if (m_type == "Attacker")
    {
      return;
    }

  if (veh_set.count (m_id) == 0)
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 0; // Red
      alertColor.g = 255; // Green
      alertColor.b = 0; // Blue
      alertColor.a = 255; // Alpha (不透明)
      try
        {
          m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
        }
      catch (const std::exception &e)
        {
          NS_LOG_DEBUG ("Failed to set vehicle color in second condition: " << e.what ());
        }
    }

  std::pair<std::string, double> leaderInfo;
  try
    {
      leaderInfo = m_client->TraCIAPI::vehicle.getLeader (m_id, denm_transmit_distance);
    }
  catch (const std::exception &e)
    {
      NS_LOG_DEBUG ("Failed to get leader info for receiver: " << e.what ());
      return;
    }
  std::string leaderID = leaderInfo.first;
  if (leaderID == attacker_id)
    {
      try
        {
          leaderInfo = m_client->TraCIAPI::vehicle.getLeader (attacker_id, denm_transmit_distance);
          leaderID = leaderInfo.first;
        }
      catch (const std::exception &e)
        {
          NS_LOG_DEBUG ("Failed to get leader info for attacker: " << e.what ());
          leaderID = "";
        }
    }
  double gap = leaderInfo.second;
  double distance = appUtil_haversineDist (myPos.y, myPos.x, pos.y, pos.x);
  double safety_distance = 50;
  try
    {
      if (myLaneIndex.back () == senderLaneIndex.back () &&
          ((myLaneIndex.substr (0, 2) == "E0" && senderLaneIndex.substr (0, 2) == "E5") ||
           (myLaneIndex == senderLaneIndex)))
        {
          libsumo::TraCIColor slowdownColor;
          slowdownColor.r = 255;
          slowdownColor.g = 0;
          slowdownColor.b = 0;
          slowdownColor.a = 255;
          try
            {
              m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to set color in receiveDENM: " << e.what ());
            }

          if (shouldEnterForkRoad != 3 &&
              (myLaneIndex.back () == '1' || myLaneIndex.back () == '2'))
            {
              shouldEnterForkRoad = 1;
            }
          else if (shouldEnterForkRoad != 3 &&
                   (myLaneIndex.back () == '0' || myLaneIndex.back () == '3'))
            {
              shouldEnterForkRoad = 4;
            }
          if (distance <= safety_distance)
            {
              double distanceToSender = appUtil_haversineDist (myPos.y, myPos.x, pos.y, pos.x);
              double safety_distance = denm_transmit_distance + 10;
              double speedReduction = (safety_distance - distanceToSender) / safety_distance;
              double targetSpeed;

              double minSpeed = 2.0;
              double min_gap = 7.0;
              double ratio = (distanceToSender - min_gap) / (safety_distance - min_gap);
              targetSpeed = speed_mps * ratio;

              try
                {
                  m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
                }
              catch (const std::exception &e)
                {
                  NS_LOG_DEBUG ("Failed to set speed in receiveDENM: " << e.what ());
                }

              try
                {
                  m_client->TraCIAPI::vehicle.slowDown (m_id, speed_mps - 5, 0.5);
                }
              catch (const std::exception &e)
                {
                  NS_LOG_DEBUG ("Failed to slow down in receiveDENM: " << e.what ());
                }
            }
        }
    }
  catch (const exception &e)
    {
      cerr << "Warning: Cannot access sender vehicle " << senderVehicleId << ": " << e.what ()
           << endl;
    }
}

void
emergencyVehicleAlert::AttackerProcedureTrigger ()
{
  try
    {
      m_set_attacker_speed_ev =
          Simulator::Schedule (Seconds (1.0), &emergencyVehicleAlert::SetAttackerSpeed, this);
      m_attacker_procedure_ev =
          Simulator::Schedule (Seconds (attack_procedure_start_time),
                               &emergencyVehicleAlert::AttackerSelectVictim, this);
    }
  catch (const exception &e)
    {
      cerr << "Warning: AttackerProcedureTrigger error for vehicle " << m_id << ": " << e.what ()
           << endl;
    }
}

void
emergencyVehicleAlert::QueryAllVehiclesAndLeaders ()
{
  std::vector<std::string> vehicleIDs;
  try
    {
      vehicleIDs = m_client->TraCIAPI::vehicle.getIDList ();
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("Failed to get vehicle list: " << e.what ());
      Simulator::Remove (m_query_all_vehicles_ev);
      m_query_all_vehicles_ev = Simulator::Schedule (
          Seconds (0.1), &emergencyVehicleAlert::QueryAllVehiclesAndLeaders, this);
      return;
    }

  json vehicles_info = json::array ();
  json message;

  // 建立一個 Lambda 函數，將浮點數四捨五入到小數點後 4 位
  auto round4 = [] (double val) { return std::round (val * 10000.0) / 10000.0; };
  for (const std::string &vehicleID : vehicleIDs)
    {
      try
        {
          libsumo::TraCIPosition pos;
          try
            {
              pos = m_client->TraCIAPI::vehicle.getPosition (vehicleID);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Vehicle " << vehicleID << " disappeared while querying position");
              continue;
            }

          try
            {
              pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to convert coordinates for " << vehicleID << ": " << e.what ());
              continue;
            }

          double speed_mps;
          try
            {
              speed_mps = m_client->TraCIAPI::vehicle.getSpeed (vehicleID);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Vehicle " << vehicleID << " disappeared while querying speed");
              continue;
            }

          double acceleration_mps2;
          try
            {
              acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (vehicleID);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to get acceleration for " << vehicleID << ": " << e.what ());
              acceleration_mps2 = 0.0;
            }

          double angle;
          try
            {
              angle = m_client->TraCIAPI::vehicle.getAngle (vehicleID);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to get angle for " << vehicleID << ": " << e.what ());
              angle = 0.0;
            }

          int lane_ID;
          try
            {
              lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (vehicleID);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to get lane ID for " << vehicleID << ": " << e.what ());
              lane_ID = -1;
            }

          std::string leader_ID;
          double gap_to_leader;
          try
            {
              std::pair<std::string, double> leaderInfo =
                  m_client->TraCIAPI::vehicle.getLeader (vehicleID, 100.0);
              leader_ID = leaderInfo.first;
              gap_to_leader = leaderInfo.second;
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Failed to get leader info for " << vehicleID << ": " << e.what ());
              leader_ID = "";
              gap_to_leader = -1.0;
            }

          json vehicle_json = {{"vehicle_id", vehicleID},
                               {"latitude", round4 (pos.y)},
                               {"longitude", round4 (pos.x)},
                               {"speed_mps", round4 (speed_mps)},
                               {"acceleration_mps2", round4 (acceleration_mps2)},
                               {"heading", static_cast<int> (angle)},
                               {"lane_ID", lane_ID},
                               {"leader_ID", leader_ID},
                               {"gap_to_leader", round4 (gap_to_leader)},
                               {"timestamp", Simulator::Now ().GetMilliSeconds ()},
                               {"target_vehicle_id", victim_m_id}};

          vehicles_info.push_back (vehicle_json);
        }
      catch (const std::exception &e)
        {
          NS_LOG_ERROR ("Unexpected error processing vehicle " << vehicleID << ": " << e.what ());
        }
    }
  message = {{"type", "query_all_vehicles"}, {"vehicles_info", vehicles_info}};
  std::string json_string = message.dump ();
  char *res;
  try
    {
      res = socketClientFunction (json_string.c_str ());
      Simulator::Remove (m_query_all_vehicles_ev);
      m_query_all_vehicles_ev = Simulator::Schedule (
          Seconds (0.1), &emergencyVehicleAlert::QueryAllVehiclesAndLeaders, this);
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("socketClientFunction failed: " << e.what ());
      m_query_all_vehicles_ev = Simulator::Schedule (
          Seconds (0.1), &emergencyVehicleAlert::QueryAllVehiclesAndLeaders, this);
    }
}

void
emergencyVehicleAlert::SetAttackerSpeed ()
{
  string target_id;
  if (victim_m_id != "")
    {
      target_id = victim_m_id;
    }
  else
    {
      if (veh_set.empty ())
        {
          NS_LOG_WARN ("No target vehicle available - veh_set is empty");
          return;
        }
      target_id = veh_set.begin ()->c_str ();
    }
  try
    {
      // 檢查目標車輛是否存在(檢查一次即可)
      try
        {
          m_client->TraCIAPI::vehicle.getSpeed (target_id);
        }
      catch (...)
        {
          NS_LOG_WARN ("Target vehicle '" << target_id << "' no longer exists in SUMO");
          victim_m_id = "";
          Simulator::Remove (m_set_attacker_speed_ev);
          return;
        }

      libsumo::TraCIPosition myPos;
      try
        {
          myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get attacker position: " << e.what ());
          return;
        }

      libsumo::TraCIPosition myGeo;
      try
        {
          myGeo = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to convert attacker coordinates: " << e.what ());
          return;
        }

      libsumo::TraCIPosition vicPos;
      try
        {
          vicPos = m_client->TraCIAPI::vehicle.getPosition (target_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get victim position: " << e.what ());
          victim_m_id = "";
          Simulator::Remove (m_set_attacker_speed_ev);
          return;
        }

      libsumo::TraCIPosition vicGeo;
      try
        {
          vicGeo = m_client->TraCIAPI::simulation.convertXYtoLonLat (vicPos.x, vicPos.y);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to convert victim coordinates: " << e.what ());
          return;
        }

      double dist = appUtil_haversineDist (myGeo.y, myGeo.x, vicGeo.y, vicGeo.x);

      double my_lane_pos;
      try
        {
          my_lane_pos = m_client->TraCIAPI::vehicle.getLanePosition (m_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get attacker lane position: " << e.what ());
          return;
        }

      double vic_lane_pos;
      try
        {
          vic_lane_pos = m_client->TraCIAPI::vehicle.getLanePosition (target_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get victim lane position: " << e.what ());
          victim_m_id = "";
          return;
        }

      string my_edge;
      try
        {
          my_edge = m_client->TraCIAPI::vehicle.getRoadID (m_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get attacker road ID: " << e.what ());
          return;
        }

      string vic_edge;
      try
        {
          vic_edge = m_client->TraCIAPI::vehicle.getRoadID (target_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get victim road ID: " << e.what ());
          victim_m_id = "";
          return;
        }

      bool is_overtaking = (my_edge == vic_edge) && (my_lane_pos > vic_lane_pos);

      double vic_speed;
      try
        {
          vic_speed = m_client->TraCIAPI::vehicle.getSpeed (target_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get victim speed: " << e.what ());
          victim_m_id = "";
          return;
        }

      double target_gap = 5.0;
      double tolerance = 3.0;
      double catch_up_speed = 5.0;
      double fall_back_factor = 0.7;

      double target_speed;
      if (dist > target_gap + tolerance)
        {
          target_speed = std::min (m_max_speed, vic_speed + catch_up_speed);
          if (target_speed < 2.0)
            target_speed = 5.0;
        }
      else if (dist < target_gap - tolerance)
        {
          target_speed = vic_speed * fall_back_factor;
          if (target_speed < 0)
            target_speed = 0;
        }
      else
        {
          target_speed = vic_speed;
        }

      try
        {
          m_client->TraCIAPI::vehicle.setSpeed (m_id, target_speed);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to set attacker speed: " << e.what ());
          return;
        }

      Simulator::Remove (m_set_attacker_speed_ev);
      m_set_attacker_speed_ev =
          Simulator::Schedule (Seconds (0.5), &emergencyVehicleAlert::SetAttackerSpeed, this);
    }
  catch (...)
    {
      std::cout << "[Debug] Target " << target_id << " lost in SetAttackerSpeed. Stopping pursuit."
                << std::endl;
      Simulator::Remove (m_set_attacker_speed_ev);
    }
}

void
emergencyVehicleAlert::AttackerSelectVictim ()
{
  int DENM_cnt = 0;
  long timestamp_ms = Simulator::Now ().GetMilliSeconds ();
  float attack_duration = 0.0;
  int denm_veh_speed;
  json json_array = json::array ();
  json message;
  std::string json_string;
  unordered_map<string, json> vehicle_data_map;
  char *res;
  while (!json_queue.empty ())
    {
      if ((timestamp_ms - 200 <= json_queue.front ()["timestamp"] &&
           json_queue.front ()["timestamp"] <= timestamp_ms))
        {

          vehicle_data_map[json_queue.front ()["vehicle_id"].get<string> () +
                           json_queue.front ()["message_type"].get<string> ()] =
              json_queue.front ();
        }
      json_queue.pop ();
    }
  for (auto &pair : vehicle_data_map)
    {
      json_array.push_back (pair.second);
    }

  libsumo::TraCIPosition pos;
  double speed_mps;
  double acceleration_mps2;
  double angle;
  int heading;

  try
    {
      pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

      speed_mps = m_client->TraCIAPI::vehicle.getSpeed (m_id);

      acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (m_id);

      angle = m_client->TraCIAPI::vehicle.getAngle (m_id);
      heading = static_cast<int> (angle);
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("Attacker vehicle "
                   << m_id << " no longer exists in AttackerSelectVictim: " << e.what ());
      Simulator::Remove (m_attacker_procedure_ev);
      return;
    }

  message = {{"type", "attacker_query"},
             {"attacker_info",
              {
                  {"latitude", pos.y},
                  {"longitude", pos.x},
                  {"speed_mps", speed_mps},
                  {"acceleration_mps2", acceleration_mps2},
                  {"heading", heading},
              }},
             {"received_data", json_array}};
  json_string = message.dump ();
  try
    {
      if (sim_type == "AI_mode")
        {
          // std::cout << "AI_mode attack launched at time " << timestamp_ms << std::endl;
          res = socketClientFunction (json_string.c_str ());
          json res_json = json::parse (res);
          victim_m_id = res_json["vehicle_id"].get<string> ();
          attack_duration = res_json["attack_duration_seconds"].get<float> ();
        }
      else if (sim_type == "Random_mode")
        {
          // std::cout << "Random_mode attack launched at time " << timestamp_ms << std::endl;
          std::vector<std::string> veh_vector (veh_set.begin (), veh_set.end ());
          // std::random_device rd;
          // std::mt19937 gen (rd ());
          // std::uniform_int_distribution<> dis (0, veh_vector.size () - 1);
          // int random_index = dis (gen);
          // int random_index = 3;
          victim_m_id = veh_vector[random_index];
          // float min_duration = attack_min_duration;
          // float max_duration = attack_max_duration;
          // std::uniform_real_distribution<float> duration_dis (min_duration, max_duration);

          // attack_duration = duration_dis (gen);
          // attack_duration = 20.0;

          attack_duration = attack_min_duration;
        }
      std::cout << "Attacker " << m_id << " selected victim " << victim_m_id << " for duration "
                << attack_duration << " seconds." << std::endl;
      m_reset_victim_ev = Simulator::Schedule (Seconds (attack_duration),
                                               &emergencyVehicleAlert::reset_victim_status, this);
      Simulator::Remove (m_attacker_procedure_ev);
      return;
    }
  catch (...)
    {
      std::cout << "error!" << std::endl;
      Simulator::Remove (m_attacker_procedure_ev);
      // m_attacker_procedure_ev =
      //     Simulator::Schedule (Seconds (1.0), &emergencyVehicleAlert::AttackerSelectVictim, this);
      return;
    }
}

void
emergencyVehicleAlert::reset_victim_status ()
{
  victim_m_id = "";
  return;
}

void
emergencyVehicleAlert::CheckDistanceAndRestore (string senderVehicleId)
{
  m_monitored_vehicle_id = senderVehicleId;
  try
    {
      // 檢查發送者和接收者是否還在場景中
      try
        {
          m_client->TraCIAPI::vehicle.getSpeed (m_id);
          m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Vehicle missing in CheckDistanceAndRestore: " << e.what ());
          m_monitored_vehicle_id = "";
          Simulator::Remove (m_speed_ev);
          return;
        }

      libsumo::TraCIPosition myPos;
      try
        {
          myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
          myPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get own position: " << e.what ());
          return;
        }

      double currentSpeed;
      try
        {
          currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get own speed: " << e.what ());
          return;
        }

      libsumo::TraCIPosition senderPos;
      try
        {
          senderPos = m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
          senderPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (senderPos.x, senderPos.y);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get sender position: " << e.what ());
          m_monitored_vehicle_id = "";
          Simulator::Remove (m_speed_ev);
          return;
        }

      double senderSpeed;
      try
        {
          senderSpeed = m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get sender speed: " << e.what ());
          m_monitored_vehicle_id = "";
          return;
        }

      string senderLaneIndex;
      try
        {
          senderLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (senderVehicleId);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get sender lane: " << e.what ());
          m_monitored_vehicle_id = "";
          return;
        }

      string myLaneIndex;
      try
        {
          myLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);
        }
      catch (const std::exception &e)
        {
          NS_LOG_WARN ("Failed to get own lane: " << e.what ());
          return;
        }

      if (senderLaneIndex.back () != myLaneIndex.back () || currentSpeed <= denm_veh_min_speed)
        {
          RestoreSpeed (senderVehicleId);
          return;
        }

      double distanceToSender = appUtil_haversineDist (myPos.y, myPos.x, senderPos.y, senderPos.x);
      double safety_distance = 30;

      if (distanceToSender >= safety_distance)
        {
          RestoreSpeed (senderVehicleId);
        }
      else
        {
          libsumo::TraCIColor slowdownColor;
          slowdownColor.r = 255;
          slowdownColor.g = 0;
          slowdownColor.b = 0;
          slowdownColor.a = 255;
          double targetSpeed;
          double safety_distance = cam_transmit_distance;
          double min_gap = 7.0;
          if (distanceToSender < min_gap)
            {
              targetSpeed = 0.0;
              try
                {
                  m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor);
                }
              catch (const std::exception &e)
                {
                  NS_LOG_WARN ("Failed to set vehicle color: " << e.what ());
                }
            }
          else
            {
              double ratio = (distanceToSender - min_gap) / (safety_distance - min_gap);
              targetSpeed = senderSpeed * ratio;

              if (targetSpeed < 0)
                targetSpeed = 0;

              try
                {
                  m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor);
                }
              catch (const std::exception &e)
                {
                  NS_LOG_WARN ("Failed to set vehicle color: " << e.what ());
                }
            }

          try
            {
              m_client->TraCIAPI::vehicle.slowDown (m_id, targetSpeed, 0.5);
            }
          catch (const std::exception &e)
            {
              NS_LOG_WARN ("Failed to slow down vehicle: " << e.what ());
              m_monitored_vehicle_id = "";
              return;
            }

          Simulator::Remove (m_speed_ev);
          m_speed_ev =
              Simulator::Schedule (Seconds (0.5), &emergencyVehicleAlert::CheckDistanceAndRestore,
                                   this, senderVehicleId);
        }
    }
  catch (const exception &e)
    {
      NS_LOG_ERROR ("Unexpected error in CheckDistanceAndRestore: " << e.what ());
      m_monitored_vehicle_id = "";
      Simulator::Remove (m_speed_ev);
    }
}

void
emergencyVehicleAlert::TriggerEmergencyDenm ()
{
  denData data;
  DEN_ActionID_t actionid;
  DENBasicService_error_t trigger_retval;
  libsumo::TraCIPosition pos;
  try
    {
      pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
    }
  catch (const std::exception &e)
    {
      NS_LOG_ERROR ("Failed to get position for DENM trigger: " << e.what ());
      return;
    }

  /* Set DENM mandatpry fields */
  data.setDenmMandatoryFields (compute_timestampIts (m_real_time), pos.x, pos.y);
  denData::denDataSituation situation_data;

  situation_data.causeCode = 97;
  situation_data.subCauseCode = 0;

  data.setDenmSituationData_asn_types (situation_data);

  GeoArea_t geoArea;
  geoArea.posLong = pos.x * DOT_ONE_MICRO;
  geoArea.posLat = pos.y * DOT_ONE_MICRO;
  geoArea.distA = denm_transmit_distance;
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  try
    {
      trigger_retval = m_denService.appDENM_trigger (data, actionid);
      if (trigger_retval == DENM_NO_ERROR)
        {
          m_updateDenmEvent = Simulator::Schedule (
              Seconds (0.1), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionid);
        }
      else
        {
          NS_LOG_ERROR ("DENM trigger failed with error: " << trigger_retval);
          std::cout << "[Debug] DENM trigger failed with error: " << trigger_retval << std::endl;
        }
    }
  catch (const std::exception &e)
    {
      NS_LOG_ERROR ("Exception during DENM trigger: " << e.what ());
    }
}

void
emergencyVehicleAlert::UpdateEmergencyDenm (DEN_ActionID_t actionID)
{
  denData data;
  denData::denDataSituation situation_data;
  DENBasicService_error_t update_retval;

  int denm_veh_speed;
  try
    {
      denm_veh_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("Failed to get speed for update: " << e.what ());
      return;
    }

  try
    {
      if (denm_veh_speed + denm_veh_accel > denm_veh_min_speed)
        {
          m_client->TraCIAPI::vehicle.slowDown (m_id, denm_veh_speed + denm_veh_accel, 0.1);
        }
      else
        {
          denm_veh_speed = denm_veh_max_speed;
          m_client->TraCIAPI::vehicle.slowDown (m_id, denm_veh_speed, 0.5);
        }
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("Failed to adjust vehicle speed: " << e.what ());
    }

  libsumo::TraCIPosition pos;
  try
    {
      pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
    }
  catch (const std::exception &e)
    {
      NS_LOG_WARN ("Failed to get position for DENM update: " << e.what ());
      return;
    }

  data.setDenmMandatoryFields (compute_timestampIts (m_real_time), pos.y, pos.x);
  situation_data.causeCode = 97;
  situation_data.subCauseCode = 0;
  data.setDenmSituationData_asn_types (situation_data);
  GeoArea_t geoArea;
  geoArea.posLong = pos.x * DOT_ONE_MICRO;
  geoArea.posLat = pos.y * DOT_ONE_MICRO;
  geoArea.distA = denm_transmit_distance;
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  try
    {
      update_retval = m_denService.appDENM_update (data, actionID);
      if (update_retval != DENM_NO_ERROR)
        {
          NS_LOG_ERROR ("Failed to update DENM. Error code: " << update_retval);
          std::cout << "[Debug] DENM update failed with error: " << update_retval << std::endl;
          return;
        }

      // std::cout << "Vehicle " << m_id << " triggered emergency DENM." << std::endl;
      m_updateDenmEvent = Simulator::Schedule (
          Seconds (0.1), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionID);
    }
  catch (const std::exception &e)
    {
      NS_LOG_ERROR ("Exception during DENM update: " << e.what ());
    }
}

void
emergencyVehicleAlert::RestoreSpeed (string senderVehicleId)
{
  libsumo::TraCIColor normal;
  normal.r = 0;
  normal.g = 225;
  normal.b = 255;
  normal.a = 255;
  try
    {
      m_client->TraCIAPI::vehicle.setColor (m_id, normal);
    }
  catch (const std::exception &e)
    {
      NS_LOG_DEBUG ("Failed to set vehicle color: " << e.what ());
    }
  try
    {
      m_client->TraCIAPI::vehicle.slowDown (m_id, m_max_speed, 5);
    }
  catch (const std::exception &e)
    {
      NS_LOG_DEBUG ("Failed to restore speed: " << e.what ());
    }
  m_monitored_vehicle_id = "";
}

void
emergencyVehicleAlert::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
{
  /* Implement CPM strategy here */
  m_cpm_received++;
  (void) from;
  //For every PO inside the CPM, if any
  bool POs_ok;
  int wrappedContainer_size = asn1cpp::sequenceof::getSize (cpm->payload.cpmContainers);
  for (int i = 0; i < wrappedContainer_size; i++)
    {
      auto wrappedContainer =
          asn1cpp::sequenceof::getSeq (cpm->payload.cpmContainers, WrappedCpmContainer, i);
      WrappedCpmContainer__containerData_PR present = asn1cpp::getField (
          wrappedContainer->containerData.present, WrappedCpmContainer__containerData_PR);
      if (present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
        {
          auto POcontainer =
              asn1cpp::getSeq (wrappedContainer->containerData.choice.PerceivedObjectContainer,
                               PerceivedObjectContainer);
          int PObjects_size = asn1cpp::sequenceof::getSize (POcontainer->perceivedObjects);
          for (int j = 0; j < PObjects_size; j++)
            {
              LDM::returnedVehicleData_t PO_data;
              auto PO_seq = asn1cpp::makeSeq (PerceivedObject);
              PO_seq =
                  asn1cpp::sequenceof::getSeq (POcontainer->perceivedObjects, PerceivedObject, j);
              //If PO is already in local copy of vLDM
              if (m_LDM->lookup (asn1cpp::getField (PO_seq->objectId, long), PO_data) ==
                  LDM::LDM_OK)
                {
                  //Add the new perception to the LDM
                  vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
                  if (find (associatedCVs.begin (), associatedCVs.end (),
                            asn1cpp::getField (cpm->header.stationId, long)) ==
                      associatedCVs.end ())
                    associatedCVs.push_back (asn1cpp::getField (cpm->header.stationId, long));
                  PO_data.vehData.associatedCVs = OptionalDataItem<vector<long>> (associatedCVs);
                  m_LDM->insert (PO_data.vehData);
                }
              else
                {
                  //Translate CPM data to LDM format
                  m_LDM->insert (translateCPMdata (cpm, PO_seq, j));
                }
            }
        }
    }
}
vehicleData_t
emergencyVehicleAlert::translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                         asn1cpp::Seq<PerceivedObject> object, int objectIndex)
{
  vehicleData_t retval;
  retval.detected = true;
  retval.stationID = asn1cpp::getField (object->objectId, long);
  retval.ID = to_string (retval.stationID);
  retval.vehicleLength = asn1cpp::getField (object->objectDimensionX->value, long);
  retval.vehicleWidth = asn1cpp::getField (object->objectDimensionY->value, long);
  retval.heading = asn1cpp::getField (object->angles->zAngle.value, double) / DECI;
  retval.xSpeedAbs.setData (
      asn1cpp::getField (object->velocity->choice.cartesianVelocity.xVelocity.value, long));
  retval.xSpeedAbs.setData (
      asn1cpp::getField (object->velocity->choice.cartesianVelocity.yVelocity.value, long));
  retval.speed_ms =
      (sqrt (pow (retval.xSpeedAbs.getData (), 2) + pow (retval.ySpeedAbs.getData (), 2))) / CENTI;

  libsumo::TraCIPosition fromPosition = m_client->TraCIAPI::simulation.convertLonLattoXY (
      asn1cpp::getField (cpm->payload.managementContainer.referencePosition.longitude, double) /
          DOT_ONE_MICRO,
      asn1cpp::getField (cpm->payload.managementContainer.referencePosition.latitude, double) /
          DOT_ONE_MICRO);
  libsumo::TraCIPosition objectPosition = fromPosition;
  objectPosition.x += asn1cpp::getField (object->position.xCoordinate.value, long) / CENTI;
  objectPosition.y += asn1cpp::getField (object->position.yCoordinate.value, long) / CENTI;
  objectPosition =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x, objectPosition.y);
  retval.lon = objectPosition.x;
  retval.lat = objectPosition.y;

  retval.camTimestamp = asn1cpp::getField (cpm->payload.managementContainer.referenceTime, long);
  retval.timestamp_us = Simulator::Now ().GetMicroSeconds () -
                        (asn1cpp::getField (object->measurementDeltaTime, long) * 1000);
  retval.stationType = StationType_passengerCar;
  retval.perceivedBy.setData (asn1cpp::getField (cpm->header.stationId, long));

  return retval;
} // namespace ns3

void
emergencyVehicleAlert::SwitchToSideRoad ()
{
  try
    {
      // 1. 取得目前所在的 Edge ID
      std::string roadID = m_client->TraCIAPI::vehicle.getRoadID (m_id);
      string LaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);

      // 2. 定義新的目標 Edge (根據 map.net.xml)
      std::string targetEdge = "";
      int targetLane = -1;
      // std::cout << "Current Road ID: " << roadID << ", Lane Index: " << LaneIndex << std::endl;
      // 如果在 E0 (準備經過 J1)，設定目標為 E2 (右轉) 或 E1 (左轉)
      // if (roadID == "E0" && LaneIndex == "E0_0")
      //   {
      //     targetEdge = "E2"; // 這裡設為右轉 E2，若想左轉改 E1
      //   }
      if (roadID == "E0" && LaneIndex == "E0_1")
        {
          targetLane = 0;
        }
      else if (roadID == "E0" && LaneIndex == "E0_2")
        {
          targetLane = 3; // 這裡設為左轉 E1，若想右轉改 E2
        }
      // else if (roadID == "E0" && LaneIndex == "E0_3")
      //   {
      //     targetEdge = "E1"; // 這裡設為左轉 E1，若想右轉改 E2
      //   }

      // 3. 如果找到了合適的叉路，執行改道
      // if (targetEdge != "")
      //   {
      //     // 改變車輛目的地 (SUMO 會自動計算路徑並換車道)
      //     m_client->TraCIAPI::vehicle.changeTarget (m_id, targetEdge);
      //     std::cout << "Vehicle " << m_id << " switching to side road: " << targetEdge << std::endl;
      //     // // (選用) 將車輛變色為黃色，表示正在避讓/改道
      //     // libsumo::TraCIColor forkColor;
      //     // forkColor.r = 255; forkColor.g = 255; forkColor.b = 0; forkColor.a = 255;
      //     // m_client->TraCIAPI::vehicle.setColor (m_id, forkColor);
      //     shouldEnterForkRoad = 3;
      //   }
      // else if (targetLane != -1)
      if (targetLane != -1)
        {
          m_client->TraCIAPI::vehicle.changeLane (m_id, targetLane, 10.0);
          shouldEnterForkRoad = 2;
        }
    }
  catch (const std::exception &e)
    {
      std::cerr << "Error in SwitchToSideRoad for " << m_id << ": " << e.what () << std::endl;
    }
}

void
emergencyVehicleAlert::SwitchToForkRoad ()
{
  try
    {
      // 1. 取得目前所在的 Edge ID
      std::string roadID = m_client->TraCIAPI::vehicle.getRoadID (m_id);
      string LaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);

      // 2. 定義新的目標 Edge (根據 map.net.xml)
      std::string targetEdge = "";
      // std::cout << "Current Road ID: " << roadID << ", Lane Index: " << LaneIndex << std::endl;
      // 如果在 E0 (準備經過 J1)，設定目標為 E2 (右轉) 或 E1 (左轉)
      if (roadID == "E0" && LaneIndex == "E0_0")
        {
          targetEdge = "E2"; // 這裡設為右轉 E2，若想左轉改 E1
        }
      else if (roadID == "E0" && LaneIndex == "E0_3")
        {
          targetEdge = "E1"; // 這裡設為左轉 E1，若想右轉改 E2
        }

      // 3. 如果找到了合適的叉路，執行改道
      if (targetEdge != "")
        {
          // 改變車輛目的地 (SUMO 會自動計算路徑並換車道)
          m_client->TraCIAPI::vehicle.changeTarget (m_id, targetEdge);
          std::cout << "Vehicle " << m_id << " switching to Fork road: " << targetEdge << std::endl;
          // // (選用) 將車輛變色為黃色，表示正在避讓/改道
          // libsumo::TraCIColor forkColor;
          // forkColor.r = 255; forkColor.g = 255; forkColor.b = 0; forkColor.a = 255;
          // m_client->TraCIAPI::vehicle.setColor (m_id, forkColor);
          shouldEnterForkRoad = 3;
        }
    }
  catch (const std::exception &e)
    {
      std::cerr << "Error in SwitchToForkRoad for " << m_id << ": " << e.what () << std::endl;
    }
}

void
emergencyVehicleAlert::SwitchToMainRoad ()
{
  // std::cout << "Switching back to main road for vehicle " << m_id << std::endl;
  try
    {
      std::string roadID = m_client->TraCIAPI::vehicle.getRoadID (m_id);
      std::string targetEdge = "";
      if (roadID == "E0")
        {
          targetEdge = "E5";
        }
      if (targetEdge != "")
        {
          m_client->TraCIAPI::vehicle.changeTarget (m_id, targetEdge);
          shouldEnterForkRoad = 2;
        }
    }
  catch (const std::exception &e)
    {
      std::cerr << "Error in SwitchToMainRoad for " << m_id << ": " << e.what () << std::endl;
    }
}

void
emergencyVehicleAlert::monitorVehicleRoutePeriodic ()
{
  try
    {
      if (shouldEnterForkRoad == 1)
        {
          SwitchToSideRoad ();
        }
      else if (shouldEnterForkRoad == 0)
        {
          SwitchToMainRoad ();
        }
      else if (shouldEnterForkRoad == 4)
        {
          SwitchToForkRoad ();
        }
      else if (shouldEnterForkRoad == 3)
        {
          std::string roadID;
          try
            {
              roadID = m_client->TraCIAPI::vehicle.getRoadID (m_id);
            }
          catch (const std::exception &e)
            {
              NS_LOG_DEBUG ("Vehicle "
                            << m_id
                            << " no longer exists in monitorVehicleRoutePeriodic: " << e.what ());
              return;
            }

          std::string targetEdge = "";
          if (roadID == "E2")
            {
              targetEdge = "E4";
            }
          else if (roadID == "E1")
            {
              targetEdge = "E6";
            }
          if (targetEdge != "")
            {
              try
                {
                  m_client->TraCIAPI::vehicle.changeTarget (m_id, targetEdge);
                  shouldEnterForkRoad = 5;
                }
              catch (const std::exception &e)
                {
                  NS_LOG_DEBUG (
                      "Failed to change target in monitorVehicleRoutePeriodic: " << e.what ());
                }
            }
        }
      Simulator::Schedule (Seconds (0.1), &emergencyVehicleAlert::monitorVehicleRoutePeriodic,
                           this);
    }
  catch (const std::exception &e)
    {
      NS_LOG_ERROR ("Unexpected error in monitorVehicleRoutePeriodic: " << e.what ());
    }
}
} // namespace ns3