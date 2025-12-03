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
const int cam_transmit_distance = 50;
const double attack_range = 50;

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
  // if (distance / distance_offset > 2)
  //   {
  //     return distance;
  //   }
  // else
  //   {
  //     return fmod (distance, distance_offset);
  //   }
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
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cpm), MakeBooleanChecker ());
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
      std::cerr << "無法開啟檔案 data.json" << std::endl;
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
      std::cerr << "JSON 解析錯誤: " << e.what () << std::endl;
      return;
    }
  attacker_id = data["attacker_id"];
  for (const auto &ev : data["emergency_vehicles"])
    {
      std::string id = ev["id"];
      veh_set.insert (id);
    }
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
  vehicleTypeInit ();
  m_id = m_client->GetVehicleId (this->GetNode ());
  m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
  m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);
  // std::cout << m_id << " " << m_type << std::endl;
  // 在現有的車輛信息獲取後添加
  string vehicleType = m_client->TraCIAPI::vehicle.getTypeID (m_id);

  // 設定激進的跟車參數
  m_client->TraCIAPI::vehicletype.setMinGap (vehicleType, 0.1); // 極小安全間距
  m_client->TraCIAPI::vehicletype.setTau (vehicleType, 0.1); // 短反應時間
  m_client->TraCIAPI::vehicletype.setDecel (vehicleType, 6.0); // 高減速能力
  m_client->TraCIAPI::vehicletype.setAccel (vehicleType, 3.0); // 適中加速度
  // 調試輸出檢查設定
  double currentMinGap = m_client->TraCIAPI::vehicle.getMinGap (m_id);

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
  // else
  //   {
  //     // TriggerEmergencyDenm();
  //     stationtype = StationType_passengerCar;
  //     libsumo::TraCIColor connected;
  //     connected.r = 0;
  //     connected.g = 225;
  //     connected.b = 255;
  //     connected.a = 255;
  //     m_client->TraCIAPI::vehicle.setColor (m_id, connected);
  //   }

  // else if (m_type=="emergency"){
  //   stationtype = StationType_specialVehicle;
  // m_LDM->enablePolygons (); // Uncomment to enable detected object polygon visualization for this specific vehicle
  // }
  // else
  //   stationtype = StationType_unknown;

  // libsumo::TraCIColor connected;
  // connected.r = 0;
  // connected.g = 225;
  // connected.b = 255;
  // connected.a = 255;
  // m_client->TraCIAPI::vehicle.setColor (m_id, connected);

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
      // Old desync code kept just for reference
      // It may lead to nodes not being desynchronized properly in specific situations in which
      // Simulator::Now().GetNanoSeconds () returns the same seed for multiple nodes
      // srand(Simulator::Now().GetNanoSeconds ());
      // double desync = ((double)rand()/RAND_MAX);

      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();

      m_caService.startCamDissemination (desync);
    }
  if (veh_set.count (m_id) > 0 && m_send_denm == true)
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 128; // Red
      alertColor.g = 0; // Green
      alertColor.b = 128; // Blue
      alertColor.a = 255; // Alpha (不透明)

      // 2. 透過 TraCI 客戶端，將這輛車 (m_id) 的顏色設定為 alertColor
      m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
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
  // /* Implement CAM strategy here */
  // libsumo::TraCIPosition pos;
  // m_cam_received++;
  // // if (is_monitoring)
  // //   {
  // //     return;
  // //   }
  // // 從 CAM 中提取發送車輛的位置
  std::string vehicle_id = "veh" + std::to_string (asn1cpp::getField (cam->header.stationId, long));
  // if (m_monitored_vehicle_id == vehicle_id || victim_m_id == vehicle_id)
  //   {
  //     return;
  //   }
  double sender_lat =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.latitude, double) /
      DOT_ONE_MICRO;
  double sender_lon =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.longitude,
                         double) /
      DOT_ONE_MICRO;
  if (m_id == attacker_id)
    {
      // 從 CAM 消息中提取數據
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

      int lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (vehicle_id);
      std::pair<std::string, double> leaderInfo =
          m_client->TraCIAPI::vehicle.getLeader (vehicle_id, denm_transmit_distance);

      // 取得結果
      std::string leaderID = leaderInfo.first; // 前車 ID
      double gap = leaderInfo.second;

      json CAM_json = {{"message_type", "CAM"},     {"vehicle_id", vehicle_id},
                       {"latitude", sender_lat},    {"longitude", sender_lon},
                       {"speed_mps", speed_mps},    {"acceleration_mps2", acceleration_mps2},
                       {"heading", heading},        {"lane_ID", lane_ID},
                       {"timestamp", timestamp_ms}, {"leader_ID", leaderID},
                       {"gap_to_leader", gap}};

      json_queue.push (CAM_json);
    }
  // else if (veh_set.count (m_id))
  //   {
  //     return;
  //   }
  // else
  //   {
  //     // 獲取接收車輛的當前位置
  //     pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  //     pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
  //     // 計算距離
  //     double distance = appUtil_haversineDist (pos.y, pos.x, sender_lat, sender_lon);
  //     // 檢查是否在同一車道
  //     string sender_id = "veh" + std::to_string (asn1cpp::getField (cam->header.stationId, long));
  //     std::pair<std::string, double> leaderInfo =
  //         m_client->TraCIAPI::vehicle.getLeader (m_id, cam_transmit_distance);
  //     std::string leaderID = leaderInfo.first;
  //     double gap = leaderInfo.second;
  //     int my_lane = m_client->TraCIAPI::vehicle.getLaneIndex (m_id);
  //     int sender_lane = m_client->TraCIAPI::vehicle.getLaneIndex (sender_id);

  //     // 如果距離和航向角都在閾值內,且在同一車道(lane 0),則減速
  //     if (distance < cam_transmit_distance && my_lane == sender_lane && leaderID == sender_id)
  //       {
  //         // 可選:改變車輛顏色以視覺化減速狀態
  //         libsumo::TraCIColor orange;
  //         orange.r = 255;
  //         orange.g = 255;
  //         orange.b = 255;
  //         orange.a = 255;
  //         m_client->TraCIAPI::vehicle.setColor (m_id, orange);
  //         // m_client->TraCIAPI::vehicle.slowDown (m_id, m_max_speed / 2, 5);
  //         Simulator::Remove (m_speed_ev);
  //         m_speed_ev = Simulator::Schedule (
  //             Seconds (0.5), &emergencyVehicleAlert::CheckDistanceAndRestore, this, sender_id);
  //       }
  //   }
}

// void
// emergencyVehicleAlert::setMaxSpeed ()
// {
//   // 恢復原始最大速度
//   m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);

//   // 恢復原始顏色
//   libsumo::TraCIColor white;
//   white.r = 0;
//   white.g = 255;
//   white.b = 255;
//   white.a = 255;
//   m_client->TraCIAPI::vehicle.setColor (m_id, white);
//   is_monitoring = false;
// }

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
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // 2. 獲取速度 (m/s)
  double speed_mps = m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);

  // 3. 獲取加速度 (m/s²)
  double acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (senderVehicleId);

  // 4. 獲取航向角度 (度)
  double angle = m_client->TraCIAPI::vehicle.getAngle (senderVehicleId);
  int heading = static_cast<int> (angle);
  int lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (senderVehicleId);

  // 創建與 CAM 格式相同的 JSON
  json DENM_sender_json = {
      {"message_type", "DENM"}, {"vehicle_id", senderVehicleId},
      {"latitude", pos.y},      {"longitude", pos.x},
      {"speed_mps", speed_mps}, {"acceleration_mps2", acceleration_mps2},
      {"heading", heading},     {"timestamp", timestamp_ms},
      {"lane_ID", lane_ID},
  };
  json_queue.push (DENM_sender_json);

  if (veh_set.count (m_id) || m_type == "Attacker")
    {
      return;
    }
  libsumo::TraCIPosition myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  myPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);

  libsumo::TraCIPosition attackerPos = m_client->TraCIAPI::vehicle.getPosition (attacker_id);
  attackerPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (attackerPos.x, attackerPos.y);
  double distance_to_attacker =
      appUtil_haversineDist (myPos.y, myPos.x, attackerPos.y, attackerPos.x);
  // 避免處理自己發送的DENM
  if (senderId == stol (m_id.substr (3)) ||
      (senderVehicleId == victim_m_id && m_type != "Attacker" &&
       distance_to_attacker <= attack_range))
    {
      std::cout << "Ignored DENM from " << senderVehicleId << " at time " << timestamp_ms
                << std::endl;
      return;
    }

  if (m_type != "Attacker")
    {
      libsumo::TraCIColor alertColor;
      alertColor.r = 0; // Red
      alertColor.g = 255; // Green
      alertColor.b = 0; // Blue
      alertColor.a = 255; // Alpha (不透明)
      m_client->TraCIAPI::vehicle.setColor (m_id, alertColor);
    }

  // 獲取發送者位置資訊(不準確)
  // double senderLat = mgmt.latitude / 10000000.0;
  // double senderLon = mgmt.longitude / 10000000.0;
  // 獲取自己的位置和車道資訊

  string myLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);
  libsumo::TraCIPosition senderPos = m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
  senderPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (senderPos.x, senderPos.y);
  std::pair<std::string, double> leaderInfo =
      m_client->TraCIAPI::vehicle.getLeader (m_id, cam_transmit_distance);
  std::string leaderID = leaderInfo.first;
  if (leaderID == attacker_id)
    {
      leaderInfo = m_client->TraCIAPI::vehicle.getLeader (attacker_id, cam_transmit_distance);
      leaderID = leaderInfo.first;
    }
  double gap = leaderInfo.second;
  // 計算距離
  double distance = appUtil_haversineDist (myPos.y, myPos.x, senderPos.y, senderPos.x);
  // cout << "Distance: " << distance << endl;
  if (distance <= denm_transmit_distance)
    { // 在DENM範圍內
      // 直接使用senderId構造車輛ID，無需遍歷

      try
        {
          // 直接獲取發送者車輛的車道資訊
          string senderLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (senderVehicleId);
          // cout << m_id << " " << myLaneIndex << " " << senderVehicleId << " " << senderLaneIndex <<endl;
          // 檢查是否在同一車道
          if (myLaneIndex == senderLaneIndex && senderVehicleId == leaderID)
            {
              // 獲取發送者車輛的實際位置
              // libsumo::TraCIPosition senderPos =
              //     m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
              // senderPos =
              //     m_client->TraCIAPI::simulation.convertXYtoLonLat (senderPos.x, senderPos.y);

              // 計算與發送者車輛的實際距離
              double distanceToSender =
                  appUtil_haversineDist (myPos.y, myPos.x, senderPos.y, senderPos.x);
              // 如果距離小於安全距離，則減速
              double safety_distance = 100;
              if (distanceToSender <= safety_distance)
                {
                  // 根據距離計算減速程度

                  double speedReduction = (safety_distance - distanceToSender) / safety_distance;
                  double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
                  double targetSpeed = currentSpeed * (1.0 - speedReduction * 0.5); // 最多減速50%

                  // 設定目標速度，但不低於最小速度
                  double minSpeed = 2.0; // 最小速度 2 m/s
                  double min_gap = 7.0;
                  double ratio = (distanceToSender - min_gap) / (safety_distance - min_gap);
                  targetSpeed = speed_mps * ratio;

                  m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);

                  // 設置視覺指示（紅色表示減速）
                  libsumo::TraCIColor slowdownColor;
                  slowdownColor.r = 255;
                  slowdownColor.g = 0;
                  slowdownColor.b = 0;
                  slowdownColor.a = 255;
                  m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor);
                  m_client->TraCIAPI::vehicle.slowDown (m_id, speed_mps-5, 0.5);
                  // Simulator::Remove (m_speed_ev);
                  // m_speed_ev = Simulator::Schedule (Seconds (0.5),
                  //                                   &emergencyVehicleAlert::CheckDistanceAndRestore,
                  //                                   this, senderVehicleId);
                }
              else
                {
                  RestoreSpeed (senderVehicleId);
                }
            }
        }
      catch (const exception &e)
        {
          // 如果發送者車輛不存在或無法訪問，記錄錯誤但繼續執行
          cerr << "Warning: Cannot access sender vehicle " << senderVehicleId << ": " << e.what ()
               << endl;
        }
    }
  // if (is_monitoring == false)
  // if (m_monitored_vehicle_id == "")
  //   {
  //     Simulator::Remove (m_range_check_ev);
  //     m_range_check_ev =
  //         Simulator::Schedule (Seconds (0.5), &emergencyVehicleAlert::RestoreOriginalColor, this);
  //   }
}

void
emergencyVehicleAlert::AttackerProcedureTrigger ()
{
  m_attacker_procedure_ev =
      Simulator::Schedule (Seconds (6.0), &emergencyVehicleAlert::AttackerSelectVictim, this);
}

void
emergencyVehicleAlert::QueryAllVehiclesAndLeaders ()
{
  // 獲取所有車輛 ID 列表
  std::vector<std::string> vehicleIDs = m_client->TraCIAPI::vehicle.getIDList ();

  json vehicles_info = json::array ();
  json message;

  for (const std::string &vehicleID : vehicleIDs)
    {
      try
        {
          // 獲取車輛位置
          libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (vehicleID);
          pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

          // 獲取車輛速度
          double speed_mps = m_client->TraCIAPI::vehicle.getSpeed (vehicleID);

          // 獲取車輛加速度
          double acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (vehicleID);

          // 獲取車輛航向角
          double angle = m_client->TraCIAPI::vehicle.getAngle (vehicleID);

          // 獲取車道索引
          int lane_ID = m_client->TraCIAPI::vehicle.getLaneIndex (vehicleID);

          std::pair<std::string, double> leaderInfo =
              m_client->TraCIAPI::vehicle.getLeader (vehicleID, 100.0); // 搜尋範圍 100 公尺

          // 建立 JSON 物件
          json vehicle_json = {{"vehicle_id", vehicleID},
                               {"latitude", pos.y},
                               {"longitude", pos.x},
                               {"speed_mps", speed_mps},
                               {"acceleration_mps2", acceleration_mps2},
                               {"heading", static_cast<int> (angle)},
                               {"lane_ID", lane_ID},
                               {"leader_ID", leaderInfo.first},
                               {"gap_to_leader", leaderInfo.second},
                               {"timestamp", Simulator::Now ().GetMilliSeconds ()}};

          vehicles_info.push_back (vehicle_json);
        }
      catch (const std::exception &e)
        {
          std::cerr << "Error querying vehicle " << vehicleID << ": " << e.what () << std::endl;
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
  catch (...)
    {
      m_query_all_vehicles_ev = Simulator::Schedule (
          Seconds (0.1), &emergencyVehicleAlert::QueryAllVehiclesAndLeaders, this);
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
  // if (victim_m_id != "")
  //   {
  try
    {
      string target_id;
      if (victim_m_id != "")
        {
          target_id = victim_m_id;
        }
      else
        {
          target_id = veh_set.begin ()->c_str ();
        }
      // 1. 獲取位置與距離
          libsumo::TraCIPosition myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
          libsumo::TraCIPosition myGeo = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);

          libsumo::TraCIPosition vicPos = m_client->TraCIAPI::vehicle.getPosition (target_id);
          libsumo::TraCIPosition vicGeo = m_client->TraCIAPI::simulation.convertXYtoLonLat (vicPos.x, vicPos.y);

          double dist = appUtil_haversineDist (myGeo.y, myGeo.x, vicGeo.y, vicGeo.x);
          
          // 2. 獲取受害者速度
          double vic_speed = m_client->TraCIAPI::vehicle.getSpeed (target_id);
          
          // 3. 定義跟車參數
          double target_gap = 15.0;      // 目標保持落後 15 公尺
          double tolerance = 3.0;        // 容許誤差 (12m ~ 18m 視為剛好)
          double catch_up_speed = 5.0;   // 追趕時的額外速度
          double fall_back_factor = 0.7; // 太近時的減速比例 (降至受害者速度的 70%)

          double target_speed;

          // 4. 三段式控制邏輯
          if (dist > target_gap + tolerance)
            {
              // [太遠] (> 18m): 加速追趕
              target_speed = std::min(m_max_speed, vic_speed + catch_up_speed);
              // 如果受害者靜止，至少保持一點速度接近
              if(target_speed < 2.0) target_speed = 5.0; 
            }
          else if (dist < target_gap - tolerance)
            {
              // [太近] (< 12m): 減速讓對方拉開距離
              target_speed = vic_speed * fall_back_factor;
              // 確保不會變成負數
              if(target_speed < 0) target_speed = 0;
            }
          else
            {
              // [剛好] (12m ~ 18m): 速度同步，鎖定距離
              target_speed = vic_speed;
            }

          // 5. 執行速度設定
          m_client->TraCIAPI::vehicle.setSpeed (m_id, target_speed);
    }
  catch (...)
    {
      std::cout << "Victim " << victim_m_id << " not found or error occurred." << std::endl;
      // 如果受害者消失或出錯，保持原速或減速
    }
  // }
  // else if (!veh_set.empty ())
  //   {
  //     // 還沒有選定 Victim 時的預設行為 (跟隨第一輛車)
  //     double default_speed = m_client->TraCIAPI::vehicle.getSpeed (veh_set.begin ()->c_str ());
  //     m_client->TraCIAPI::vehicle.setSpeed (m_id, default_speed);
  //   }
  while (!json_queue.empty ())
    {
      if ((timestamp_ms - 200 <= json_queue.front ()["timestamp"] &&
           json_queue.front ()["timestamp"] <= timestamp_ms))
        {

          vehicle_data_map[json_queue.front ()["vehicle_id"].get<string> () +
                           json_queue.front ()["message_type"].get<string> ()] =
              json_queue.front ();
          // DENM_cnt++;
          // json_array.push_back (json_queue.front ());
        }
      json_queue.pop ();
    }
  for (auto &pair : vehicle_data_map)
    {
      json_array.push_back (pair.second);
    }
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // 2. 獲取速度 (m/s)
  double speed_mps = m_client->TraCIAPI::vehicle.getSpeed (m_id);

  // 3. 獲取加速度 (m/s²)
  double acceleration_mps2 = m_client->TraCIAPI::vehicle.getAcceleration (m_id);

  // 4. 獲取航向角度 (度)
  double angle = m_client->TraCIAPI::vehicle.getAngle (m_id);
  int heading = static_cast<int> (angle);
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
  // cout << json_string << endl;
  try
    {
      res = socketClientFunction (json_string.c_str ());
      json res_json = json::parse (res);
      victim_m_id = res_json["vehicle_id"].get<string> ();
      attack_duration = res_json["attack_duration_seconds"].get<float> ();
      Simulator::Remove (m_attacker_procedure_ev);
      m_attacker_procedure_ev = Simulator::Schedule (
          Seconds (attack_duration), &emergencyVehicleAlert::AttackerSelectVictim, this);
      // Simulator::Stop(Seconds(attack_duration));
      return;
    }
  catch (...)
    {
      std::cout << "error!" << std::endl;
      Simulator::Remove (m_attacker_procedure_ev);
      m_attacker_procedure_ev =
          Simulator::Schedule (Seconds (1.0), &emergencyVehicleAlert::AttackerSelectVictim, this);
      return;
    }
}

void
emergencyVehicleAlert::CheckDistanceAndRestore (string senderVehicleId)
{
  // is_monitoring = true;
  m_monitored_vehicle_id = senderVehicleId;
  try
    {
      // 獲取當前位置
      libsumo::TraCIPosition myPos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      myPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (myPos.x, myPos.y);
      double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);

      // 獲取發送者位置
      libsumo::TraCIPosition senderPos = m_client->TraCIAPI::vehicle.getPosition (senderVehicleId);
      senderPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (senderPos.x, senderPos.y);
      double senderSpeed = m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);

      string senderLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (senderVehicleId);
      string myLaneIndex = m_client->TraCIAPI::vehicle.getLaneID (m_id);
      if (senderLaneIndex != myLaneIndex || currentSpeed <= denm_veh_min_speed)
        {
          // 不在同一車道，恢復速度
          RestoreSpeed (senderVehicleId);
          return;
        }

      double distanceToSender = appUtil_haversineDist (myPos.y, myPos.x, senderPos.y, senderPos.x);
      double safety_distance = 50;

      if (distanceToSender >= safety_distance)
        {
          // 距離安全，恢復原速度
          RestoreSpeed (senderVehicleId);
          // cout << "Vehicle " << m_id << " restored speed - safe distance: " << distanceToSender
          // << "m" << endl;
        }
      else
        {
          // **關鍵修改：重新計算減速邏輯**
          // double speedReduction = (safety_distance - distanceToSender) / safety_distance;
          // double currentSpeed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
          // double targetSpeed = currentSpeed * (1.0 - speedReduction * 0.5);

          // double minSpeed = 2.0;
          // targetSpeed = max (targetSpeed, minSpeed);
          libsumo::TraCIColor slowdownColor;
          slowdownColor.r = 255;
          slowdownColor.g = 0;
          slowdownColor.b = 0;
          slowdownColor.a = 255;
          // m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor);
          // // 重新設定速度而不是只監控
          // // m_client->TraCIAPI::vehicle.setSpeed (m_id, targetSpeed);
          // m_client->TraCIAPI::vehicle.slowDown (m_id, currentSpeed * 0.95, 0.5);

          // cout << "Vehicle " << m_id << " adjusting speed due to proximity to DENM sender "
          //           << senderVehicleId << " (distance: " << distanceToSender
          //           << "m, target speed: " << targetSpeed << " m/s)" << endl;
          double targetSpeed;
          // 定義安全距離 (建議稍微加大，考慮車長)
          double safety_distance = cam_transmit_distance;
          // 定義最小安全距離 (例如 5米，低於此距離需緊急煞停)
          double min_gap = 7.0;
          if (distanceToSender < min_gap)
            {
              // 距離極近：緊急煞車 (設為 0 或極低速)
              targetSpeed = 0.0;
              m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor); // 紅色
            }
          else
            {
              // 距離不足：目標速度應低於前車速度以拉開距離
              // 邏輯：(當前距離 / 安全距離) * 前車速度
              // 這樣當距離越近，速度會降得比前車更低
              double ratio = (distanceToSender - min_gap) / (safety_distance - min_gap);
              targetSpeed = senderSpeed * ratio;

              // 確保不會變成負數
              if (targetSpeed < 0)
                targetSpeed = 0;

              m_client->TraCIAPI::vehicle.setColor (m_id, slowdownColor); // 橘色
            }

          // 執行減速：最後參數 0.5 是 duration，設為 0 代表立即改變目標速度讓 SUMO 控制煞車
          // 或者設一個很短的時間如 0.1
          m_client->TraCIAPI::vehicle.slowDown (m_id, targetSpeed, 0.5);
          Simulator::Remove (m_speed_ev);
          m_speed_ev =
              Simulator::Schedule (Seconds (0.5), &emergencyVehicleAlert::CheckDistanceAndRestore,
                                   this, senderVehicleId);
        }
    }
  catch (const exception &e)
    {
      // 發送者車輛可能已經離開，恢復速度
      // cout << "error" << endl;
      RestoreSpeed (senderVehicleId);
    }
}

void
emergencyVehicleAlert::RestoreOriginalColor ()
{
  if (m_type == "Attacker")
    {
      return;
    }
  libsumo::TraCIColor normalColor;
  normalColor.r = 0;
  normalColor.g = 225;
  normalColor.b = 255;
  normalColor.a = 255; // 藍色
  m_client->TraCIAPI::vehicle.setColor (m_id, normalColor);
  // cout << "Vehicle " << m_id << " restored to original color (out of DENM range)" << endl;
}

void
emergencyVehicleAlert::TriggerEmergencyDenm ()
{
  denData data;
  DEN_ActionID_t actionid;
  DENBasicService_error_t trigger_retval;
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);
  /* Set DENM mandatpry fields */
  data.setDenmMandatoryFields (compute_timestampIts (m_real_time), pos.x, pos.y);
  denData::denDataSituation situation_data;

  // 範例：設定事件為「緊急車輛接近」
  // 注意：CauseCode_emergencyVehicleApproaching 是一個假設的常數名稱，
  // 你需要根據你使用的函式庫 (asn1c) 找到實際的常數名稱。
  // 常見的可能是 CauseCode_t_emergencyVehicleApproaching 或類似的名稱。
  situation_data.causeCode = 97; // 根據 ETSI TS 101 628-6-1，97 代表 emergencyVehicleApproaching
  situation_data.subCauseCode = 0; // 此原因沒有次代碼

  /* 3. 將 situation 容器設定到主 data 物件中 */
  data.setDenmSituationData_asn_types (situation_data);

  /* Compute GeoArea for denms */
  GeoArea_t geoArea;
  // 直接使用上面已經獲取的位置
  geoArea.posLong = pos.x * DOT_ONE_MICRO;
  geoArea.posLat = pos.y * DOT_ONE_MICRO;
  geoArea.distA = 1000;
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  trigger_retval = m_denService.appDENM_trigger (data, actionid);
  if (trigger_retval == DENM_NO_ERROR)
    {
      m_updateDenmEvent = Simulator::Schedule (
          Seconds (1.0), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionid);
    }
}

void
emergencyVehicleAlert::UpdateEmergencyDenm (DEN_ActionID_t actionID)
{
  libsumo::TraCIPosition attackerPos = m_client->TraCIAPI::vehicle.getPosition (attacker_id);
  attackerPos = m_client->TraCIAPI::simulation.convertXYtoLonLat (attackerPos.x, attackerPos.y);
  // std::cout << attackerPos.x << " " << attackerPos.y << std::endl;
  denData data;
  denData::denDataSituation situation_data;
  DENBasicService_error_t update_retval;
  int denm_veh_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  if (denm_veh_speed + denm_veh_accel > denm_veh_min_speed)
    {
      m_client->TraCIAPI::vehicle.slowDown (m_id, denm_veh_speed + denm_veh_accel, 0.1);
    }
  else
    {
      // int decel_duration = floor ((denm_veh_speed - denm_veh_min_speed) / denm_veh_accel);
      // std::cout << "Vehicle " << m_id << " is performing emergency braking to "
      //           << denm_veh_min_speed << " m/s over " << decel_duration << " seconds."
      //           << std::endl;
      // m_client->TraCIAPI::vehicle.setAcceleration (m_id, denm_veh_accel,
      //                                              decel_duration); // 緊急煞車
      denm_veh_speed = denm_veh_max_speed;
      m_client->TraCIAPI::vehicle.slowDown (m_id, denm_veh_speed, 0.5);
    }

  // 設置強制欄位
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // 2. 使用有效位置設定強制性欄位
  data.setDenmMandatoryFields (compute_timestampIts (m_real_time), pos.y, pos.x);
  // 更新事件資訊
  situation_data.causeCode = 97; // 根據 ETSI TS 101 628-6-1，97 代表 emergencyVehicleApproaching
  situation_data.subCauseCode = 0; // 此原因沒有次代碼
  data.setDenmSituationData_asn_types (situation_data);
  // 更新地理區域
  GeoArea_t geoArea;
  geoArea.posLong = pos.x * DOT_ONE_MICRO;
  geoArea.posLat = pos.y * DOT_ONE_MICRO;
  geoArea.distA = denm_transmit_distance; // 1000米半徑
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  // 更新DENM
  update_retval = m_denService.appDENM_update (data, actionID);
  if (update_retval != DENM_NO_ERROR)
    {
      NS_LOG_ERROR ("Failed to update DENM. Error code: " << update_retval);
      return; // 停止進一步的更新
    }

  // 排程下一次更新
  m_updateDenmEvent = Simulator::Schedule (
      Seconds (0.1), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionID);
}

void
emergencyVehicleAlert::RestoreSpeed (string senderVehicleId)
{
  libsumo::TraCIColor normal;
  normal.r = 0;
  normal.g = 225;
  normal.b = 255;
  normal.a = 255; //blue
  double senderSpeed = m_client->TraCIAPI::vehicle.getSpeed (senderVehicleId);
  m_client->TraCIAPI::vehicle.setColor (m_id, normal);
  // m_client->TraCIAPI::vehicle.setSpeed (m_id, senderSpeed + 200);
  // m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);
  m_client->TraCIAPI::vehicle.slowDown (m_id, m_max_speed, 5);
  // is_monitoring = false;
  m_monitored_vehicle_id = "";
}

// void
// emergencyVehicleAlert::SetMaxSpeed ()
// {
//   libsumo::TraCIColor normal;
//   normal.r = 0;
//   normal.g = 225;
//   normal.b = 255;
//   normal.a = 255; //blue
//   m_client->TraCIAPI::vehicle.setColor (m_id, normal);
//   m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);
// }

void
emergencyVehicleAlert::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
{
  /* Implement CPM strategy here */
  m_cpm_received++;
  (void) from;
  //For every PO inside the CPM, if any
  bool POs_ok;
  //auto wrappedContainer = asn1cpp::makeSeq(WrappedCpmContainer);
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
          //cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv2 from " << asn1cpp::getField(cpm->header.stationId,long) << " with " << PObjects_size << " perceived objects." << endl;
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
}

} // namespace ns3
