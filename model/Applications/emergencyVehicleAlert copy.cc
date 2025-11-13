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
#include <cmath>

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"


#define DEG_2_RAD(val) ((val)*M_PI/180.0)
const double distance_offset = 5.06614e+06;
const int denm_transmit_distance = 50;

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("emergencyVehicleAlert");

  NS_OBJECT_ENSURE_REGISTERED(emergencyVehicleAlert);

  // Function to compute the distance between two objects, given their Lon/Lat
  double appUtil_haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
      // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
      // return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
      double distance = 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
      return fmod(distance, distance_offset);
    }


  // Function to compute the absolute difference between two angles (angles must be between -180 and 180)
  double appUtil_angDiff(double ang1, double ang2) {
      double angDiff;
      angDiff=ang1-ang2;

      if(angDiff>180)
      {
        angDiff-=360;
      }
      else if(angDiff<-180)
      {
        angDiff+=360;
      }
      return std::abs(angDiff);
  }

  TypeId
  emergencyVehicleAlert::GetTypeId (void)
  {
    static TypeId tid =
        TypeId ("ns3::emergencyVehicleAlert")
        .SetParent<Application> ()
        .SetGroupName ("Applications")
        .AddConstructor<emergencyVehicleAlert> ()
        .AddAttribute ("RealTime",
            "To compute properly timestamps",
            BooleanValue(false),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_real_time),
            MakeBooleanChecker ())
        .AddAttribute ("IpAddr",
            "IpAddr",
            Ipv4AddressValue ("10.0.0.1"),
            MakeIpv4AddressAccessor (&emergencyVehicleAlert::m_ipAddress),
            MakeIpv4AddressChecker ())
        .AddAttribute ("PrintSummary",
            "To print summary at the end of simulation",
            BooleanValue(false),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_print_summary),
            MakeBooleanChecker ())
        .AddAttribute ("CSV",
            "CSV log name",
            StringValue (),
            MakeStringAccessor (&emergencyVehicleAlert::m_csv_name),
            MakeStringChecker ())
        .AddAttribute ("Model",
            "Physical and MAC layer communication model",
            StringValue (""),
            MakeStringAccessor (&emergencyVehicleAlert::m_model),
            MakeStringChecker ())
        .AddAttribute ("Client",
            "TraCI client for SUMO",
            PointerValue (0),
            MakePointerAccessor (&emergencyVehicleAlert::m_client),
            MakePointerChecker<TraciClient> ())
        .AddAttribute ("MetricSupervisor",
            "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
            PointerValue (0),
            MakePointerAccessor (&emergencyVehicleAlert::m_metric_supervisor),
            MakePointerChecker<MetricSupervisor> ())
        .AddAttribute ("SendCAM",
            "To enable/disable the transmission of CAM messages",
            BooleanValue(true),
            MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cam),
            MakeBooleanChecker ())
        .AddAttribute ("SendCPM",
           "To enable/disable the transmission of CPM messages",
           BooleanValue(true),
           MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cpm),
           MakeBooleanChecker ());
        return tid;
  }

  emergencyVehicleAlert::emergencyVehicleAlert ()
  {
    NS_LOG_FUNCTION(this);
    m_client = nullptr;
    m_print_summary = true;
    m_already_print = false;
    m_send_cam = false;
    m_send_denm = false;


    m_denm_sent = 0;
    m_cam_received = 0;
    m_cpm_received = 0;
    m_denm_received = 0;
    m_denm_intertime = 0;

    m_distance_threshold = 0; // Distance used in GeoNet to determine the radius of the circumference arounf the emergency vehicle where the DENMs are valid
    m_heading_threshold = 45; // Max heading angle difference between the normal vehicles and the emergenecy vehicle, that triggers a reaction in the normal vehicles
  }

  emergencyVehicleAlert::~emergencyVehicleAlert ()
  {
    NS_LOG_FUNCTION(this);
  }

  void
  emergencyVehicleAlert::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    Application::DoDispose ();
  }

  void
  emergencyVehicleAlert::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);

    /*
     * In this example, the vehicle can be either of type "passenger" or of type "emergency" (see cars.rou.xml in SUMO folder inside examples/sumo_files_v2v_map)
     * All the vehicles broadcast CAM messages. When a "passenger" car receives a CAM from an "emergency" vehicle, it checks the distance between them and
     * the difference in heading, and if it considers it to be close, it takes proper actions to facilitate the takeover maneuver.
     */

    /* Save the vehicles informations */
    m_id = m_client->GetVehicleId (this->GetNode ());
    m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
    m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);
    VDP* traci_vdp = new VDPTraCI(m_client,m_id);
    // if (m_type=="emergency" && m_id != "veh1"){
    //   return;
    // }
    //Create LDM and sensor object
    m_LDM = CreateObject<LDM>();
    m_LDM->setStationID(m_id);
    m_LDM->setTraCIclient(m_client);
    m_LDM->setVDP(traci_vdp);

    m_sensor = CreateObject<SUMOSensor>();
    m_sensor->setStationID(m_id);
    m_sensor->setTraCIclient(m_client);
    m_sensor->setVDP(traci_vdp);
    m_sensor->setLDM (m_LDM);

    // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
    m_btp = CreateObject <btp>();
    m_geoNet = CreateObject <GeoNet>();

    if(m_metric_supervisor!=nullptr)
    {
      m_geoNet->setMetricSupervisor(m_metric_supervisor);
    }

    m_btp->setGeoNet(m_geoNet);
    m_denService.setBTP(m_btp);
    m_caService.setBTP(m_btp);
    m_cpService.setBTP(m_btp);
    m_caService.setLDM(m_LDM);
    m_cpService.setLDM(m_LDM);

    /* Create the Sockets for TX and RX */
    TypeId tid;
    if(m_model=="80211p")
      tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
    else if(m_model=="cv2x" || m_model=="nrv2x")
      tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    else
      NS_FATAL_ERROR ("No communication model set - check simulation script - valid models: '80211p' or 'lte'");
    m_socket = Socket::CreateSocket (GetNode (), tid);

    if(m_model=="80211p")
    {
        /* Bind the socket to local address */
        PacketSocketAddress local = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress () );
        if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
        // Set the socketAddress for broadcast
        PacketSocketAddress remote = getGNAddress(GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetBroadcast () );
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
        m_socket->Connect (InetSocketAddress(m_ipAddress,19));
    }

    /* Set Station Type in DENBasicService */
    StationType_t stationtype;
    if (m_type=="passenger"){
      // TriggerEmergencyDenm();
      stationtype = StationType_passengerCar;
    }

    // else if (m_type=="emergency"){
    //   stationtype = StationType_specialVehicle;
    // m_LDM->enablePolygons (); // Uncomment to enable detected object polygon visualization for this specific vehicle
    // }
    else
      stationtype = StationType_unknown;
    libsumo::TraCIColor connected;
    connected.r=0;connected.g=225;connected.b=255;connected.a=255;
    m_client->TraCIAPI::vehicle.setColor (m_id, connected);

    /* Set sockets, callback and station properties in DENBasicService */
    m_denService.setSocketTx (m_socket);
    m_denService.setSocketRx (m_socket);
    m_denService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_denService.addDENRxCallback (std::bind(&emergencyVehicleAlert::receiveDENM,this,std::placeholders::_1,std::placeholders::_2));
    m_denService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
    m_caService.setSocketTx (m_socket);
    m_caService.setSocketRx (m_socket);
    m_caService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_caService.addCARxCallback (std::bind(&emergencyVehicleAlert::receiveCAM,this,std::placeholders::_1,std::placeholders::_2));
    m_caService.setRealTime (m_real_time);

    /* Set sockets, callback, station properties and TraCI VDP in CPBasicService */
    m_cpService.setSocketTx (m_socket);
    m_cpService.setSocketRx (m_socket);
    m_cpService.setStationProperties (std::stol(m_id.substr (3)), (long)stationtype);
    m_cpService.addCPRxCallback (std::bind(&emergencyVehicleAlert::receiveCPM,this,std::placeholders::_1,std::placeholders::_2));
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
    m_cpService_v1.setStationProperties(std::stol(m_id.substr (3)), (long)stationtype);
    m_cpService_v1.addCPRxCallback(std::bind(&emergencyVehicleAlert::receiveCPMV1,this,std::placeholders::_1,std::placeholders::_2));
    m_cpService_v1.startCpmDissemination ();
    */

    /* Set TraCI VDP for GeoNet object */
    m_caService.setVDP(traci_vdp);
    m_denService.setVDP(traci_vdp);
    m_cpService.setVDP(traci_vdp);

    /* Schedule CAM dissemination */
    if(m_send_cam == true)
    {
      // Old desync code kept just for reference
      // It may lead to nodes not being desynchronized properly in specific situations in which
      // Simulator::Now().GetNanoSeconds () returns the same seed for multiple nodes
      // std::srand(Simulator::Now().GetNanoSeconds ());
      // double desync = ((double)std::rand()/RAND_MAX);

      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();

      m_caService.startCamDissemination(desync);
    }

    // if(m_id == "veh2" || m_id == "veh5" || m_id == "veh1" || m_id == "veh6" || m_id == "veh7" || m_id == "veh10")
    if((m_id == "veh1" || m_id == "veh7") && m_send_denm == true)
    {
      
      libsumo::TraCIColor alertColor;
      alertColor.r = 128; // Red
      alertColor.g = 0;   // Green
      alertColor.b = 128;   // Blue
      alertColor.a = 255; // Alpha (不透明)

    // 2. 透過 TraCI 客戶端，將這輛車 (m_id) 的顏色設定為 alertColor
      m_client->TraCIAPI::vehicle.setColor(m_id, alertColor);
      TriggerEmergencyDenm();  
    }

    /* Schedule CPM dissemination */
    if(m_send_cpm == true)
    {
      m_cpService.startCpmDissemination ();
    }

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name+"-"+m_id+"-CAM.csv",std::ofstream::trunc);
      m_csv_ofstream_cam << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }
  }

  void
  emergencyVehicleAlert::StopApplication ()
  {
    NS_LOG_FUNCTION(this);
    Simulator::Cancel(m_speed_ev);
    Simulator::Cancel(m_send_cam_ev);
    Simulator::Cancel(m_update_denm_ev);

    uint64_t cam_sent, cpm_sent;

    if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.close ();
    }

    cam_sent = m_caService.terminateDissemination ();
    cpm_sent = m_cpService.terminateDissemination ();
    m_denService.cleanup();
    m_LDM->cleanup();
    m_sensor->cleanup();

    if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id
                << ",CAM-SENT:" << cam_sent
                << ",CAM-RECEIVED:" << m_cam_received
                << ",CPM-SENT: " << cpm_sent
                << ",CPM-RECEIVED: " << m_cpm_received
                << std::endl;
      m_already_print=true;
    }
    Simulator::Cancel(m_updateDenmEvent);
  }

  void
  emergencyVehicleAlert::StopApplicationNow ()
  {
    NS_LOG_FUNCTION(this);
    StopApplication ();
  }

  void
  emergencyVehicleAlert::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
  {
    /* Implement CAM strategy here */
   m_cam_received++;

   /* If the CAM is received from an emergency vehicle, and the host vehicle is a "passenger" car, then process the CAM */
  //  if (asn1cpp::getField(cam->cam.camParameters.basicContainer.stationType,StationType_t)==StationType_specialVehicle && m_type!="emergency")
  //  {
  //    libsumo::TraCIPosition pos=m_client->TraCIAPI::vehicle.getPosition(m_id);
  //    pos=m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);

  //    /* If the distance between the "passenger" car and the emergency vehicle and the difference in the heading angles
  //     * are below certain thresholds, then actuate the slow-down strategy */
  //    if (appUtil_haversineDist (pos.y,pos.x,
  //                               asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO,
  //                               asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO)
  //        < m_distance_threshold
  //        &&
  //        appUtil_angDiff (m_client->TraCIAPI::vehicle.getAngle (m_id),(double)asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,HeadingValue_t)/DECI)<m_heading_threshold)
  //    {
  //      /* Slowdown only if you are not in the takeover lane,
  //       * otherwise the emergency vechicle may get stuck behind */
  //      if (m_client->TraCIAPI::vehicle.getLaneIndex (m_id) == 0)
  //      {
  //        m_client->TraCIAPI::vehicle.changeLane (m_id,0,3);
  //        m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed*0.5);
  //        libsumo::TraCIColor orange;
  //        orange.r=232;orange.g=126;orange.b=4;orange.a=255;
  //        m_client->TraCIAPI::vehicle.setColor (m_id,orange);

  //        Simulator::Remove(m_speed_ev);
  //        m_speed_ev = Simulator::Schedule (Seconds (3.0), &emergencyVehicleAlert::SetMaxSpeed, this);
  //      }
  //      else
  //      {
  //        m_client->TraCIAPI::vehicle.changeLane (m_id,0,3);
  //        m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed*1.5);
  //        libsumo::TraCIColor green;
  //        green.r=0;green.g=128;green.b=80;green.a=255;
  //        m_client->TraCIAPI::vehicle.setColor (m_id,green);

  //        Simulator::Remove(m_speed_ev);
  //        m_speed_ev = Simulator::Schedule (Seconds (3.0), &emergencyVehicleAlert::SetMaxSpeed, this);
  //      }
  //    }
  //  }

   if (!m_csv_name.empty ())
     {
       // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
       m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
       m_csv_ofstream_cam << cam->cam.generationDeltaTime << "," << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/DOT_ONE_MICRO << ",";
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/DOT_ONE_MICRO << "," ;
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue,double)/DECI << "," << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue,double)/CENTI << ",";
       m_csv_ofstream_cam << asn1cpp::getField(cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.value,double)/DECI << std::endl;
     }

  }



  void
  emergencyVehicleAlert::receiveCPMV1 (asn1cpp::Seq<CPMV1> cpm, Address from)
  {
    /* Implement CPM strategy here */
    m_cpm_received++;
    (void) from;
    //std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv1 from vehicle " << asn1cpp::getField(cpm->header.stationId,long) <<" with "<< asn1cpp::getField(cpm->cpm.cpmParameters.numberOfPerceivedObjects,long)<< " perceived objects." <<std::endl;
    //For every PO inside the CPM, if any
    bool POs_ok;
    auto PObjects = asn1cpp::getSeqOpt(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObjectContainer,&POs_ok);
    if (POs_ok)
      {
        int PObjects_size = asn1cpp::sequenceof::getSize(cpm->cpm.cpmParameters.perceivedObjectContainer);
        for(int i=0; i<PObjects_size;i++)
          {
            LDM::returnedVehicleData_t PO_data;
            auto PO_seq = asn1cpp::makeSeq(PerceivedObjectV1);
            PO_seq = asn1cpp::sequenceof::getSeq(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObjectV1,i);
            //If PO is already in local copy of vLDM
            if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectID,long),PO_data) == LDM::LDM_OK)
              {
                  //Add the new perception to the LDM
                  std::vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
                  if(std::find(associatedCVs.begin(), associatedCVs.end (), asn1cpp::getField(cpm->header.stationId,long)) == associatedCVs.end ())
                    associatedCVs.push_back (asn1cpp::getField(cpm->header.stationId,long));
                  PO_data.vehData.associatedCVs = OptionalDataItem<std::vector<long>>(associatedCVs);
                  m_LDM->insert (PO_data.vehData);
              }
            else
              {
               //Translate CPM data to LDM format
               m_LDM->insert(translateCPMV1data(cpm,i));
              }
          }
      }
  }

  vehicleData_t
  emergencyVehicleAlert::translateCPMV1data (asn1cpp::Seq<CPMV1> cpm, int objectIndex)
  {
    vehicleData_t retval;
    auto PO_seq = asn1cpp::makeSeq(PerceivedObjectV1);
    using namespace boost::geometry::strategy::transform;
    PO_seq = asn1cpp::sequenceof::getSeq(cpm->cpm.cpmParameters.perceivedObjectContainer,PerceivedObjectV1,objectIndex);
    retval.detected = true;
    retval.stationID = asn1cpp::getField(PO_seq->objectID,long);
    retval.ID = std::to_string(retval.stationID);
    retval.vehicleLength = asn1cpp::getField(PO_seq->planarObjectDimension1->value,long);
    retval.vehicleWidth = asn1cpp::getField(PO_seq->planarObjectDimension2->value,long);
    retval.heading = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue,double)/10 +
                        asn1cpp::getField(PO_seq->yawAngle->value,double)/10;
    if (retval.heading > 360.0)
      retval.heading -= 360.0;

    retval.speed_ms = (double) (asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue,long) +
                        asn1cpp::getField(PO_seq->xSpeed.value,long))/CENTI;

    double fromLon = asn1cpp::getField(cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude,double)/DOT_ONE_MICRO;
    double fromLat = asn1cpp::getField(cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude,double)/DOT_ONE_MICRO;


    libsumo::TraCIPosition objectPosition = m_client->TraCIAPI::simulation.convertLonLattoXY (fromLon,fromLat);

    point_type objPoint(asn1cpp::getField(PO_seq->xDistance.value,double)/CENTI,asn1cpp::getField(PO_seq->yDistance.value,double)/CENTI);
    double fromAngle = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue,double)/10;
    rotate_transformer<boost::geometry::degree, double, 2, 2> rotate(fromAngle-90);
    boost::geometry::transform(objPoint, objPoint, rotate);// Transform points to the reference (x,y) axises
    objectPosition.x += boost::geometry::get<0>(objPoint);
    objectPosition.y += boost::geometry::get<1>(objPoint);

    libsumo::TraCIPosition objectPosition2 = objectPosition;
    objectPosition = m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);

    retval.lon = objectPosition.x;
    retval.lat = objectPosition.y;

    point_type speedPoint(asn1cpp::getField(PO_seq->xSpeed.value,double)/CENTI,asn1cpp::getField(PO_seq->ySpeed.value,double)/CENTI);
    boost::geometry::transform(speedPoint, speedPoint, rotate);// Transform points to the reference (x,y) axises
    retval.speed_ms = asn1cpp::getField(cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue,double)/CENTI + boost::geometry::get<0>(speedPoint);

    retval.camTimestamp = asn1cpp::getField(cpm->cpm.generationDeltaTime,long);
    retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(PO_seq->timeOfMeasurement,long)*1000);
    retval.stationType = StationType_passengerCar;
    retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));
    retval.confidence = asn1cpp::getField(PO_seq->objectConfidence,long);
    return retval;

  }

  
  void emergencyVehicleAlert::receiveDENM (denData denm, Address from)  
  {  
    // 獲取發送者的管理資料  
    auto mgmt = denm.getDenmMgmtData_asn_types();  
    uint32_t senderId = mgmt.stationID;  
      
    // 避免處理自己發送的DENM  
    if (senderId == std::stol(m_id.substr(3))) {  
        return;  
    }  
    libsumo::TraCIColor alertColor;
    alertColor.r = 255; // Red
    alertColor.g = 100;   // Green
    alertColor.b = 0;   // Blue
    alertColor.a = 255; // Alpha (不透明)
    m_client->TraCIAPI::vehicle.setColor(m_id, alertColor);
      
    // 獲取發送者位置資訊  
    double senderLat = mgmt.latitude / 10000000.0;  
    double senderLon = mgmt.longitude / 10000000.0;  
      
    // 獲取自己的位置和車道資訊  
    libsumo::TraCIPosition myPos = m_client->TraCIAPI::vehicle.getPosition(m_id);  
    myPos = m_client->TraCIAPI::simulation.convertXYtoLonLat(myPos.x, myPos.y);  
    int myLaneIndex = m_client->TraCIAPI::vehicle.getLaneIndex(m_id);  
      
    // 計算距離  
    double distance = appUtil_haversineDist(myPos.y, myPos.x, senderLat, senderLon);  
    // std::cout << m_id << " " << myPos.x << " " << myPos.y << " " << senderLon << " " << senderLat << " " << distance << std::endl;
    if (distance <= denm_transmit_distance) { // 在DENM範圍內  
        // 嘗試找到發送者車輛並檢查車道  
        std::vector<std::string> allVehicles = m_client->TraCIAPI::vehicle.getIDList();  
          
        for (const auto& vehId : allVehicles) {  
            // 檢查是否為發送者車輛（通過位置匹配）  
            libsumo::TraCIPosition vehPos = m_client->TraCIAPI::vehicle.getPosition(vehId);  
            vehPos = m_client->TraCIAPI::simulation.convertXYtoLonLat(vehPos.x, vehPos.y);  
              
            double vehDistance = appUtil_haversineDist(vehPos.y, vehPos.x, senderLat, senderLon);  
              
            if (vehDistance < 10.0) { // 找到發送者車輛（10米誤差範圍內）  
                int senderLaneIndex = m_client->TraCIAPI::vehicle.getLaneIndex(vehId);  
                  
                // 檢查是否在同一車道  
                if (myLaneIndex == senderLaneIndex) {  
                    std::cout << senderId << " " << senderLaneIndex << std::endl;
                    // 獲取發送者當前速度  
                    double senderSpeed = m_client->TraCIAPI::vehicle.getSpeed(vehId);  
                      
                    // 設置自己的速度與發送者相同  
                    m_client->TraCIAPI::vehicle.setSpeed(m_id, senderSpeed);  
                      
                    // 設置視覺指示（黃色表示同速跟隨）  
                    libsumo::TraCIColor followColor;  
                    followColor.r = 255; followColor.g = 100; followColor.b = 0; followColor.a = 255;  
                    m_client->TraCIAPI::vehicle.setColor(m_id, followColor);  
                      
                    std::cout << "Vehicle " << m_id << " matching speed with DENM sender "   
                              << vehId << " (speed: " << senderSpeed << " m/s)" << std::endl;  
                      
                    // // 3秒後恢復正常顏色和速度控制  
                    // Simulator::Remove(m_speed_ev);  
                    // m_speed_ev = Simulator::Schedule(Seconds(3.0), &emergencyVehicleAlert::SetMaxSpeed, this);  
                }  
                break;  
            }  
        }  
          
        std::cout << "Received a new DENM." << std::endl;  
    }  
    m_rangeCheckEvent = Simulator::Schedule(Seconds(0.5), &emergencyVehicleAlert::RestoreOriginalColor, this);
  }
  // void
  // emergencyVehicleAlert::receiveDENM (denData denm, Address from)
  // {
  //   /* This is just a sample dummy receiveDENM function. The user can customize it to parse the content of a DENM when it is received. */
  //   (void) denm; // Contains the data received from the DENM
  //   (void) from; // Contains the address from which the DENM has been received
  //   // 從 DENM 標頭中獲取發送者的 Station ID
  //   long sender_station_id = denm.getDenmHeaderStationID();
  //   std::string sender_vehicle_id = "veh" + std::to_string(sender_station_id);

  //   // 只有當接收者不是發送者時，才改變顏色
  //   if (m_id != sender_vehicle_id)
  //   {
  //   libsumo::TraCIColor alertColor;
  //   alertColor.r = 255; // Red
  //   alertColor.g = 0;   // Green
  //   alertColor.b = 0;   // Blue
  //   alertColor.a = 255; // Alpha (不透明)
  //   m_client->TraCIAPI::vehicle.setColor(m_id, alertColor);
  //   std::cout << "=========================================" << std::endl;
  //   std::cout << "Received a new DENM from Station ID: " << denm.getDenmHeaderStationID() << std::endl;
  //   std::cout << "=========================================" << std::endl;

  //   // --- 1. 打印強制性的 Management 容器資訊 ---
  //   std::cout << "[Management Container]" << std::endl;
  //   std::cout << "  Detection Time: " << denm.getDenmMgmtDetectionTime() << " ms" << std::endl;
  //   std::cout << "  Latitude: " << denm.getDenmMgmtLatitude() << std::endl;
  //   std::cout << "  Longitude: " << denm.getDenmMgmtLongitude() << std::endl;
  //   std::cout << "  Validity: " << denm.getDenmMgmtValidityDuration() << " seconds" << std::endl;

  //   // --- 2. 檢查並打印可選的 Situation 容器資訊 ---
  //   if (denm.isDenmSituationDataSet())
  //   {
  //       // 先獲取 Situation 容器物件
  //       denData::denDataSituation situation = denm.getDenmSituationData_asn_types().getData();
        
  //       std::cout << "[Situation Container (Available)]" << std::endl;
  //       std::cout << "  Cause Code: " << situation.causeCode << std::endl;
  //       std::cout << "  Sub Cause Code: " << situation.subCauseCode << std::endl;
  //   }
  //   else
  //   {
  //       std::cout << "[Situation Container (Not Available)]" << std::endl;
  //   }

  //   // --- 3. 檢查並打印可選的 Location 容器資訊 ---
  //   if (denm.isDenmLocationDataSet())
  //   {
  //       // 先獲取 Location 容器物件
  //       denData::denDataLocation location = denm.getDenmLocationData_asn_types().getData();
        
  //       std::cout << "[Location Container (Available)]" << std::endl;
  //       if (location.eventSpeed.isAvailable())
  //       {
  //           std::cout << "  Event Speed: " << location.eventSpeed.getData().getValue() << std::endl;
  //       }
  //   }
  //   else
  //   {
  //       std::cout << "[Location Container (Not Available)]" << std::endl;
  //   }

  //   // --- 4. 檢查並打印可選的 Alacarte 容器資訊 (以此類推) ---
  //   if (denm.isDenmAlacarteDataSet())
  //   {
  //       std::cout << "[Alacarte Container (Available)]" << std::endl;
  //       // 你可以在這裡加入更多打印 Alacarte 內容的程式碼
  //   }
  //   else
  //   {
  //       std::cout << "[Alacarte Container (Not Available)]" << std::endl;
  //   }
  //   std::cout << std::endl; // 輸出一個空行，方便閱讀
  //   }
    
  // }

  void emergencyVehicleAlert::RestoreOriginalColor()  
  {  
      libsumo::TraCIColor normalColor;  
      normalColor.r = 0; normalColor.g = 225; normalColor.b = 255; normalColor.a = 255; // 藍色  
      m_client->TraCIAPI::vehicle.setColor(m_id, normalColor);  
      // std::cout << "Vehicle " << m_id << " restored to original color (out of DENM range)" << std::endl;  
  }

  void
  emergencyVehicleAlert::TriggerEmergencyDenm()
  {
    denData data;
    DEN_ActionID_t actionid;
    DENBasicService_error_t trigger_retval;
    libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
    pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x,pos.y);
    /* Set DENM mandatpry fields */
    data.setDenmMandatoryFields (compute_timestampIts(m_real_time),
                                  pos.x,
                                  pos.y);
     denData::denDataSituation situation_data;

    // 範例：設定事件為「緊急車輛接近」
    // 注意：CauseCode_emergencyVehicleApproaching 是一個假設的常數名稱，
    // 你需要根據你使用的函式庫 (asn1c) 找到實際的常數名稱。
    // 常見的可能是 CauseCode_t_emergencyVehicleApproaching 或類似的名稱。
    situation_data.causeCode = 97; // 根據 ETSI TS 101 628-6-1，97 代表 emergencyVehicleApproaching
    situation_data.subCauseCode = 0; // 此原因沒有次代碼

    /* 3. 將 situation 容器設定到主 data 物件中 */
    data.setDenmSituationData_asn_types(situation_data);

    /* Compute GeoArea for denms */
    GeoArea_t geoArea;
    // 直接使用上面已經獲取的位置
    geoArea.posLong = pos.x*DOT_ONE_MICRO;
    geoArea.posLat = pos.y*DOT_ONE_MICRO;
    geoArea.distA = 1000;
    geoArea.distB = 0;
    geoArea.angle = 0;
    geoArea.shape = CIRCULAR;
    m_denService.setGeoArea (geoArea);

    trigger_retval=m_denService.appDENM_trigger(data,actionid);
    
    if(trigger_retval==DENM_NO_ERROR)
    {
       m_updateDenmEvent = Simulator::Schedule(Seconds(1.0), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionid);
    }
  }

  void emergencyVehicleAlert::UpdateEmergencyDenm(DEN_ActionID_t actionID)  
{  
    denData data;  
    DENBasicService_error_t update_retval;  
      
    // 設置強制欄位  
    libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition(m_id);  
    pos = m_client->TraCIAPI::simulation.convertXYtoLonLat(pos.x, pos.y);  

    // 2. 使用有效位置設定強制性欄位
    data.setDenmMandatoryFields(compute_timestampIts(m_real_time),   
                               pos.y,   
                               pos.x);
      
    // 更新地理區域  
    GeoArea_t geoArea;  
    geoArea.posLong = pos.x * DOT_ONE_MICRO;  
    geoArea.posLat = pos.y * DOT_ONE_MICRO;  
    geoArea.distA = denm_transmit_distance; // 1000米半徑  
    geoArea.distB = 0;  
    geoArea.angle = 0;  
    geoArea.shape = CIRCULAR;  
    m_denService.setGeoArea(geoArea);  
      
    // 更新DENM  
    update_retval = m_denService.appDENM_update(data, actionID);  
    if (update_retval != DENM_NO_ERROR) {  
        NS_LOG_ERROR("Failed to update DENM. Error code: " << update_retval);  
        return; // 停止進一步的更新  
    }  
      
    std::cout << m_id << "  transmit a new DENM." << std::endl;  
      
    // 排程下一次更新  
    m_updateDenmEvent = Simulator::Schedule(Seconds(1.0), &emergencyVehicleAlert::UpdateEmergencyDenm, this, actionID);  
}

  void
  emergencyVehicleAlert::SetMaxSpeed ()
  {
    libsumo::TraCIColor normal;
    normal.r=0;normal.g=225;normal.b=255;normal.a=255; //blue
    m_client->TraCIAPI::vehicle.setColor (m_id, normal);
    m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);
  }

  void
  emergencyVehicleAlert::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
  {
    /* Implement CPM strategy here */
    m_cpm_received++;
    (void) from;
    //For every PO inside the CPM, if any
    bool POs_ok;
    //auto wrappedContainer = asn1cpp::makeSeq(WrappedCpmContainer);
    int wrappedContainer_size = asn1cpp::sequenceof::getSize(cpm->payload.cpmContainers);
    for (int i=0; i<wrappedContainer_size; i++)
      {
        auto wrappedContainer = asn1cpp::sequenceof::getSeq(cpm->payload.cpmContainers,WrappedCpmContainer,i);
        WrappedCpmContainer__containerData_PR present = asn1cpp::getField(wrappedContainer->containerData.present,WrappedCpmContainer__containerData_PR);
        if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
        {
          auto POcontainer = asn1cpp::getSeq(wrappedContainer->containerData.choice.PerceivedObjectContainer,PerceivedObjectContainer);
          int PObjects_size = asn1cpp::sequenceof::getSize(POcontainer->perceivedObjects);
          //std::cout << "["<< Simulator::Now ().GetSeconds ()<<"] " << m_id <<" received a new CPMv2 from " << asn1cpp::getField(cpm->header.stationId,long) << " with " << PObjects_size << " perceived objects." << std::endl;
          for(int j=0; j<PObjects_size;j++)
              {
               LDM::returnedVehicleData_t PO_data;
               auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
               PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);
               //If PO is already in local copy of vLDM
               if(m_LDM->lookup(asn1cpp::getField(PO_seq->objectId,long),PO_data) == LDM::LDM_OK)
                    {
                      //Add the new perception to the LDM
                      std::vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
                      if(std::find(associatedCVs.begin(), associatedCVs.end (), asn1cpp::getField(cpm->header.stationId,long)) == associatedCVs.end ())
                        associatedCVs.push_back (asn1cpp::getField(cpm->header.stationId,long));
                      PO_data.vehData.associatedCVs = OptionalDataItem<std::vector<long>>(associatedCVs);
                      m_LDM->insert (PO_data.vehData);
                    }
               else
                    {
                      //Translate CPM data to LDM format
                      m_LDM->insert(translateCPMdata(cpm,PO_seq,j));
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
    retval.stationID = asn1cpp::getField(object->objectId,long);
    retval.ID = std::to_string(retval.stationID);
    retval.vehicleLength = asn1cpp::getField(object->objectDimensionX->value,long);
    retval.vehicleWidth = asn1cpp::getField(object->objectDimensionY->value,long);
    retval.heading = asn1cpp::getField(object->angles->zAngle.value,double) / DECI;
    retval.xSpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.xVelocity.value,long));
    retval.xSpeedAbs.setData (asn1cpp::getField(object->velocity->choice.cartesianVelocity.yVelocity.value,long));
    retval.speed_ms = (sqrt (pow(retval.xSpeedAbs.getData(),2) +
                             pow(retval.ySpeedAbs.getData(),2)))/CENTI;

    libsumo::TraCIPosition fromPosition = m_client->TraCIAPI::simulation.convertLonLattoXY (asn1cpp::getField(cpm->payload.managementContainer.referencePosition.longitude,double)/DOT_ONE_MICRO,
                                                                                           asn1cpp::getField(cpm->payload.managementContainer.referencePosition.latitude,double)/DOT_ONE_MICRO);
    libsumo::TraCIPosition objectPosition = fromPosition;
    objectPosition.x += asn1cpp::getField(object->position.xCoordinate.value,long)/CENTI;
    objectPosition.y += asn1cpp::getField(object->position.yCoordinate.value,long)/CENTI;
    objectPosition = m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x,objectPosition.y);
    retval.lon = objectPosition.x;
    retval.lat = objectPosition.y;

    retval.camTimestamp = asn1cpp::getField(cpm->payload.managementContainer.referenceTime,long);
    retval.timestamp_us = Simulator::Now().GetMicroSeconds () - (asn1cpp::getField(object->measurementDeltaTime,long)*1000);
    retval.stationType = StationType_passengerCar;
    retval.perceivedBy.setData(asn1cpp::getField(cpm->header.stationId,long));

        return retval;
  }


  }





