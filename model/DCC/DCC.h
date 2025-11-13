#ifndef NS3_DCC_H
#define NS3_DCC_H

#include <string>
#include <vector>
#include "ns3/MetricSupervisor.h"
#include "ns3/wifi-net-device.h"
//#include "ns3/nr-net-device.h"
#include "ns3/net-device.h"
#include "ns3/wifi-phy.h"
//#include "ns3/nr-ue-phy.h"
#include "ns3/traci-client.h"
#include "ns3/BSMap.h"
//#include "ns3/nr-module.h"

namespace ns3 {

/**
 * \ingroup automotive
 *
 * \brief This class implements the Decentralized Congestion Control (DCC) algorithm.
 *
 * This class provides capabilities for computing both the Reactive DCC and the Proactive DCC.
 */

class DCC : public Object
{
public:

  typedef enum ReactiveState
  {
    Relaxed,
    Active1,
    Active2,
    Active3,
    Restrictive
  } ReactiveState;

  typedef struct ReactiveParameters {
    double cbr_threshold;
    double tx_power;
    double data_rate;
    long tx_inter_packet_time;
    double sensitivity;
  } ReactiveParameters;

  static TypeId GetTypeId(void);
  /**
   * \brief Default constructor
   *
   */
  DCC ();
  ~DCC();

  const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_1ms = 
  {
    // Values represent: CBR threshold, Tx Power [dBm], Data Rate [Mbit/s], Tx Inter Packet Time [ms], Rx Sensitivity [dBm]
    {Relaxed,     {0.3, 30.0, -1, 100, -95.0}},
    {Active1,     {0.4, 24.0, -1, 200,  -95.0}},
    {Active2,     {0.5, 18.0, -1, 400, -95.0}},
    {Active3,     {0.6, 12.0, -1, 500, -95.0}},
    {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
  };

  const std::unordered_map<ReactiveState, ReactiveParameters> m_reactive_parameters_Ton_500_us = 
  {
    {Relaxed,     {0.3, 30.0, -1, 50, -95.0}},
    {Active1,     {0.4, 24.0, -1, 100,  -95.0}},
    {Active2,     {0.5, 18.0, -1, 200, -95.0}},
    {Active3,     {0.65, 12.0, -1, 250, -95.0}},
    {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
  };

  /**
    * \brief Setup DCC
    *
    * \param item_id item id
    * \param node node object
    * \param modality modality of DCC, can be "reactive" or "adaptive"
    * \param dcc_interval time interval DCC
    * \param traci_client pointer to MetricSupervisor 
    */
  void SetupDCC(std::string item_id, Ptr<Node> node, std::string modality, uint32_t dcc_interval, Ptr<MetricSupervisor> traci_client); 
 /**
    * \brief Set the CAM Basic Service
    *
    * \param nodeID id of the node
    * \param caBasicService basic service for CAMs
    */
  void AddCABasicService(Ptr<CABasicService> caBasicService) {m_caService = caBasicService;};
  /**
    * \brief Set the CAM Basic Service (Version 1)
    *
    * \param nodeID id of the node
    * \param caBasicService basic service for CAMs
    */
  void AddCABasicServiceV1(Ptr<CABasicServiceV1> caBasicService) {m_caServiceV1 = caBasicService;};
  /**
    * \brief Set the CPM Basic Service
    *
    * \param nodeID id of the node
    * \param cpBasicService basic service for CPMs
    */
  void AddCPBasicService(Ptr<CPBasicService> cpBasicService) {m_cpService = cpBasicService;};
  /**
    * \brief Set the CPM Basic Service (Version 1)
    *
    * \param nodeID id of the node
    * \param cpBasicService basic service for CPMs
    */
  void AddCPBasicService(Ptr<CPBasicServiceV1> cpBasicService) {m_cpServiceV1 = cpBasicService;};
  /**
    * \brief Set the VRU Basic Service
    *
    * \param nodeID id of the node
    * \param vruBasicService basic service for VRUs
    */
  void AddVRUBasicService(std::string nodeID, Ptr<VRUBasicService> vruBasicService) {m_vruService = vruBasicService;};

  void StartDCC();

private:

  /**
   * \brief Start the reactive DCC mechanism
   *
   */
  void reactiveDCC();
  /**
   * \brief Start the adaptive DCC mechanism
   *
   */
  void adaptiveDCC();
  /**
   * \brief Start the CBR check for the adaptive DCC mechanism
   *
   */
  void adaptiveDCCcheckCBR();

  std::unordered_map<ReactiveState, ReactiveParameters> getConfiguration(double Ton, double currentCBR);

  std::string m_item_id;
  Ptr<Node> m_node;

  std::string m_modality = ""; //!< Boolean to indicate if the DCC is reactive or adaptive
  uint32_t m_dcc_interval = -1; //!< Time interval for DCC
  Ptr<MetricSupervisor> m_metric_supervisor = NULL; //!< Pointer to the MetricSupervisor object
  Ptr<CABasicService> m_caService; //!< Pointer to the CABasicService object
  Ptr<CABasicServiceV1> m_caServiceV1; //!< Pointer to the CABasicService object
  Ptr<CPBasicService> m_cpService; //!< Pointer to the CPBasicService object
  Ptr<CPBasicServiceV1> m_cpServiceV1; //!< Pointer to the CPBasicService object
  Ptr<VRUBasicService> m_vruService; //!< Pointer to the VRUBasicService object
  ReactiveState m_current_state = ReactiveState::Relaxed;

  double m_CBR_its = -1;
  double m_alpha = 0.016;
  double m_beta = 0.0012;
  double m_CBR_target = 0.68;
  double m_delta_max = 0.03;
  double m_delta_min = 0.0006;
  double m_Gmax = 0.0005;
  double m_Gmin = -0.00025;
  uint32_t m_T_CBR = 100; // Check the CBR value each 100 ms for Adaptive DCC from standard suggestion
  double m_delta = 0;
  double m_previous_cbr;

};

}



#endif //NS3_DCC_H
