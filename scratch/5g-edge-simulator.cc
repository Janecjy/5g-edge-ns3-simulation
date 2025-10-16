// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @ingroup examples
 * @file cttc-nr-demo.cc
 * @brief A cozy, simple, NR demo (in a tutorial style)
 *
 * Notice: this entire program uses technical terms defined by the 3GPP TS 38.300 [1].
 *
 * This example describes how to setup a simulation using the 3GPP channel model from TR 38.901 [2].
 * This example consists of a simple grid topology, in which you
 * can choose the number of gNbs and UEs. Have a look at the possible parameters
 * to know what you can configure through the command line.
 *
 * With the default configuration, the example will create two flows that will
 * go through two different subband numerologies (or bandwidth parts). For that,
 * specifically, two bands are created, each with a single CC, and each CC containing
 * one bandwidth part.
 *
 * The example will print on-screen the end-to-end result of one (or two) flows,
 * as well as writing them on a file.
 *
 * \code{.unparsed}
$ ./ns3 run "cttc-nr-demo --PrintHelp"
    \endcode
 *
 */

// NOLINTBEGIN
// clang-format off

/**
 * Useful references that will be used for this tutorial:
 * [1] <a href="https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=3191">3GPP TS 38.300</a>
 * [2] <a href="https://portal.3gpp.org/desktopmodules/Specifications/SpecificationDetails.aspx?specificationId=3173">3GPP channel model from TR 38.901</a>
 * [3] <a href="https://www.nsnam.org/docs/release/3.38/tutorial/html/tweaking.html#using-the-logging-module">ns-3 documentation</a>
 */

// clang-format on
// NOLINTEND

/*
 * Include part. Often, you will have to include the headers for an entire module;
 * do that by including the name of the module you need with the suffix "-module.h".
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/abort.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/network-module.h"

#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <iomanip>
#include <limits>

/*
 * Use, always, the namespace ns3. All the NR classes are inside such namespace.
 */
using namespace ns3;

/*
 * Configuration structure to hold testbed parameters
 */
struct TestbedConfig {
    uint32_t num_ues = 1;
    std::vector<uint32_t> smec_ue_indices;
    std::vector<uint32_t> transcoding_ue_indices;
    std::vector<uint32_t> video_detection_ue_indices;
    std::vector<uint32_t> video_sr_ue_indices;
    std::vector<uint32_t> file_transfer_ue_indices;
    std::vector<uint32_t> throughput_ue_indices;
    std::vector<uint32_t> ping_ue_indices;
    uint32_t max_cpus = 24;
};

/*
 * Simple JSON parser for configuration file
 */
std::vector<uint32_t> ParseIndices(const std::string& indices_str) {
    std::vector<uint32_t> indices;
    if (indices_str.empty()) {
        return indices;
    }
    
    std::stringstream ss(indices_str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) {
            indices.push_back(std::stoi(item));
        }
    }
    return indices;
}

TestbedConfig LoadConfig(const std::string& configPath) {
    TestbedConfig config;
    std::ifstream file(configPath);
    
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file: " << configPath << ". Using defaults." << std::endl;
        return config;
    }
    
    std::string line;
    std::map<std::string, std::string> configMap;
    
    // Simple JSON-like parsing (assumes one key-value per line)
    while (std::getline(file, line)) {
        // Remove whitespace and quotes
        line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
        if (line.empty() || line[0] == '{' || line[0] == '}') continue;
        
        // Remove trailing comma
        if (line.back() == ',') line.pop_back();
        
        size_t colonPos = line.find(':');
        if (colonPos != std::string::npos) {
            std::string key = line.substr(0, colonPos);
            std::string value = line.substr(colonPos + 1);
            
            // Remove quotes
            key.erase(std::remove(key.begin(), key.end(), '"'), key.end());
            value.erase(std::remove(value.begin(), value.end(), '"'), value.end());
            
            configMap[key] = value;
        }
    }
    
    // Parse configuration values
    if (configMap.find("num_ues") != configMap.end()) {
        config.num_ues = std::stoi(configMap["num_ues"]);
    }
    if (configMap.find("max_cpus") != configMap.end()) {
        config.max_cpus = std::stoi(configMap["max_cpus"]);
    }
    
    // Parse UE indices
    config.smec_ue_indices = ParseIndices(configMap["smec_ue_indices"]);
    config.transcoding_ue_indices = ParseIndices(configMap["transcoding_ue_indices"]);
    config.video_detection_ue_indices = ParseIndices(configMap["video_detection_ue_indices"]);
    config.video_sr_ue_indices = ParseIndices(configMap["video_sr_ue_indices"]);
    config.file_transfer_ue_indices = ParseIndices(configMap["file_transfer_ue_indices"]);
    config.throughput_ue_indices = ParseIndices(configMap["throughput_ue_indices"]);
    config.ping_ue_indices = ParseIndices(configMap["ping_ue_indices"]);
    
    std::cout << "Loaded config: " << config.num_ues << " UEs, " << config.max_cpus << " max CPUs" << std::endl;
    
    return config;
}

/*
 * Global variables and callback functions for ping RTT measurement
 */
struct PingResult {
    uint32_t ueId;
    uint32_t sequence;
    Time rtt;
    Time timestamp;
};

std::vector<PingResult> pingResults;

void PingRttCallback(uint16_t nodeId, Time rtt)
{
    PingResult result;
    result.ueId = nodeId;
    result.sequence = 0; // Will be incremented by the trace source
    result.rtt = rtt;
    result.timestamp = Simulator::Now();
    pingResults.push_back(result);
    
    std::cout << "Ping RTT: UE " << nodeId << " time=" << rtt.GetMilliSeconds() << " ms" << std::endl;
}

// static void
// PingRtt(std::string context, uint16_t seqNo, Time rtt)
// {
//     NS_LOG_UNCOND("Received " << seqNo << " Response with RTT = " << rtt);
// }


/*
 * With this line, we will be able to see the logs of the file by enabling the
 * component "CttcNrDemo".
 * Further information on how logging works can be found in the ns-3 documentation [3].
 */
NS_LOG_COMPONENT_DEFINE("CttcNrDemo");

int
main(int argc, char* argv[])
{
    LogComponentEnable("NrNoBackhaulEpcHelper", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Ipv4", LOG_LEVEL_DEBUG);
    LogComponentEnable("Ipv4StaticRouting", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Ipv4", LOG_LEVEL_DEBUG);
    // LogComponentEnable("Packet", LOG_LEVEL_DEBUG);
    /*
     * Variables that represent the parameters we will accept as input by the
     * command line. Each of them is initialized with a default value, and
     * possibly overridden below when command-line arguments are parsed.
     */
    // Scenario parameters (that we will use inside this script):
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 2;
    bool logging = false;

    // Traffic parameters (that we will use inside this script):
    uint32_t pingSize = 64;  // Standard ping packet size
    uint32_t pingCount = 3;  // Number of pings to send (reduced for faster testing)
    double pingInterval = 1.0;  // Ping interval in seconds
    
    // Throughput test parameters:
    uint32_t throughputPacketSize = 1024;  // Throughput test packet size
    uint32_t throughputDataRate = 1000000000;  // Data rate in bps (1 Gbps default)
    Time throughputDuration = Seconds(10);  // Duration of throughput test

    // Simulation parameters. Please don't use double to indicate seconds; use
    // ns-3 Time values which use integers to avoid portability issues.
    Time simTime = MilliSeconds(23000);  // 5 seconds default for faster testing
    Time udpAppStartTime = MilliSeconds(400);

    // NR parameters configured for Band 78 (3.5 GHz) testbed
    // TDD mode with 80MHz bandwidth and 2x2 MIMO configuration
    // Reference: 3GPP TS 38.104 for Band 78 specifications
    uint16_t numerologyBwp1 = 1;  // Numerology 1 (30 kHz SCS) typical for Band 78
    double centralFrequencyBand1 = 3.5e9;  // Band 78 center frequency (3.3-3.8 GHz range)
    double bandwidthBand1 = 80e6;  // 80MHz bandwidth as per testbed
    double totalTxPower = 35;

    // TDD Pattern for Band 78 configuration
    // Default pattern: 7 DL slots, 1 Special slot, 2 UL slots (typical for eMBB applications)
    std::string tddPattern = "DL|DL|DL|DL|DL|DL|DL|S|UL|UL|";

    // Where we will store the output files.
    std::string simTag = "default";
    std::string outputDir = "./";

    // Configuration file for testbed setup
    std::string configFile = "config/baseline_all_tasks.json";

    // External server configuration
    bool useExternalServer = false;
    std::string externalServerIp = "10.10.1.2";

    /*
     * From here, we instruct the ns3::CommandLine class of all the input parameters
     * that we may accept as input, as well as their description, and the storage
     * variable.
     */
    CommandLine cmd(__FILE__);

    cmd.AddValue("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
    cmd.AddValue("ueNumPergNb", "The number of UE per gNb in multiple-ue topology", ueNumPergNb);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("pingSize",
                 "Ping packet size in bytes",
                 pingSize);
    cmd.AddValue("pingCount",
                 "Number of ping packets to send",
                 pingCount);
    cmd.AddValue("pingInterval",
                 "Interval between ping packets in seconds",
                 pingInterval);
    cmd.AddValue("throughputPacketSize",
                 "Throughput test packet size in bytes",
                 throughputPacketSize);
    cmd.AddValue("throughputDataRate",
                 "Throughput test data rate in bps",
                 throughputDataRate);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("centralFrequencyBand1",
                 "The system frequency to be used in Band 78",
                 centralFrequencyBand1);
    cmd.AddValue("bandwidthBand1", "The system bandwidth to be used in Band 78", bandwidthBand1);
    cmd.AddValue("totalTxPower",
                 "total tx power that will be proportionally assigned to"
                 " bands, CCs and bandwidth parts depending on each BWP bandwidth ",
                 totalTxPower);
    cmd.AddValue("tddPattern", 
                 "TDD pattern for Band 78 (e.g., 'DL|DL|DL|DL|DL|DL|DL|S|UL|UL|'). "
                 "DL=Downlink, UL=Uplink, S=Special, F=Flexible", 
                 tddPattern);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("configFile", "path to JSON configuration file for testbed setup", configFile);
    cmd.AddValue("useExternalServer", 
                 "Use external physical server instead of simulated remote host", 
                 useExternalServer);
    cmd.AddValue("externalServerIp", 
                 "IP address of external server (when useExternalServer=true)", 
                 externalServerIp);

    // Parse the command line
    cmd.Parse(argc, argv);

    // Load testbed configuration
    TestbedConfig testbedConfig = LoadConfig(configFile);
    
    // Update UE number based on config
    ueNumPergNb = testbedConfig.num_ues;

    /*
     * Check if the frequency is in the allowed range.
     * If you need to add other checks, here is the best position to put them.
     */
    NS_ABORT_IF(centralFrequencyBand1 < 0.5e9 && centralFrequencyBand1 > 100e9);

    /*
     * If the logging variable is set to true, enable the log of some components
     * through the code. The same effect can be obtained through the use
     * of the NS_LOG environment variable:
     *
     * export NS_LOG="UdpClient=level_info|prefix_time|prefix_func|prefix_node:UdpServer=..."
     *
     * Usually, the environment variable way is preferred, as it is more customizable,
     * and more expressive.
     */
    if (logging)
    {
        LogComponentEnable("Ping", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
    }

    /*
     * In general, attributes for the NR module are typically configured in NrHelper.  However, some
     * attributes need to be configured globally through the Config::SetDefault() method. Below is
     * an example: if you want to make the RLC buffer very large, you can pass a very large integer
     * here.
     */
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    /*
     * Create the scenario. In our examples, we heavily use helpers that setup
     * the gnbs and ue following a pre-defined pattern. Please have a look at the
     * GridScenarioHelper documentation to see how the nodes will be distributed.
     */
    int64_t randomStream = 1;
    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);
    // All units below are in meters
    gridScenario.SetHorizontalBsDistance(10.0);
    gridScenario.SetVerticalBsDistance(10.0);
    gridScenario.SetBsHeight(10);
    gridScenario.SetUtHeight(1.5);
    // must be set before BS number
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNbNum);
    gridScenario.SetUtNumber(ueNumPergNb * gNbNum);
    gridScenario.SetScenarioHeight(3); // Create a 3x3 scenario where the UE will
    gridScenario.SetScenarioLength(3); // be distributed.
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    /*
     * Create NodeContainers for different application types based on config.
     * UEs will be assigned to different containers based on their indices in the config file.
     */
    NodeContainer ueTranscodingContainer;
    NodeContainer ueVideoDetectionContainer;
    NodeContainer ueVideoSRContainer;
    NodeContainer ueFileTransferContainer;
    NodeContainer ueSMECContainer;
    NodeContainer ueThroughputContainer;
    NodeContainer uePingContainer;
    // Helper function to check if UE index is in a vector
    auto isUEInVector = [](uint32_t ueIndex, const std::vector<uint32_t>& vec) {
        return std::find(vec.begin(), vec.end(), ueIndex) != vec.end();
    };

    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);
        uint32_t ueIndex = j + 1; // Config uses 1-based indexing
        
        // Assign UEs to containers based on configuration
        if (isUEInVector(ueIndex, testbedConfig.smec_ue_indices)) {
            ueSMECContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.transcoding_ue_indices)) {
            ueTranscodingContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.video_detection_ue_indices)) {
            ueVideoDetectionContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.video_sr_ue_indices)) {
            ueVideoSRContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.file_transfer_ue_indices)) {
            ueFileTransferContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.throughput_ue_indices)) {
            ueThroughputContainer.Add(ue);
        }
        if (isUEInVector(ueIndex, testbedConfig.ping_ue_indices)) {
            uePingContainer.Add(ue);
        }
    }

    std::cout << "UE Assignment Summary:" << std::endl;
    std::cout << "  SMEC UEs: " << ueSMECContainer.GetN() << std::endl;
    std::cout << "  Transcoding UEs: " << ueTranscodingContainer.GetN() << std::endl;
    std::cout << "  Detection UEs: " << ueVideoDetectionContainer.GetN() << std::endl;
    std::cout << "  SR UEs: " << ueVideoSRContainer.GetN() << std::endl;
    std::cout << "  File Transfer UEs: " << ueFileTransferContainer.GetN() << std::endl;
    std::cout << "  Throughput Test UEs: " << ueThroughputContainer.GetN() << std::endl;
    std::cout << "  Total UEs created: " << gridScenario.GetUserTerminals().GetN() << std::endl;

    /*
     * TODO: Add a print, or a plot, that shows the scenario.
     */
    NS_LOG_INFO("Creating " << gridScenario.GetUserTerminals().GetN() << " user terminals and "
                            << gridScenario.GetBaseStations().GetN() << " gNBs");

    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - nrEpcHelper, which will setup the core network
     * - IdealBeamformingHelper, which takes care of the beamforming part
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     * - NrChannelHelper, which takes care of the spectrum channel
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    /*
     * Spectrum division. We create one operational band for Band 78 (3.5 GHz) containing
     * one component carrier, and the CC containing a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * The spectrum part length is specified by the input parameters.
     * The band will use the UMa channel modeling for sub-6 GHz frequencies.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; // Single CC for Band 78

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
    // a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf1(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    numCcPerBand);

    // Create the band and install the channel into it
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);

    /*
     * The configured spectrum division is:
     * ------------Band1--------------|--------------Band2-----------------
     * ------------CC1----------------|--------------CC2-------------------
     * ------------BWP1---------------|--------------BWP2------------------
     */

    /*
     * Start to account for the bandwidth used by the example, as well as
     * the total power that has to be divided among the BWPs.
     */
    double x = pow(10, totalTxPower / 10);
    double totalBandwidth = bandwidthBand1;
    /**
     * The channel is configured by this helper using a combination of the scenario, the channel
     * condition model, and the fading model.
     */

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    // Use "UMa" (Urban Macro) for Band 78 (3.5 GHz) representation
    // Band 78 is typically used in macro cell deployments with larger coverage areas
    channelHelper->ConfigureFactories("UMa", "Default", "ThreeGpp");
    /**
     * Use channelHelper API to define the attributes for the channel model (condition, pathloss and
     * spectrum)
     */
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    
    // Single band configuration for Band 78
    channelHelper->AssignChannelsToBands({band1});
    allBwps = CcBwpCreator::GetAllBwps({band1});

    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    // Disable strict packet checking to avoid header format issues with FdNetDevice
    // Packet::EnableChecking();
    // Packet::EnablePrinting();

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Antennas for all the UEs - configured for 2x2 MIMO (testbed specification)
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));  // 2x1 = 2 antennas
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs - configured for 2x2 MIMO base station side
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));  // 2x1 = 2 antennas
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // For single band operation, all traffic uses BWP 0
    uint32_t bwpIdForLowLat = 0;
    uint32_t bwpIdForVoice = 0;

    // gNb routing between Bearer and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",
                                                 UintegerValue(bwpIdForLowLat));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForVoice));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLat));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForVoice));

    /*
     * We miss many other parameters. By default, not configuring them is equivalent
     * to use the default values. Please, have a look at the documentation to see
     * what are the default values for all the attributes you are not seeing here.
     */

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // NOT PRESENT IN THIS SIMPLE EXAMPLE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */

    NetDeviceContainer gnbNetDev =
        nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    // Install UE devices for all user terminals (we'll manage traffic types through bearers)
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(gridScenario.GetUserTerminals(), allBwps);

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // Get the first netdevice (gnbNetDev.Get (0)) and the first bandwidth part (0)
    // and set the attribute.
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("TxPower", DoubleValue(10 * log10((bandwidthBand1 / totalBandwidth) * x)));
    
    // Configure TDD Pattern for Band 78 testbed simulation
    // Pattern format: DL=Downlink, UL=Uplink, S=Special(flexible), F=Flexible
    // This pattern provides good balance for 5G applications: 7 DL, 2 UL, 1 Special slot per 10ms frame
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)
        ->SetAttribute("Pattern", StringValue(tddPattern));

    // From here, it is standard NS3. In the future, we will create helpers
    // for this part as well.

    // Use 25 GbE to match real testbed network speed between RAN and edge servers
    auto [edge_server, edge_server_Ipv4Address] =
        nrEpcHelper->SetupRemoteHost("25Gb/s", 2500, Seconds(0.000));
    std::cout << "Edge server created with IP: " << edge_server_Ipv4Address << std::endl;

    // Function to setup a ethernet device in the edge_server to connect to the remote host
    std::string emuMode("raw");
    Ipv4Address remoteIp("10.10.1.2");
    Ipv4Address localIp("10.10.1.11");  // Edge server will use .11
    Ipv4Mask localMask("255.255.255.0");
    std::string localGateway("10.10.1.1");  // Interface IP is the gateway
    std::string deviceName("enp94s0f1np1");

    FdNetDeviceHelper* helper = nullptr;
    auto raw = new EmuFdNetDeviceHelper;
    raw->SetDeviceName(deviceName);
    helper = raw;

    if (helper == nullptr)
    {
        NS_ABORT_MSG(emuMode << " not supported.");
    }

    NetDeviceContainer devices = helper->Install(edge_server);
    // get the first device
    Ptr<NetDevice> device = devices.Get(0);
    device->SetAttribute("Address", Mac48AddressValue(Mac48Address::Allocate()));
    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    //
    // Add a default internet stack to the node.  This gets us the ns-3 versions
    // of ARP, IPv4, ICMP, UDP and TCP.
    //
    NS_LOG_INFO("Retrieve edge_server Internet Stack, IPv4 and routing");
    Ptr<Ipv4> edge_server_ipv4 = edge_server->GetObject<Ipv4>();
    uint32_t ethernet_interface_ID = edge_server_ipv4->AddInterface(device);
    Ipv4InterfaceAddress address = Ipv4InterfaceAddress(localIp, localMask);
    edge_server_ipv4->AddAddress(ethernet_interface_ID, address);
    edge_server_ipv4->SetMetric(ethernet_interface_ID, 0);
    edge_server_ipv4->SetUp(ethernet_interface_ID);
    Ipv4Address gateway(localGateway.c_str());
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(edge_server_ipv4);
    staticRouting->AddNetworkRouteTo(remoteIp, localMask, gateway, ethernet_interface_ID, 1);
    edge_server_ipv4->SetAttribute("IpForward", BooleanValue(true));

    InternetStackHelper internet;

    internet.Install(gridScenario.GetUserTerminals());

    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
    
    // Setup default gateway for UEs
    Ipv4InterfaceContainer::Iterator i;
    for (i = ueIpIface.Begin (); i != ueIpIface.End (); ++i){
        std::pair<Ptr<Ipv4>, uint32_t> ueIpv4_pair = *i;
        Ptr<Ipv4> ueIpv4 = ueIpv4_pair.first;
        uint32_t ueInterface = ueIpv4_pair.second;
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        staticRouting->SetDefaultRoute(remoteIp, ueInterface);
    }

    // Setup routing on pgw
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    std::cout << "pgw node id: " << pgw->GetId() << std::endl;
    Ptr<Ipv4> pgwIpv4 = pgw->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> pgwStaticRouting = ipv4RoutingHelper.GetStaticRouting(pgwIpv4);
    pgwStaticRouting->AddNetworkRouteTo(remoteIp, Ipv4Mask("255.255.255.0") , 3);
    
    // Setup routing on sgw
    Ptr<Node> sgw = nrEpcHelper->GetSgwNode();
    std::cout << "sgw node id: " << sgw->GetId() << std::endl;
    Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> sgwStaticRouting = ipv4RoutingHelper.GetStaticRouting(sgwIpv4);
    sgwStaticRouting->AddNetworkRouteTo(remoteIp, Ipv4Mask("255.255.255.0") , 3);
    
    /*
     * Traffic part. Install applications based on UE type:
     * - Throughput UEs: UDP client/server for max throughput testing
     * - All other UEs (default): Ping applications for RTT measurement
     */

    ApplicationContainer allApps;

    std::cout << "Application Assignment:" << std::endl;
    std::cout << "  Ping UEs: " << uePingContainer.GetN() << std::endl;

    /*
     * Install Throughput Test Applications (UDP Client/Server for max throughput)
     */
    // if (ueThroughputContainer.GetN() > 0) {
    //     std::cout << "Setting up throughput test applications..." << std::endl;
        
    //     // Throughput test uses UDP with high data rates
    //     uint16_t dlPortThroughput = 9999;
    //     uint16_t ulPortThroughput = 9998;

    //     // Downlink throughput test (remote host -> UE)
    //     UdpServerHelper dlThroughputServer(dlPortThroughput);
    //     ApplicationContainer dlThroughputServerApps = dlThroughputServer.Install(ueThroughputContainer);
    //     allApps.Add(dlThroughputServerApps);

    //     UdpClientHelper dlThroughputClient;
    //     dlThroughputClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    //     dlThroughputClient.SetAttribute("PacketSize", UintegerValue(throughputPacketSize));
    //     dlThroughputClient.SetAttribute("Interval", 
    //                                    TimeValue(Seconds(throughputPacketSize * 8.0 / throughputDataRate)));

    //     // Uplink throughput test (UE -> remote host)
    //     UdpServerHelper ulThroughputServer(ulPortThroughput);
    //     ApplicationContainer ulThroughputServerApps = ulThroughputServer.Install(edge_server);
    //     allApps.Add(ulThroughputServerApps);

    //     UdpClientHelper ulThroughputClient;
    //     ulThroughputClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    //     ulThroughputClient.SetAttribute("PacketSize", UintegerValue(throughputPacketSize));
    //     ulThroughputClient.SetAttribute("Interval", 
    //                                    TimeValue(Seconds(throughputPacketSize * 8.0 / throughputDataRate)));

    //     // Install clients for each throughput UE
    //     for (uint32_t i = 0; i < ueThroughputContainer.GetN(); ++i) {
    //         // Find the UE index in the full UE list
    //         Ptr<Node> throughputUE = ueThroughputContainer.Get(i);
    //         uint32_t ueIdx = 0;
    //         for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j) {
    //             if (gridScenario.GetUserTerminals().Get(j) == throughputUE) {
    //                 ueIdx = j;
    //                 break;
    //             }
    //         }
            
    //         Address ueAddress = ueIpIface.GetAddress(ueIdx);

    //         // Downlink client (remote host sends to UE)
    //         dlThroughputClient.SetAttribute("Remote", 
    //             AddressValue(addressUtils::ConvertToSocketAddress(ueAddress, dlPortThroughput)));
    //         allApps.Add(dlThroughputClient.Install(edge_server));

    //         // Uplink client (UE sends to remote host)
    //         ulThroughputClient.SetAttribute("Remote", 
    //             AddressValue(addressUtils::ConvertToSocketAddress(edge_server_Ipv4Address, ulPortThroughput)));
    //         allApps.Add(ulThroughputClient.Install(throughputUE));
    //     }
    // }

    /*
     * Install Ping Applications (default for unassigned UEs)
     */
    if (uePingContainer.GetN() > 0) {
        std::cout << "Setting up ping applications..." << std::endl;
        
        ApplicationContainer pingApps;

        /*
         * Configure ping application attributes
         */
        PingHelper pingHelper(remoteIp);

        pingHelper.SetAttribute("Count", UintegerValue(pingCount));
        pingHelper.SetAttribute("Size", UintegerValue(pingSize));
        pingHelper.SetAttribute("Interval", TimeValue(Seconds(pingInterval)));

        // Install ping applications on ping UEs
        pingApps = pingHelper.Install(uePingContainer);
        allApps.Add(pingApps);

        // Connect ping RTT trace to our callback function
        Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::Ping/Rtt",
                                      MakeCallback(&PingRttCallback));
    }

    // Start all applications
    allApps.Start(udpAppStartTime);
    allApps.Stop(simTime);

    // NS_LOG_INFO("Create fd-emu-Ping Application");
    // Ptr<Ping> app = CreateObject<Ping>();
    // app->SetAttribute("Destination", AddressValue(remoteIp));
    // app->SetAttribute("VerboseMode", EnumValue(Ping::VerboseMode::VERBOSE));
    // app->SetAttribute("InterfaceAddress", AddressValue(edge_server_ipv4->GetAddress(ethernet_interface_ID, 0).GetAddress()));
    // edge_server->AddApplication(app);
    // app->SetStartTime(Seconds(21));
    // app->SetStopTime(Seconds(22));
    // Names::Add("app", app);

    //
    // Hook a trace to print something when the response comes back.
    //
    // Config::Connect("/Names/app/Rtt", MakeCallback(&PingRtt));

    //
    // Enable a promiscuous pcap trace to see what is coming and going on our device.
    //
    helper->EnablePcap(emuMode + "-emu-ping", device, true);




    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    // Install FlowMonitor if we have throughput UEs for flow statistics
    Ptr<ns3::FlowMonitor> monitor = nullptr;
    FlowMonitorHelper flowmonHelper;
    if (ueThroughputContainer.GetN() > 0) {
        NodeContainer endpointNodes;
        endpointNodes.Add(edge_server);
        endpointNodes.Add(gridScenario.GetUserTerminals());
        monitor = flowmonHelper.Install(endpointNodes);
        monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
        monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
        monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));
    }

    Simulator::Stop(simTime);
    Simulator::Run();

    /*
     * To check what was installed in the memory, i.e., BWPs of gNB Device, and its configuration.
     * Example is: Node 1 -> Device 0 -> BandwidthPartMap -> {0,1} BWPs -> NrGnbPhy -> Numerology,
    GtkConfigStore config;
    config.ConfigureAttributes ();
    */

    // Print application results (ping and/or throughput)
    std::ofstream outFile;
    std::string filename = outputDir + "/" + simTag;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    outFile << "=== 5G Edge Simulation Results ===" << std::endl;
    outFile << "Target: " << edge_server_Ipv4Address << std::endl;
    if (useExternalServer) {
        outFile << "Server Type: External Physical Server" << std::endl;
    } else {
        outFile << "Server Type: Simulated Remote Host" << std::endl;
    }
    outFile << "RAN-Server Link: 25 GbE" << std::endl;
    outFile << "Total UEs: " << gridScenario.GetUserTerminals().GetN() << std::endl;
    outFile << "Ping UEs: " << uePingContainer.GetN() << std::endl;
    outFile << "Throughput UEs: " << ueThroughputContainer.GetN() << std::endl;
    outFile << "Ping UEs: " << uePingContainer.GetN() << std::endl;
    outFile << std::endl;

    // Print ping results if any ping UEs exist
    if (uePingContainer.GetN() > 0 && pingResults.size() > 0) {
        double totalRtt = 0.0;
        double minRtt = std::numeric_limits<double>::max();
        double maxRtt = 0.0;
        uint32_t successfulPings = pingResults.size();

        outFile << "=== Ping Test Results ===" << std::endl;
        outFile << "Packet size: " << pingSize << " bytes" << std::endl;
        outFile << std::endl;

        outFile << "Individual ping results:" << std::endl;
        for (const auto& result : pingResults) {
            double rttMs = result.rtt.GetMilliSeconds();
            outFile << "UE " << result.ueId << " seq=" << result.sequence 
                    << " time=" << std::fixed << std::setprecision(3) << rttMs << " ms" << std::endl;
            
            totalRtt += rttMs;
            minRtt = std::min(minRtt, rttMs);
            maxRtt = std::max(maxRtt, rttMs);
        }

        double avgRtt = totalRtt / successfulPings;
        
        outFile << std::endl;
        outFile << "=== Ping Summary Statistics ===" << std::endl;
        outFile << "Packets transmitted: " << (pingCount * uePingContainer.GetN()) << std::endl;
        outFile << "Packets received: " << successfulPings << std::endl;
        outFile << "Packet loss: " << std::fixed << std::setprecision(1) 
                << (100.0 * (pingCount * uePingContainer.GetN() - successfulPings) / 
                   (pingCount * uePingContainer.GetN())) << "%" << std::endl;
        outFile << "Average RTT: " << std::fixed << std::setprecision(3) << avgRtt << " ms" << std::endl;
        outFile << "Minimum RTT: " << std::fixed << std::setprecision(3) << minRtt << " ms" << std::endl;
        outFile << "Maximum RTT: " << std::fixed << std::setprecision(3) << maxRtt << " ms" << std::endl;
    }

    // Print throughput results if any throughput UEs exist
    if (ueThroughputContainer.GetN() > 0 && monitor != nullptr) {
        monitor->CheckForLostPackets();
        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
        FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

        outFile << std::endl << "=== Throughput Test Results ===" << std::endl;
        outFile << "Test Duration: " << (simTime - udpAppStartTime).GetSeconds() << " seconds" << std::endl;
        outFile << "Packet Size: " << throughputPacketSize << " bytes" << std::endl;
        outFile << "Target Data Rate: " << throughputDataRate / 1000000.0 << " Mbps" << std::endl;
        outFile << std::endl;

        double totalThroughput = 0.0;
        uint32_t flowCount = 0;
        double flowDuration = (simTime - udpAppStartTime).GetSeconds();

        for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
            
            if (i->second.rxPackets > 0) {
                double throughputMbps = i->second.rxBytes * 8.0 / flowDuration / 1000.0 / 1000.0;
                totalThroughput += throughputMbps;
                flowCount++;

                outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort 
                        << " -> " << t.destinationAddress << ":" << t.destinationPort << ")" << std::endl;
                outFile << "  Throughput: " << std::fixed << std::setprecision(3) << throughputMbps << " Mbps" << std::endl;
                outFile << "  Packets: " << i->second.txPackets << " sent, " << i->second.rxPackets << " received" << std::endl;
                outFile << "  Mean delay: " << std::fixed << std::setprecision(3) 
                        << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms" << std::endl;
                outFile << std::endl;
            }
        }

        if (flowCount > 0) {
            outFile << "=== Throughput Summary ===" << std::endl;
            outFile << "Total flows: " << flowCount << std::endl;
            outFile << "Average throughput per flow: " << std::fixed << std::setprecision(3) 
                    << totalThroughput / flowCount << " Mbps" << std::endl;
            outFile << "Total throughput: " << std::fixed << std::setprecision(3) << totalThroughput << " Mbps" << std::endl;
        }
    }

    outFile.close();

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();
    delete helper;
    return EXIT_SUCCESS;
}
