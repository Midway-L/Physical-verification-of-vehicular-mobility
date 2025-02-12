//
// Copyright (C) 2023 Tobias Hardes <tobias.hardes@uni-paderborn.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins_carla/app/ExampleCarlaApplication.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"


using namespace veins_carla;
using namespace veins;

Define_Module(ExampleCarlaApplication);

void ExampleCarlaApplication::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        messageSendTimeVec.setName("messageSendTime");
        messageReceiveTimeVec.setName("messageReceiveTime");
        rssiVec.setName("rssi");
         // Get the CarlaMobility module
        carlaMobility = veins_carla::CarlaMobilityAccess().get(getParentModule());
        ASSERT(carlaMobility);
        // Initialize UDP socket
            socket_initialized = false;
            try {
                rssi_socket = socket(AF_INET, SOCK_DGRAM, 0);
                if (rssi_socket < 0) {
                    throw std::runtime_error("Socket creation failed");
                }
                
                python_addr.sin_family = AF_INET;
                python_addr.sin_port = htons(12345);
                python_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
                
                socket_initialized = true;
            }
            catch (const std::exception& e) {
                EV << "Socket initialization failed: " << e.what() << endl;
            }
    }
}

void ExampleCarlaApplication::finish()
{   
    if (socket_initialized) {
            close(rssi_socket);
        }
    DemoBaseApplLayer::finish();
}

void ExampleCarlaApplication::onBSM(DemoSafetyMessage* bsm)
{
    std::string myId = getParentModule()->getFullName();
    if (myId != "node[1]") {  //  node[1] is follow vehicle
        return;
    }
    // record receive time
    messageReceiveTimeVec.record(simTime());
    std::cout << "received at: " << simTime() << std::endl;
    std::cout << "Sender position: " << bsm->getSenderPos() << std::endl;
    // record RSSI value
    auto ctrlInfo = check_and_cast<PhyToMacControlInfo*>(bsm->getControlInfo());
    if (ctrlInfo) {
        auto deciderResult = check_and_cast<DeciderResult80211*>(ctrlInfo->getDeciderResult());
        if (deciderResult && socket_initialized) {
            double rssi = deciderResult->getRecvPower_dBm();
            rssiVec.record(rssi);
        
        try {
                    nlohmann::json data;
                    data["rssi"] = rssi;
                    data["messageSendTime"] = bsm->getTimestamp().dbl();
                    data["messageReceiveTime"] = simTime().dbl();
                    
                    std::string json_str = data.dump();
                    sendto(rssi_socket, json_str.c_str(), json_str.length(), 0,
                           (struct sockaddr*)&python_addr, sizeof(python_addr));
                }
                catch (const std::exception& e) {
                    EV << "Error sending RSSI data: " << e.what() << endl;
                }
        }
    }
}

void ExampleCarlaApplication::onWSM(BaseFrame1609_4* wsm)
{
    
}

void ExampleCarlaApplication::onWSA(DemoServiceAdvertisment* wsa)
{
   
}

void ExampleCarlaApplication::handleSelfMsg(cMessage* msg)
{

    DemoBaseApplLayer::handleSelfMsg(msg);
}


void ExampleCarlaApplication::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

void ExampleCarlaApplication::handleLowerMsg(omnetpp::cMessage* msg)
{
    DemoBaseApplLayer::handleLowerMsg(msg);
}

void ExampleCarlaApplication::onCarlaTick()
{
    Enter_Method_Silent("onCarlaTick()");
    // send message on every tick in Carla world
    sendMessage();
}

void ExampleCarlaApplication::sendMessage()
{
    // creat a new message
    auto bsm = new DemoSafetyMessage();
    populateWSM(bsm);
    bsm->setSenderPos(carlaMobility->getPositionAt(simTime()));
    // record send time
    messageSendTimeVec.record(simTime());
    std::cout << "send at: " << simTime() << std::endl;
    // send message
    sendDown(bsm);
}

