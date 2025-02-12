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
    }
}

void ExampleCarlaApplication::finish()
{
    DemoBaseApplLayer::finish();
}

void ExampleCarlaApplication::onBSM(DemoSafetyMessage* bsm)
{
    // record receive time
    messageReceiveTimeVec.record(simTime());
    std::cout << "received at: " << simTime() << std::endl;
    std::cout << "Sender position: " << bsm->getSenderPos() << std::endl;
    // record RSSI value
    auto ctrlInfo = check_and_cast<PhyToMacControlInfo*>(bsm->getControlInfo());
    if (ctrlInfo) {
        auto deciderResult = check_and_cast<DeciderResult80211*>(ctrlInfo->getDeciderResult());
        if (deciderResult) {
            double rssi = deciderResult->getRecvPower_dBm();
            rssiVec.record(rssi);
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

