//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "inet/common/packet/chunk/SequenceChunk.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/physicallayer/common/packetlevel/BandListening.h"
#include "inet/physicallayer/common/packetlevel/ListeningDecision.h"
#include "Phy.h"

namespace lte {

Define_Module(LteTransmitter);
Define_Module(LteReceiver);
Define_Module(LteRadio);
Protocol lte("lte", "LTE");

const ITransmission *LteTransmitter::createTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime) const
{
    auto endTime = startTime + packet->getDuration();
    auto mobility = transmitter->getAntenna()->getMobility();
    auto startPosition = mobility->getCurrentPosition();
    auto endPosition = mobility->getCurrentPosition();
    auto startOrientation = mobility->getCurrentAngularPosition();
    auto endOrientation = mobility->getCurrentAngularPosition();
    return new LteTransmission(transmitter, packet, startTime, endTime, startPosition, endPosition, startOrientation, endOrientation);
}

const IListening *LteReceiver::createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition) const
{
    return new BandListening(radio, startTime, endTime, startPosition, endPosition, Hz(NaN), Hz(NaN));
}

const IListeningDecision *LteReceiver::computeListeningDecision(const IListening *listening, const IInterference *interference) const
{
    return new ListeningDecision(listening, true);
}

bool LteReceiver::computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const
{
    return true;
}

LteRadio::LteRadio()
{
    // NOTE: generate fake allocation, this should be done somewhere else
    for (int i = 0; i < 100; i++) {
        AllocatedResourceBlock allocatedResourceBlock;
        allocatedResourceBlock.subframeIndex = 0;
        allocatedResourceBlock.slotIndex = 0;
        allocatedResourceBlock.resourceBlockIndex = i;
        allocatedResourceBlocks.push_back(allocatedResourceBlock);
    }
}

void LteRadio::handleUpperPacket(Packet *packet)
{
    auto frame = new Frame();
    frame->setName("LteFrame");
    frame->addTag<PacketProtocolTag>()->setProtocol(&lte);
    insertPacketIntoFrame(*packet, b(0), packet->getTotalLength(), *frame, allocatedResourceBlocks);
    delete packet;
    computeFrameContent(*frame);
    Radio::handleUpperPacket(frame);
}

void LteRadio::sendUp(Packet *packet)
{
    auto frame = check_and_cast<Frame *>(packet);
    packet = extractPacketFromFrame(*frame, allocatedResourceBlocks);
    Radio::sendUp(packet);
    delete frame;
}

void LteRadio::insertPacketIntoFrame(const Packet& packet, b offset, b length, Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks)
{
    for (auto allocatedResourceBlock : allocatedResourceBlocks) {
        auto& subframe = frame.getSubframe(allocatedResourceBlock.subframeIndex);
        auto& slot = subframe.getSlot(allocatedResourceBlock.slotIndex);
        auto& resourceBlock = slot.getResourceBlock(allocatedResourceBlock.resourceBlockIndex);
        for (int i = 0; i < resourceBlock.getNumResourceElements(); i++) {
            auto& resourceElement = resourceBlock.getResourceElement(i);
            auto resourceElementLength = resourceElement.getLength();
            auto content = packet.peekAt(offset, resourceElementLength);
            resourceElement.setContent(content);
            offset += resourceElementLength;
            length -= resourceElementLength;
            if (length == b(0))
                return;
        }
    }
}

Packet *LteRadio::extractPacketFromFrame(Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks)
{
    auto packet = new Packet("LtePacket");
    for (auto allocatedResourceBlock : allocatedResourceBlocks) {
        auto& subframe = frame.getSubframe(allocatedResourceBlock.subframeIndex);
        auto& slot = subframe.getSlot(allocatedResourceBlock.slotIndex);
        auto& resourceBlock = slot.getResourceBlock(allocatedResourceBlock.resourceBlockIndex);
        for (int i = 0; i < resourceBlock.getNumResourceElements(); i++) {
            auto& resourceElement = resourceBlock.getResourceElement(i);
            const auto& content = resourceElement.getContent();
            if (content != nullptr)
                packet->insertAtBack(content);
        }
    }
    return packet;
}

void LteRadio::computeFrameContent(Frame& frame)
{
    auto frameContent = makeShared<SequenceChunk>();
    for (int i = 0; i < frame.getNumSubframes(); i++) {
        auto& subframe = frame.getSubframe(i);
        auto subframeContent = makeShared<SequenceChunk>();
        for (int j = 0; j < subframe.getNumSlots(); j++) {
            auto& slot = subframe.getSlot(j);
            auto slotContent = makeShared<SequenceChunk>();
            for (int k = 0; k < slot.getNumResourceBlocks(); k++) {
                auto& resourceBlock = slot.getResourceBlock(k);
                auto resourceBlockContent = makeShared<SequenceChunk>();
                for (int l = 0; l < resourceBlock.getNumResourceElements(); l++) {
                    auto& resourceElement = resourceBlock.getResourceElement(l);
                    const auto& content = resourceElement.getContent();
                    if (content != nullptr)
                        resourceBlockContent->insertAtBack(content);
                }
                resourceBlock.setContent(resourceBlockContent);
                slotContent->insertAtBack(resourceBlockContent);
            }
            slot.setContent(slotContent);
            subframeContent->insertAtBack(slotContent);
        }
        subframe.setContent(subframeContent);
        frameContent->insertAtBack(subframeContent);
    }
    frame.setContent(frameContent);
}

} // namespace lte

