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
#include "Phy.h"

namespace lte {

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

void LteRadio::insertPacketIntoFrame(const Packet& packet, b offset, b length, Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks)
{
    for (auto allocatedResourceBlock : allocatedResourceBlocks) {
        auto subframe = frame.getSubframe(allocatedResourceBlock.subframeIndex);
        auto slot = subframe.getSlot(allocatedResourceBlock.slotIndex);
        auto resourceBlock = slot.getResourceBlock(allocatedResourceBlock.resourceBlockIndex);
        for (int i = 0; i < resourceBlock.getNumResourceElements(); i++) {
            auto resourceElement = resourceBlock.getResourceElement(i);
            auto resourceElementLength = resourceElement.getLength();
            auto contents = packet.peekAt(offset, resourceElementLength);
            resourceElement.setContent(contents);
            offset += resourceElementLength;
            length -= resourceElementLength;
            if (length == b(0))
                return;
        }
    }
}

Packet *LteRadio::extractPacketFromFrame(Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks)
{
    auto packet = new Packet("LTE");
    for (auto allocatedResourceBlock : allocatedResourceBlocks) {
        auto subframe = frame.getSubframe(allocatedResourceBlock.subframeIndex);
        auto slot = subframe.getSlot(allocatedResourceBlock.slotIndex);
        auto resourceBlock = slot.getResourceBlock(allocatedResourceBlock.resourceBlockIndex);
        for (int i = 0; i < resourceBlock.getNumResourceElements(); i++) {
            auto resourceElement = resourceBlock.getResourceElement(i);
            packet->insertAtBack(resourceElement.getContent());
        }
    }
    return packet;
}

void LteRadio::computeFrameContent(Frame& frame)
{
    auto frameContent = makeShared<SequenceChunk>();
    for (int i = 0; i < frame.getNumSubframes(); i++) {
        auto subframe = frame.getSubframe(i);
        auto subframeContent = makeShared<SequenceChunk>();
        for (int j = 0; j < subframe.getNumSlots(); j++) {
            auto slot = subframe.getSlot(j);
            auto slotContent = makeShared<SequenceChunk>();
            for (int k = 0; k < slot.getNumResourceBlocks(); k++) {
                auto resourceBlock = slot.getResourceBlock(k);
                auto resourceBlockContent = makeShared<SequenceChunk>();
                for (int l = 0; l < resourceBlock.getNumResourceElements(); l++) {
                    auto resourceElement = resourceBlock.getResourceElement(l);
                    resourceBlockContent->insertAtBack(resourceElement.getContent());
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

void LteRadio::transmitFrameSignal(Frame& frame)
{
    startTransmission(&frame, IRadioSignal::SIGNAL_PART_WHOLE);
}

void LteRadio::transmitSubframeSignal(Frame& frame, int subframeIndex)
{
    auto subframe = frame.getSubframe(subframeIndex);
    startTransmission(&subframe, IRadioSignal::SIGNAL_PART_WHOLE);
}

} // namespace lte

