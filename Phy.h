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

#ifndef __LTE_PHY_H_
#define __LTE_PHY_H_

#include "inet/common/packet/chunk/EmptyChunk.h"
#include "inet/common/packet/Packet.h"
#include "inet/physicallayer/base/packetlevel/ErrorModelBase.h"
#include "inet/physicallayer/base/packetlevel/NarrowbandTransmissionBase.h"
#include "inet/physicallayer/base/packetlevel/ReceiverBase.h"
#include "inet/physicallayer/base/packetlevel/TransmitterBase.h"
#include "inet/physicallayer/common/packetlevel/Radio.h"
#include "inet/physicallayer/contract/packetlevel/IRadioSignal.h"
#include "inet/physicallayer/modulation/Qam64Modulation.h"

namespace lte {

using namespace inet;
using namespace inet::physicallayer;

enum class LevelOfDetail {
//    FRAME,
//    SUBFRAME,
//    SLOT,
    RESOURCE_BLOCK,
    RESOURCE_ELEMENT
};

class LteMode {
  protected:
    const int numSubframesPerFrame = 10;
    const int numSlotsPerSubframe = 2;
    const int numSubcarriersPerSlot = 2048; // from 128 to 2048
    const int numOccupiedSubcarriersPerSlot = 1200; // from 76 to 1200
    const int numSymbolsPerResourceBlock = 7; // either 7 or 6
    const int numSubcarriersPerResourceBlock = 12;
    const IApskModulation &subcarrierModulation = Qam64Modulation::singleton; // QPSK, QAM-16, or QAM-64

  public:
    int getNumSubframesPerFrame() const { return numSubframesPerFrame; }
    int getNumSlotsPerSubframe() const { return numSlotsPerSubframe; }
    int getNumSubcarriersPerSlot() const { return numSubcarriersPerSlot; }
    int getNumOccupiedSubcarriersPerSlot() const { return numOccupiedSubcarriersPerSlot; }
    int getNumSymbolsPerResourceBlock() const { return numSymbolsPerResourceBlock; }
    int getNumSubcarriersPerResourceBlock() const { return numSubcarriersPerResourceBlock; }
    int getNumResourceBlocksPerSlot() const { return getNumOccupiedSubcarriersPerSlot() / getNumSubcarriersPerResourceBlock(); }
    int getNumResourceElementsPerResourceBlock() const { return getNumSubcarriersPerResourceBlock() * getNumSymbolsPerResourceBlock(); }

    b getFrameLength() const { return getSubframeLength() * getNumSubframesPerFrame(); }
    b getSubframeLength() const { return getSlotLength() * getNumSlotsPerSubframe(); }
    b getSlotLength() const { return getResourceBlockLength() * getNumResourceBlocksPerSlot(); }
    b getResourceBlockLength() const { return getResourceElementLength() * getNumResourceElementsPerResourceBlock(); }
    b getResourceElementLength() const { return B(1); } // return b(subcarrierModulation.getCodeWordSize()); }

    simtime_t getResourceElementDuration() const { return (double)(2048 + 144) / 30720000; }
    simtime_t getResourceBlockDuration() const { return getNumSymbolsPerResourceBlock() * getResourceElementDuration() + (double)(160 - 144) / 30720000; }
    simtime_t getSlotDuration() const { return 5E-4; }
    simtime_t getSubframeDuration() const { return 1E-3; }
    simtime_t getFrameDuration() const { return 10E-3; }
};

/**
 * Represents one OFDM symbol using one subcarrier.
 */
class ResourceElement {
  protected:
    Ptr<const Chunk> content; // this level of detail is optional

  public:
    const Chunk *getContentPtr() { return content.get(); } // only for class descriptor

  public:
    ResourceElement(LevelOfDetail levelOfDetail) { ASSERT(levelOfDetail == LevelOfDetail::RESOURCE_ELEMENT); }
    ResourceElement(const ResourceElement& other) { content = other.content; }
    void operator=(const ResourceElement& other) { content = other.content; }

    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple OFDM symbols over multiple subcarriers.
 */
class ResourceBlock {
  protected:
    std::vector<ResourceElement> resourceElements; // this level of detail is optional
    Ptr<const Chunk> content;

  public:
    const Chunk *getContentPtr() { return content.get(); } // only for class descriptor
    int getResourceElementsArraySize() const { return resourceElements.size(); } // only for class descriptor

  public:
    ResourceBlock(LteMode& mode, LevelOfDetail levelOfDetail) { if (levelOfDetail > LevelOfDetail::RESOURCE_BLOCK) for (int i = 0; i < mode.getNumResourceElementsPerResourceBlock(); i++) resourceElements.push_back(ResourceElement(levelOfDetail)); }
    ResourceBlock(const ResourceBlock& other) { content = other.content; resourceElements = other.resourceElements; }
    void operator=(const ResourceBlock& other) { content = other.content; resourceElements = other.resourceElements; }

    ResourceElement& getResourceElement(int index) { return resourceElements[index]; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple resource blocks over all subcarriers.
 */
class Slot {
  protected:
    std::vector<ResourceBlock> resourceBlocks;
    Ptr<const Chunk> content; // this aggregate is optional

  public:
    const Chunk *getContentPtr() { return content.get(); } // only for class descriptor
    int getResourceBlocksArraySize() const { return resourceBlocks.size(); } // only for class descriptor

  public:
    Slot(LteMode& mode, LevelOfDetail levelOfDetail) { for (int i = 0; i < mode.getNumResourceBlocksPerSlot(); i++) resourceBlocks.push_back(ResourceBlock(mode, levelOfDetail)); }
    Slot(const Slot& other) { content = other.content; resourceBlocks = other.resourceBlocks; }
    void operator=(const Slot& other) { content = other.content; resourceBlocks = other.resourceBlocks; }

    ResourceBlock& getResourceBlock(int index) { return resourceBlocks[index]; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple slots.
 */
class Subframe {
  protected:
    std::vector<Slot> slots;
    Ptr<const Chunk> content; // this aggregate is optional

  public:
    const Chunk *getContentPtr() { return content.get(); } // only for class descriptor
    int getSlotsArraySize() const { return slots.size(); } // only for class descriptor

  public:
    Subframe(LteMode& mode, LevelOfDetail levelOfDetail) { for (int i = 0; i < mode.getNumSlotsPerSubframe(); i++) slots.push_back(Slot(mode, levelOfDetail)); }
    Subframe(const Subframe& other) { content = other.content; slots = other.slots; }
    void operator=(const Subframe& other) { content = other.content; slots = other.slots; }

    Slot& getSlot(int index) { return slots[index]; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple subframes.
 */
class Frame : public Packet {
  protected:
    std::vector<Subframe> subframes;

  public:
    int getSubframesArraySize() const { return subframes.size(); } // only for class descriptor

  public:
    Frame(LteMode& mode, LevelOfDetail levelOfDetail) { for (int i = 0; i < mode.getNumSubframesPerFrame(); i++) subframes.push_back(Subframe(mode, levelOfDetail)); setDuration(10E-3); }
    Frame(const Frame& other) : Packet(other) { subframes = other.subframes; }
    Frame(const char *name, const Ptr<const Chunk>& content) : Packet(name, content) { setDuration(10E-3); }
    void operator=(const Frame& other) { Packet::operator=(other); subframes = other.subframes; }

    virtual Frame *dup() const override { return new Frame(*this); }

    Subframe& getSubframe(int index) { return subframes[index]; }
    Ptr<const Chunk> getContent() const { return peekAll(); }
    void setContent(Ptr<const Chunk> content) { removeAll(); insertAtBack(content); }
};

/**
 * Represents an allocated resource block within a frame.
 */
class AllocatedResourceBlock {
  public:
    int subframeIndex = -1;
    int slotIndex = -1;
    int resourceBlockIndex = -1;
};

/**
 * Implements the INET transmission.
 */
class LteTransmission : public NarrowbandTransmissionBase, public IScalarSignal {
  protected:
    W power;

  public:
    LteTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition, const EulerAngles startOrientation, const EulerAngles endOrientation) :
        NarrowbandTransmissionBase(transmitter, packet, startTime, endTime, 0, 0, startTime - endTime, startPosition, endPosition, startOrientation, endOrientation, nullptr, Hz(NaN), Hz(NaN)) {}

    virtual W computeMinPower(simtime_t startTime, simtime_t endTime) const { return power; }
    virtual W getPower() const override { return power; }
};

/**
 * Implements the INET transmitter.
 */
class LteTransmitter : public TransmitterBase {
  public:
    virtual const ITransmission *createTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime) const override;
};

/**
 * Implements the INET receiver.
 */
class LteReceiver : public ReceiverBase {
  public:
    virtual const IListening *createListening(const IRadio *radio, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition) const override;
    virtual const IListeningDecision *computeListeningDecision(const IListening *listening, const IInterference *interference) const override;
    virtual bool computeIsReceptionSuccessful(const IListening *listening, const IReception *reception, IRadioSignal::SignalPart part, const IInterference *interference, const ISnir *snir) const override;
};

/**
 * Implements the INET error model.
 */
class LteErrorModel : public ErrorModelBase {
    virtual double computePacketErrorRate(const ISnir *snir, IRadioSignal::SignalPart part) const override { return NaN; }
    virtual double computeBitErrorRate(const ISnir *snir, IRadioSignal::SignalPart part) const override { return NaN; }
    virtual double computeSymbolErrorRate(const ISnir *snir, IRadioSignal::SignalPart part) const override { return NaN; }
};

/**
 * Implements the INET radio.
 */
class LteRadio : public Radio {
  protected:
    LteMode mode;
    LevelOfDetail levelOfDetail = LevelOfDetail::RESOURCE_BLOCK;
    std::vector<AllocatedResourceBlock> allocatedResourceBlocks;

  public:
    LteRadio();

  protected:
    virtual void handleUpperPacket(Packet *packet) override;
    virtual void sendUp(Packet *packet) override;

    /**
     * Inserts a part of the packet's content into the resource blocks of the given frame according to the allocation.
     */
    void insertPacketIntoFrame(const Packet& packet, b offset, b length, Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks);

    /**
     * Extracts a packet from the resource blocks of the given frame according to the allocation.
     */
    Packet *extractPacketFromFrame(Frame& frame, const std::vector<AllocatedResourceBlock>& allocatedResourceBlocks);

    /**
     * Computes all frame, subframe, slot, and resource block contents based on resource element contents.
     */
    void computeFrameContent(Frame& frame);
};

} // namespace lte

#endif // __LTE_PHY_H_
