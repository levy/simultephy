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

/**
 * Represents one OFDM symbol using one subcarrier.
 */
class ResourceElement {
  protected:
    // const IApskModulation &modulation = Qam64Modulation::singleton; // QPSK, QAM-16, or QAM-64
    Ptr<const Chunk> content; // this level of detail could be optional

  public:
    ResourceElement() {}
    ResourceElement(const ResourceElement& other) { /*modulation = other.modulation;*/ content = other.content; }
    void operator=(const ResourceElement& other) { content = other.content; }

    static int getNumSubcarriers() { return 1; }
    static int getNumSymbols() { return 1; }
    static Hz getBandwidth() { return kHz(15); }
    static simtime_t getDuration() { return (double)(2048 + 144) / 30720000; }

    b getLength() const { return B(1); } // TODO: b(modulation->getCodeWordSize()); }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple OFDM symbols over multiple subcarriers.
 */
class ResourceBlock {
  protected:
    int numSymbols = 7; // either 7 or 6
    std::vector<ResourceElement> resourceElements; // this level of detail could be optional
    Ptr<const Chunk> content;

  public:
    ResourceBlock() { resourceElements.resize(getNumResourceElements()); }
    ResourceBlock(const ResourceBlock& other) { resourceElements = other.resourceElements; }
    void operator=(const ResourceBlock& other) { resourceElements = other.resourceElements; }

    static int getNumSubcarriers() { return 12; }
    static Hz getBandwidth() { return ResourceElement::getBandwidth() * getNumSubcarriers(); }

    int getNumSymbols() const { return numSymbols; }
    int getNumResourceElements() const { return getNumSubcarriers() * getNumSymbols(); }
    ResourceElement& getResourceElement(int index) { return resourceElements[index]; }
    simtime_t getDuration() const { return getNumSymbols() * ResourceElement::getDuration() + (double)(160 - 144) / 30720000; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple resource blocks over all subcarriers.
 */
class Slot {
  protected:
    int numSubcarriers = 2048; // from 128 to 2048
    int numOccupiedSubcarriers = 1200; // from 76 to 1200
    std::vector<ResourceBlock> resourceBlocks;
    Ptr<const Chunk> content; // this aggregate could be optional

  public:
    Slot() { resourceBlocks.resize(getNumResourceBlocks()); }
    Slot(const Slot& other) { resourceBlocks = other.resourceBlocks; }
    void operator=(const Slot& other) { resourceBlocks = other.resourceBlocks; }

    int getNumSubcarriers() const { return numSubcarriers; }
    int getNumOccupiedSubcarriers() const { return numOccupiedSubcarriers; }
    int getNumGuardSubcarriers() const { return numSubcarriers - numOccupiedSubcarriers; }
    int getNumResourceBlocks() const { return numOccupiedSubcarriers / ResourceBlock::getNumSubcarriers(); }
    ResourceBlock& getResourceBlock(int index) { return resourceBlocks[index]; }
    simtime_t getDuration() const { return 5E-4; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple slots.
 */
class Subframe {
  protected:
    static constexpr int numSlots = 2;
    Slot slots[numSlots];
    Ptr<const Chunk> content; // this aggregate could be optional

  public:
    Subframe() {}
    Subframe(const Subframe& other) { for (int i = 0; i < numSlots; i++) slots[i] = other.slots[i]; }
    void operator=(const Subframe& other) { for (int i = 0; i < numSlots; i++) slots[i] = other.slots[i]; }

    static int getNumSlots() { return numSlots; }

    Slot& getSlot(int index) { return slots[index]; }
    simtime_t getDuration() const { return 1E-3; }
    Ptr<const Chunk> getContent() const { return content; }
    void setContent(Ptr<const Chunk> content) { this->content = content; }
};

/**
 * Represents multiple subframes.
 */
class Frame : public Packet {
  protected:
    static constexpr int numSubframes = 10;
    Subframe subframes[numSubframes];

  public:
    Frame() { setDuration(10E-3); }
    Frame(const Frame& other) : Packet(other) { for (int i = 0; i < numSubframes; i++) subframes[i] = other.subframes[i]; }
    Frame(const char *name, const Ptr<const Chunk>& content) : Packet(name, content) { setDuration(10E-3); }
    void operator=(const Frame& other) { Packet::operator=(other); for (int i = 0; i < numSubframes; i++) subframes[i] = other.subframes[i]; }

    virtual Frame *dup() const override { return new Frame(*this); }

    static int getNumSubframes() { return numSubframes; }

    Subframe& getSubframe(int index) { return subframes[index]; }
    void setContent(Ptr<const Chunk> content) { removeAll(); insertAtBack(content); }
};

/**
 * Represents an allocated resource block within a frame.
 */
class AllocatedResourceBlock {
  public:
    int subframeIndex;
    int slotIndex;
    int resourceBlockIndex;
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
