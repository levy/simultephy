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

#include "inet/common/packet/Packet.h"
#include "inet/physicallayer/base/packetlevel/ErrorModelBase.h"
#include "inet/physicallayer/base/packetlevel/ReceiverBase.h"
#include "inet/physicallayer/base/packetlevel/TransmissionBase.h"
#include "inet/physicallayer/base/packetlevel/TransmitterBase.h"
#include "inet/physicallayer/common/packetlevel/Radio.h"
#include "inet/physicallayer/modulation/Qam64Modulation.h"

namespace lte {

using namespace inet;
using namespace inet::physicallayer;

/**
 * Represents one OFDM symbol using one subcarrier.
 */
class ResourceElement {
  protected:
    const IApskModulation& modulation = Qam64Modulation::singleton; // QPSK, QAM-16, or QAM-64
    Ptr<const Chunk> contents; // this level of detail could be optional

  public:
    static int getNumSubcarriers() { return 1; }
    static int getNumSymbols() { return 1; }
    static Hz getBandwidth() { return kHz(15); }
    static simtime_t getDuration() { return (double)(2048 + 144) / 30720000; }
    b getLength() const { return b(modulation.getCodeWordSize()); }
    Ptr<const Chunk> getContent() const { return contents; }
    void setContent(Ptr<const Chunk> contents) { this->contents = contents; }
};

/**
 * Represents multiple OFDM symbols over multiple subcarriers.
 */
class ResourceBlock {
  protected:
    int numSymbols = 7; // either 7 or 6
    std::vector<ResourceElement> resourceElements; // this level of detail could be optional
    Ptr<const Chunk> contents;

  public:
    static int getNumSubcarriers() { return 12; }
    static Hz getBandwidth() { return ResourceElement::getBandwidth() * getNumSubcarriers(); }
    int getNumSymbols() const { return numSymbols; }
    int getNumResourceElements() const { return getNumSubcarriers() * getNumSymbols(); }
    ResourceElement& getResourceElement(int index) { return resourceElements[index]; }
    simtime_t getDuration() const { return getNumSymbols() * ResourceElement::getDuration() + (double)(160 - 144) / 30720000; }
    Ptr<const Chunk> getContent() const { return contents; }
    void setContent(Ptr<const Chunk> contents) { this->contents = contents; }
};

/**
 * Represents multiple resource blocks over all subcarriers.
 */
class Slot {
  protected:
    int numSubcarriers = 2048; // from 128 to 2048
    int numOccupiedSubcarriers = 1200; // from 76 to 1200
    std::vector<ResourceBlock> resourceBlocks;
    Ptr<const Chunk> contents; // this aggregate could be optional

  public:
    int getNumSubcarriers() const { return numSubcarriers; }
    int getNumOccupiedSubcarriers() const { return numOccupiedSubcarriers; }
    int getNumGuardSubcarriers() const { return numSubcarriers - numOccupiedSubcarriers; }
    int getNumResourceBlocks() const { return numOccupiedSubcarriers / ResourceBlock::getNumSubcarriers(); }
    ResourceBlock& getResourceBlock(int index) { return resourceBlocks[index]; }
    simtime_t getDuration() const { return 5E-4; }
    Ptr<const Chunk> getContent() const { return contents; }
    void setContent(Ptr<const Chunk> contents) { this->contents = contents; }
};

/**
 * Represents multiple slots.
 */
class Subframe : public Packet {
  protected:
    static constexpr int numSlots = 2;
    Slot slots[numSlots];
    Ptr<const Chunk> contents; // this aggregate could be optional

  public:
    static int getNumSlots() { return numSlots; }
    Slot& getSlot(int index) { return slots[index]; }
    simtime_t getDuration() const { return 1E-3; }
    Ptr<const Chunk> getContent() const { return contents; }
    void setContent(Ptr<const Chunk> contents) { this->contents = contents; }
};

/**
 * Represents multiple subframes.
 */
class Frame : public Packet {
  protected:
    static constexpr int numSubframes = 10;
    Subframe subframes[numSubframes];
    Ptr<const Chunk> contents; // this aggregate could be optional

  public:
    static int getNumSubframes() { return numSubframes; }
    Subframe& getSubframe(int index) { return subframes[index]; }
    simtime_t getDuration() const { return 10E-3; }
    Ptr<const Chunk> getContent() const { return contents; }
    void setContent(Ptr<const Chunk> contents) { this->contents = contents; }
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
class LteTransmission : public TransmissionBase {
  public:
    LteTransmission(const IRadio *transmitter, const Packet *packet, const simtime_t startTime, const simtime_t endTime, const Coord startPosition, const Coord endPosition, const EulerAngles startOrientation, const EulerAngles endOrientation) :
        TransmissionBase(transmitter, packet, startTime, endTime, 0, 0, startTime - endTime, startPosition, endPosition, startOrientation, endOrientation) {}
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
    /**
     * Inserts a part of the packet's contents into the resource blocks of the given frame according to the allocation.
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

    /**
     * Transmits a whole frame as a single wireless signal using the radio over the radio medium.
     */
    void transmitFrameSignal(Frame& frame);

    /**
     * Transmits one subframe as a single wireless signal using the radio over the radio medium.
     */
    void transmitSubframeSignal(Frame& frame, int subframeIndex);
};

} // namespace lte

#endif // __LTE_PHY_H_
