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

@namespace(lte);

import inet.physicallayer.common.packetlevel.Radio;
import inet.physicallayer.common.packetlevel.RadioMedium;
import inet.physicallayer.contract.packetlevel.IReceiver;
import inet.physicallayer.contract.packetlevel.ITransmitter;

module LteTransmitter like ITransmitter
{
    parameters:
        @display("i=block/wtx");
        @class(LteTransmitter);
}

module LteReceiver like IReceiver
{
    parameters:
        @display("i=block/wrx");
        @class(LteReceiver);
}

module LteRadio extends Radio
{
    parameters:
        antenna.typename = default("IsotropicAntenna");
        transmitter.typename = default("LteTransmitter");
        receiver.typename = default("LteReceiver");
        @class(LteRadio);
}

module LteRadioMedium extends RadioMedium
{
    parameters:
        propagation.typename = default("ConstantSpeedPropagation");
        pathLoss.typename = default("FreeSpacePathLoss");
        analogModel.typename = default("ScalarAnalogModel");
        backgroundNoise.typename = default("IsotropicScalarBackgroundNoise");
}
