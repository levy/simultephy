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

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.inet.WirelessHost;
import inet.visualizer.integrated.IntegratedCanvasVisualizer;

network SimpleNetwork
{
    submodules:
        radioMedium: LteRadioMedium {
            parameters:
	            @display("p=100,100;is=s");
        }
        configurator: Ipv4NetworkConfigurator {
            parameters:
			    @display("p=100,200;is=s");
        }
        visualizer: IntegratedCanvasVisualizer {
            parameters:
	            @display("p=100,300;is=s");
        }
        eNodeB: WirelessHost {
            parameters:
	            @display("p=250,200;i=device/antennatower");
        }
        ue: WirelessHost {
            parameters:
	            @display("p=650,200;i=device/cellphone");
        }
}
