DynamixelTools
========
#####Matlab Toolbox to control ROBOTIS Dynamixel smart servo actuators with the OpenCM9.04 microcontroller.#####
######Version 1.0, 2-25-16######
#####Download Repository: [ZIP Archive](https://github.com/horchler/DynamixelTools/archive/master.zip)#####

--------

DynamixelTools consistes of two Matlab classes, [```OPENCM```](https://github.com/horchler/DynamixelTools/blob/master/OPENCM.m) and [```DXL```](https://github.com/horchler/DynamixelTools/blob/master/DXL.m), that allow for control of [ROBOTIS Dynamixel smart servo actuators](http://www.robotis.com/xe/dynamixel_en) with the [OpenCM9.04 microcontroller](http://support.robotis.com/en/product/auxdevice/controller/opencm9.04.htm). These classes have vectorized methods for interacting with multiple Dynamixel actuators simultaneously. Commands can be sent at approximately 200 Hz (limited by serial speed and your computer hardware and OS). See [demos](https://github.com/horchler/DynamixelTools/tree/master/demos) and included help for usage details.

DynamixelTools requires that a simple &#8220;tosser&#8221; code be running on the OpenCM9.04. A compiled binary of this is included. The ```DXL``` class (or the ```OPENCM``` superclass) will attempt to upload and install the tosser. See my [DynamixelQ](https://github.com/horchler/DynamixelQ) project, a C++ library to control Dynamixel actuators, for implementation details.
&nbsp;  

--------

Andrew D. Horchler, *horchler @ gmail . com*, [biorobots.case.edu](http://biorobots.case.edu/)  
Created: 6-28-15, Revision: 1.0, 2-25-16  

This version tested with Matlab 8.6.0.267246 (R2015b)  
Mac OS X 10.11.4 (Build: 15E65), Java 1.7.0_75-b13  
Compatibility maintained back through Matlab 8.5 (R2015a)  
&nbsp;  

--------

Acknowledgment of support: This material is based upon work supported by the [National Science Foundation](http://www.nsf.gov/) under [Grant No.&nbsp;1065489](http://www.nsf.gov/awardsearch/showAward.do?AwardNumber=1065489). Disclaimer: Any opinions, findings, and conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the National Science Foundation.  
&nbsp;  

Copyright &copy; 2015&ndash;2017, Andrew D. Horchler  
All rights reserved.  

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Case Western Reserve University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANDREW D. HORCHLER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.