
//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  


//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mti7_mti8device.h"
#include <xstypes/xsstatusflag.h>
#include <xstypes/xsvector.h>

MTi7_MTi8Device::MTi7_MTi8Device(Communicator* comm)
	: MtiBaseDeviceEx(comm)
{
	if (comm)
		comm->setDefaultTimeout(2000); //Increase the default timeout for MTi-1 devices because a settings write can occasionally take ~900ms
}

MTi7_MTi8Device::MTi7_MTi8Device(XsDevice* masterdevice)
	: MtiBaseDeviceEx(masterdevice)
{
}

MTi7_MTi8Device::~MTi7_MTi8Device()
{
}

namespace
{
//! \brief Returns the default frequency of the supplied \a dataType
int baseFreq(MTi7_MTi8Device const* device, XsDataIdentifier dataType)
{
	switch (dataType & XDI_TypeMask)
	{
		case XDI_None:
			return 100;
		case XDI_TimestampGroup:
			return XDI_MAX_FREQUENCY_VAL;
		case XDI_StatusGroup:
			return 100;
		case XDI_TemperatureGroup:
			return 100;
		case XDI_PositionGroup:
			return 100;
		case XDI_VelocityGroup:
			return 100;
		case XDI_OrientationGroup:
			return 100;
		case XDI_AccelerationGroup:
			return 100;
		case XDI_AngularVelocityGroup:
			return 100;
		case XDI_MagneticGroup:
			return 100;
		case XDI_PressureGroup:
			return 50;
		case XDI_GnssGroup:
		{
			XsDataIdentifier fullType = (dataType & XDI_FullTypeMask);
			if (fullType == XDI_GnssPvtPulse)
				return device->deviceId().isRtk() ? XDI_MAX_FREQUENCY_VAL : 0;
			if (fullType == XDI_GnssGroup || fullType == XDI_GnssPvtData)
				return XDI_MAX_FREQUENCY_VAL;
			return 0;
		}
		default:
			return 0;
	}
}
}

/*! \brief Returns the base update rate (hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult MTi7_MTi8Device::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType & XDI_FullTypeMask) == XDI_LocationId || (dataType & XDI_FullTypeMask) == XDI_DeviceId)
		return result;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR || (dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		bool isMtMk4_1_v1 = hardwareVersion().major() == 1;
		bool isMtMk4_1_v2 = hardwareVersion().major() == 2;
		result.m_frequency = isMtMk4_1_v2 ? 800 : 1000;
		result.m_divedable = isMtMk4_1_v1 ? false : true;

		return result;
	}

	result.m_frequency = baseFreq(this, dataType);

	if (((dataType & XDI_TypeMask) == XDI_TimestampGroup) || ((dataType & XDI_TypeMask) == XDI_GnssGroup))
		result.m_divedable = false;

	return result;
}

bool MTi7_MTi8Device::hasIccSupport() const
{
	return true;
}

uint32_t MTi7_MTi8Device::supportedStatusFlags() const
{
	return (uint32_t)(
			//|XSF_SelfTestOk
			XSF_OrientationValid
			| XSF_GpsValid
			| XSF_NoRotationMask
			| XSF_RepresentativeMotion
			| XSF_ExternalClockSynced
			| XSF_ClipAccX
			| XSF_ClipAccY
			| XSF_ClipAccZ
			| XSF_ClipGyrX
			| XSF_ClipGyrY
			| XSF_ClipGyrZ
			| XSF_ClipMagX
			| XSF_ClipMagY
			| XSF_ClipMagZ
			//|XSF_Retransmitted
			| XSF_ClippingDetected
			//|XSF_Interpolated
			//|XSF_SyncIn
			//|XSF_SyncOut
			| XSF_FilterMode
			| XSF_HaveGnssTimePulse
			| (deviceId().isRtk() ? XSF_RtkStatus : 0)
		);
}

bool MTi7_MTi8Device::setStringOutputMode(uint16_t /*type*/, uint16_t /*period*/, uint16_t /*skipFactor*/)
{
	return true;
}

/*! \copybrief XsDevice::shortProductCode
*/
XsString MTi7_MTi8Device::shortProductCode() const
{
	XsString code = productCode();

	if (hardwareVersion() >= XsVersion(2, 0, 0))
		code = stripProductCode(deviceId());

	return code;
}

/*! \copydoc XsDevice::setGnssLeverArm
*/
bool MTi7_MTi8Device::setGnssLeverArm(const XsVector& arm)
{
	if (!deviceId().isRtk())
		return false;

	XsMessage snd(XMID_SetGnssLeverArm, 3 * sizeof(float));
	snd.setBusId(busId());
	snd.setDataFloat((float)arm[0], 0/* sizeof(float)*/);
	snd.setDataFloat((float)arm[1], 1 * sizeof(float));
	snd.setDataFloat((float)arm[2], 2 * sizeof(float));

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	return true;
}

/*! \copydoc XsDevice::gnssLeverArm
*/
XsVector MTi7_MTi8Device::gnssLeverArm() const
{
	if (!deviceId().isRtk())
		return XsVector();

	XsMessage snd(XMID_ReqGnssLeverArm), rcv;
	if (!doTransaction(snd, rcv))
		return XsVector();

	XsVector arm(3);
	arm[0] = rcv.getDataFloat(0/* sizeof(float)*/);
	arm[1] = rcv.getDataFloat(1 * sizeof(float));
	arm[2] = rcv.getDataFloat(2 * sizeof(float));
	return arm;
}
