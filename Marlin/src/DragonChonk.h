/**
 * DragonChonk Laser Firmware
 * Copyright (c) 2025 Geoff Van Valkenberg [https://github.com/RoboDad/DragonChonk]
 *
 * Based on Marlin Firmware.
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "inc/MarlinConfig.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PVVFOO MYSERIAL1.println

#define DRAGON_CHONK_LOG_EVERYTHING

namespace DragonChonk
{
static constexpr unsigned int MAX_LENSES = 10;
static constexpr unsigned int MAX_LENS_NAME_LENGTH = 32;

struct IndicatorLight
{
	enum class State
	{
		OFF,
		ON,
		PULSE,
	};

	State   m_iState = State::OFF;
	uint8_t m_iPin = 13;
	float   m_cycleMilliTime = 2000.0f; // PULSE cycle time.

	IndicatorLight()
	{
	}

	void Init(const unsigned char PIN)
	{
		m_iPin = PIN;
	}

	void Process(const long milliTime)
	{
		float fi;
		const float t = modf(float(milliTime) / m_cycleMilliTime, &fi);

		int iLevel256 = 0;
		switch ( m_iState )
		{
			default:
			case State::OFF:
				iLevel256 = 0;
				break;

			case State::ON:
				iLevel256 = 255;
				break;

			case State::PULSE:
			{
				iLevel256 = int(255.0f * ( 0.5f + 0.5f * sinf(2.0f * 3.14159265f * t) ));
				iLevel256 = max(iLevel256, 0);
			}
		}

		//analogWriteResolution(12); // Doesn't seem to do anything on Grand Central
		analogWrite(m_iPin, iLevel256);
		//LOG("PVV: iButtonLedLevel=%d\n", iButtonLedLevel);
	}

	IndicatorLight& SetState(const State iState)
	{
		m_iState = iState;
		return *this;
	}

	IndicatorLight& SetCycleTime(const float seconds)
	{
		m_cycleMilliTime = seconds * 1000.0f;
		return *this;
	}
};

static const long kDebounceDelay = 50;    // the debounce time; increase if the output flickers

class Button
{
	public:
	Button() : _PIN(0), _buttonState(HIGH), _lastButtonState(HIGH), _stateChanged(0), _lastDebounceTime(0)
	{
	}
	void Init(const unsigned char PIN);
	void Process(const long milliTime);
	bool HasStateChanged() const
	{
		return _stateChanged != 0;
	}
	bool IsPressed() const
	{
		return _buttonState == LOW;
	}
	bool ButtonDown() const
	{
		return HasStateChanged() && IsPressed();
	}

	unsigned char   _PIN;
	unsigned char   _buttonState;      // the current reading from the input pin
	unsigned char   _lastButtonState;  // the previous reading from the input pin
	unsigned char   _stateChanged;
	long            _lastDebounceTime; // the last time the output pin was toggled
};

struct Settings
{
	// Index of the currently installed lens...
	uint16_t                      m_indexInstalledLens = 0;
	MString<MAX_LENS_NAME_LENGTH> m_lensNames[DragonChonk::MAX_LENSES];
	float                         m_lensFocalZ[DragonChonk::MAX_LENSES];

	void SetIndexInstalledLens(const int indexInstalledLens);

	static int s_eeprom_base_address;
};

} // namespace DragonChonk



extern void SetNeoPixel(const uint8_t r, const uint8_t g, const uint8_t b);
extern void PVV_FOREVER();
extern void DragonChonk_setup();
extern void DragonChonk_Process();
extern void DragonChonk_menu_laser();
extern DragonChonk::Settings& DragonChonk_get_settings();
extern void DragonChonk_save_settings();
extern void DragonChonk_reset_settings();
extern void DragonChonk_report_settings(const bool forReplay = true);
extern void DragonChonk_PostHoming();
