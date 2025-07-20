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

/**
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/grbl/grbl
 */

#include "DragonChonk.h"
//#include "C:\Users\phgev\Documents\Make\DragonChonk\Marlin\Configuration_adv.h"

#include "HAL/shared/Delay.h"
#include "HAL/shared/esp_wifi.h"
#include "HAL/shared/cpu_exception/exception_hook.h"

#if ENABLED(WIFISUPPORT)
  #include "HAL/shared/esp_wifi.h"
#endif

#ifdef ARDUINO
  #include <pins_arduino.h>
#endif
#include <math.h>
#include <string>
#include <vector>

#include <Adafruit_NeoPixel.h>

#include "module/endstops.h"
#include "module/motion.h"
#include "module/planner.h"
#include "module/printcounter.h" // PrintCounter or Stopwatch
#include "module/settings.h"
#include "module/stepper.h"

#include "gcode/gcode.h"
#include "gcode/parser.h"
#include "gcode/queue.h"

#include "feature/pause.h"
#include "sd/cardreader.h"

#include "lcd/marlinui.h"
#include "lcd/menu/menu_item.h"
#if HAS_TOUCH_BUTTONS
  #include "lcd/touch/touch_buttons.h"
#endif


#if HAS_ETHERNET
  #include "feature/ethernet.h"
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #include "libs/BL24CXX.h"
#endif

#if ENABLED(DIRECT_STEPPING)
  #include "feature/direct_stepping.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "feature/host_actions.h"
#endif

#if HAS_BEEPER
  #include "libs/buzzer.h"
#endif

#if ENABLED(EXTERNAL_CLOSED_LOOP_CONTROLLER)
  #include "feature/closedloop.h"
#endif

#if HAS_MOTOR_CURRENT_I2C
  #include "feature/digipot/digipot.h"
#endif

#if HAS_COLOR_LEDS
  #include "feature/leds/leds.h"
#endif

#if ENABLED(POLL_JOG)
  #include "feature/joystick.h"
#endif

#if HAS_SERVOS
  #include "module/servo.h"
#endif

#if HAS_MOTOR_CURRENT_DAC
  #include "feature/dac/stepper_dac.h"
#endif

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "feature/twibus.h"
#endif

#if ENABLED(I2C_POSITION_ENCODERS)
  #include "feature/encoder_i2c.h"
#endif

#if (HAS_TRINAMIC_CONFIG || HAS_TMC_SPI) && DISABLED(PSU_DEFAULT_OFF)
  #include "feature/tmc_util.h"
#endif

#if HAS_CUTTER
  #include "feature/spindle_laser.h"
#endif

#if HAS_MEDIA
  CardReader card;
#endif

#if ENABLED(GCODE_REPEAT_MARKERS)
  #include "feature/repeat.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "feature/powerloss.h"
#endif

#if ENABLED(CANCEL_OBJECTS)
  #include "feature/cancel_object.h"
#endif

#if ANY(PROBE_TARE, HAS_Z_SERVO_PROBE)
  #include "module/probe.h"
#endif


#if ENABLED(TEMP_STAT_LEDS)
  #include "feature/leds/tempstat.h"
#endif

#if ENABLED(CASE_LIGHT_ENABLE)
  #include "feature/caselight.h"
#endif

#if HAS_FANMUX
  #include "feature/fanmux.h"
#endif

#if HAS_TOOLCHANGE
  #include "module/tool_change.h"
#endif

#if HAS_FANCHECK
  #include "feature/fancheck.h"
#endif

#if ENABLED(USE_CONTROLLER_FAN)
  #include "feature/controllerfan.h"
#endif

#if HAS_DRIVER_SAFE_POWER_PROTECT
  #include "feature/stepper_driver_safety.h"
#endif

#if ENABLED(PSU_CONTROL)
  #include "feature/power.h"
#endif

#if HAS_RS485_SERIAL
  #include "feature/rs485.h"
#endif

// =====================================================================================================================

// TODO: Support localized strings via language.h
#define STR_DRAGON_CHONK_SELECTED_LENS                         "Lens #: "


// =====================================================================================================================

#define NEOPIXEL_PIN 88 // NeoPixel signal pin (Grand Central M4)
#define NUMPIXELS 1 // Number of NeoPixels (Grand Central has 1)
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void SetNeoPixel(const uint8_t r, const uint8_t g, const uint8_t b)
{
  pixels.setPixelColor(0, pixels.Color(r, g, b)); // Red
  pixels.show(); // Send the color to the NeoPixel
}

// =====================================================================================================================

void DragonChonk::Button::Init(const unsigned char PIN)
{
  _PIN = PIN;
  pinMode(_PIN, INPUT_PULLUP);
}

void DragonChonk::Button::Process(const long milliTime)
{
  const unsigned char reading = digitalRead(_PIN);
  // If the switch changed, due to noise or pressing:

  if ( reading != _lastButtonState )
  {
   // reset the debouncing timer
    _lastDebounceTime = milliTime;
  }

  _stateChanged = 0;
  if ( ( milliTime - _lastDebounceTime ) > kDebounceDelay )
  {
    _stateChanged = reading ^ _buttonState;
    _buttonState = reading;
  }
  _lastButtonState = reading;
}

// =====================================================================================================================

static const char NUM_BUTTONS            = 3;
static const char BUTTON_JOYSTICK_ENABLE = 0;
static const char BUTTON_DOWN            = 1;
static const char BUTTON_AUTOFOCUS       = 2;

static DragonChonk::Button         s_button[NUM_BUTTONS];

static DragonChonk::IndicatorLight s_buttonIndicatorLights[NUM_BUTTONS];
static bool s_bIsMoving  = false;

// =====================================================================================================================

static MString<64> s_lensSelect_GCODE[DragonChonk::MAX_LENSES];

static DragonChonk::Settings s_settings;
int DragonChonk::Settings::s_eeprom_base_address = 0; // Gets recorded in settings.cpp, when the settings are loaded

DragonChonk::Settings& DragonChonk_get_settings() { return s_settings; }

void DragonChonk::Settings::SetIndexInstalledLens(const int indexInstalledLens)
{
	m_indexInstalledLens = std::max(std::min(indexInstalledLens, int(DragonChonk::MAX_LENSES-1)), 0);
}

void DragonChonk_save_settings()
{
	#if ENABLED(EEPROM_SETTINGS)
	persistentStore.access_start();
	persistentStore.write_data(DragonChonk::Settings::s_eeprom_base_address, (uint8_t*)&s_settings, sizeof(s_settings));
	persistentStore.access_finish();
	settings.save();
	#endif
}

static float GetCurrentLensFocal_Z()
{
	return s_settings.m_lensFocalZ[s_settings.m_indexInstalledLens];
}

static void SetupDefaultLensList()
{
	for (unsigned int iLens = 0; iLens < DragonChonk::MAX_LENSES; iLens++)
	{
		s_settings.m_lensNames[iLens]  = "Lens #";
		s_settings.m_lensNames[iLens].append(iLens);
		s_settings.m_lensFocalZ[iLens] = 0.0f;
	}

	s_settings.m_lensNames[0]  = "F100 (70x70)";
	s_settings.m_lensFocalZ[0] = 25.0f;

	s_settings.m_lensNames[1] = "F254 (175x175)";
	s_settings.m_lensFocalZ[1] = 50.0f;

	s_settings.m_lensNames[2] = "F420 (300x300)";
	s_settings.m_lensFocalZ[2] = 75.0f;
}

void DragonChonk_reset_settings()
{
	s_settings.m_indexInstalledLens = 0;
	SetupDefaultLensList();
}

void DragonChonk_report_settings(const bool forReplay)
{
	GcodeSuite::report_heading(forReplay, F("Laser Settings"));
	SERIAL_ECHOLNPGM("  Installed Lens Index: ", s_settings.m_indexInstalledLens);
	for (unsigned int iLens = 0; iLens < DragonChonk::MAX_LENSES; iLens++)
	{
		SERIAL_ECHOLNPGM("  #", iLens, " Name: \"", &s_settings.m_lensNames[iLens], "\" Focal Z: ", s_settings.m_lensFocalZ[iLens]);
	}
}

// =====================================================================================================================

void PVV_FOREVER()
{
  for ( ;;)
  {
    PVVFOO("PVV Hello World!\n");
    //MSerial0.println("This works!\n");
    //MYSERIAL1.println("Doesn't work!\n");
    delay(1000);
    SetNeoPixel(0, 0, 128);
    delay(1000);
    SetNeoPixel(128, 0, 0);
  }
}

// =====================================================================================================================

void DragonChonk_setup()
{
	//delay(2000);

	PVVFOO("DragonChonk start\n");

	//#define JOY_Z_PIN   12  // RAMPS: Suggested pin A12 on AUX2
	//int iFoo = int(PIN_TO_ADC(JOY_Z_PIN));
	//SERIAL_ECHOLNPGM("PIN_TO_ADC(JOY_Z_PIN)=", iFoo);
	//SERIAL_ECHOLNPGM("(ANAPIN_TO_ADCAIN(JOY_Z_PIN) >> 8)=", int((ANAPIN_TO_ADCAIN(JOY_Z_PIN) >> 8)));
	//SERIAL_ECHOLNPGM("_PIN_TO_ADCAIN(ANAPIN_TO_SAMDPIN(JOY_Z_PIN))=", int(_PIN_TO_ADCAIN(ANAPIN_TO_SAMDPIN(JOY_Z_PIN))) );


	pixels.begin(); // Initialize NeoPixel library
	pixels.show(); // Turn off all pixels

	SetNeoPixel(0, 0, 128);

	pinMode(DRAGON_CHONK_BUTTON_AUTOFOCUS_LED_PIN, OUTPUT);
	pinMode(DRAGON_CHONK_BUTTON_AUTOFOCUS_PIN, INPUT_PULLUP);
	digitalWrite(DRAGON_CHONK_BUTTON_AUTOFOCUS_LED_PIN, LOW);
	//analogWrite(DRAGON_CHONK_BUTTON_AUTOFOCUS_LED_PIN, 64);

	s_button[BUTTON_JOYSTICK_ENABLE].Init(DRAGON_CHONK_BUTTON_JOYSTICK_ENABLE_PIN);
	s_buttonIndicatorLights[BUTTON_JOYSTICK_ENABLE].Init(DRAGON_CHONK_BUTTON_UP_LED_PIN);

	//s_button[BUTTON_DOWN].Init(DRAGON_CHONK_BUTTON_DOWN_PIN);
	//s_buttonIndicatorLights[BUTTON_DOWN].Init(DRAGON_CHONK_BUTTON_DOWN_LED_PIN);

	s_button[BUTTON_AUTOFOCUS].Init(DRAGON_CHONK_BUTTON_AUTOFOCUS_PIN);
	s_buttonIndicatorLights[BUTTON_AUTOFOCUS].Init(DRAGON_CHONK_BUTTON_AUTOFOCUS_LED_PIN);

	//pinMode(LED_BUILTIN, OUTPUT);
	//for ( ;;)
	//{
	//  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
	//  delay(100);                      // wait for a second
	//  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
	//  delay(100);                      // wait for a second
	//}
}

// =====================================================================================================================

static void ProcessButtons(const long milliTime)
{
	for ( unsigned char iButton = 0; iButton < NUM_BUTTONS; iButton++ )
	{
		s_button[iButton].Process(milliTime);
	}

	bool s_bEnabled = s_button[BUTTON_AUTOFOCUS].IsPressed();

	s_buttonIndicatorLights[BUTTON_JOYSTICK_ENABLE]
		.SetState(( s_bEnabled ) ? DragonChonk::IndicatorLight::State::ON : DragonChonk::IndicatorLight::State::PULSE)
		.SetCycleTime(4.0f);

	//s_buttonIndicatorLights[BUTTON_DOWN]
	//	.SetState(( s_bEnabled ) ? DragonChonk::IndicatorLight::State::ON : DragonChonk::IndicatorLight::State::PULSE)
	//	.SetCycleTime(1.0f);

	s_buttonIndicatorLights[BUTTON_AUTOFOCUS]
		.SetState(( s_bIsMoving ) ? DragonChonk::IndicatorLight::State::PULSE : DragonChonk::IndicatorLight::State::OFF )
		.SetCycleTime(0.25f);

	if ( !s_bIsMoving && s_button[BUTTON_AUTOFOCUS].ButtonDown() )
	{
		gcode.process_subcommands_now(F("G28 Z"));
		//LOG("BUTTON_GREEN\n");
	}

	for ( unsigned char iButton = 0; iButton < NUM_BUTTONS; iButton++ )
	{
		s_buttonIndicatorLights[iButton].Process(milliTime);
	}
}

// =====================================================================================================================

void DragonChonk_PostHoming()
{
	MString<128> gcode_string = "M300 P100\nG0 Z";
	gcode_string.append(GetCurrentLensFocal_Z());
	gcode_string += "\nG92 Z0";

	#if defined(DRAGON_CHONK_LOG_EVERYTHING)
	SERIAL_ECHOLNPGM("DragonChonk_PostHoming gcode: ", &gcode_string);
	#endif

	gcode.process_subcommands_now(F(&gcode_string)); // "M300 P100\nG0 Z50\nG92 Z0"
}

// =====================================================================================================================

void DragonChonk_Process()
{
	const long milliTime = millis();
	static long s_lastMoveSample_ms = milliTime;
	constexpr long kMoveSamplingInterval_ms = 100;
	if ((s_lastMoveSample_ms + kMoveSamplingInterval_ms) <= milliTime)
	{
		s_lastMoveSample_ms = milliTime;

		const float cartes_z = planner.get_axis_position_mm(Z_AXIS);
		static float s_previous_cartes_z = cartes_z;
		s_bIsMoving = (s_previous_cartes_z != cartes_z);
		s_previous_cartes_z = cartes_z;
	}

	ProcessButtons(milliTime);
}

// =====================================================================================================================

static void MarkHome_Z()
{
	set_axis_is_at_home(Z_AXIS);
	sync_plan_position();
}

// =====================================================================================================================

	  void DragonChonk_menu_lens_select()
{
	START_MENU();
	//"F100 (70x70)"
	//"F254 (175x175)"
	//"F420 (300x300)"

	//
	// ^ Laser
	//
	BACK_ITEM_F(F("Laser"));
	for (unsigned int i = 0; i < DragonChonk::MAX_LENSES; i++)
	{
		auto &lensSelect_gcode = s_lensSelect_GCODE[i];
		//const std::string name = s_settings.m_lensNames[i].buffer();
		lensSelect_gcode = "M300 S220 P100\nM4200 T";
		lensSelect_gcode.append(i);
		lensSelect_gcode += "\nM117 "; // M117 sets the status message
		lensSelect_gcode += "Lens #";
		lensSelect_gcode.append(i);
		lensSelect_gcode += " Selected";
		GCODES_ITEM_F(F(&s_settings.m_lensNames[i]), F(&lensSelect_gcode));
	}

	END_MENU();
}

void DragonChonk_menu_laser()
{
	START_MENU();

	//
	// ^ Main
	//
	BACK_ITEM(MSG_MAIN_MENU);

	ACTION_ITEM_F(F("Override Z Focused"), MarkHome_Z);
    SUBMENU_F(F("Select Lens"), DragonChonk_menu_lens_select);
	GCODES_ITEM_F(F("Set Focal Z"), F("M300 S220 P100\nM4201"));
	GCODES_ITEM_F(F("Auto-Focus (Home Z)"), F("G28 Z"));
	GCODES_ITEM_F(F("GOTO Z0"), F("G0 Z0"));

	END_MENU();
}


/**
 * M4200: Get or set Current Lens Number
 *
 * Parameters:
 *   T<int>  Lens number [0, DragonChonk::MAX_LENSES)
 *
 * Without parameters:
 *   Report the currently selected lens number
 */

void GcodeSuite::M4200()
{
	if (parser.seenval('T'))
	{
		const int indexLens = parser.value_long();
		s_settings.SetIndexInstalledLens(indexLens);
		DragonChonk_save_settings();
	}
	else
	{
		s_settings.SetIndexInstalledLens(s_settings.m_indexInstalledLens); // Clamp
		const std::string name = s_settings.m_lensNames[s_settings.m_indexInstalledLens].buffer();
		SERIAL_ECHOLNPGM(STR_DRAGON_CHONK_SELECTED_LENS, s_settings.m_indexInstalledLens, " Name: \"", name.c_str(), "\" Focal Z: ", GetCurrentLensFocal_Z());
	}
}

/**
 * M4201: Set Focal Distance to Current Z
 *
 * Parameters:
 *   None
 *
 */

void GcodeSuite::M4201()
{
	//planner.synchronize();
	SERIAL_ECHOLNPGM("current_position.z = ", current_position.z);
	const xyze_pos_t lpos = current_position.asLogical();
	SERIAL_ECHOLNPGM("current_position.asLogical().z = ", lpos.z);

	s_settings.m_lensFocalZ[s_settings.m_indexInstalledLens] = current_position.z;
	DragonChonk_save_settings();
}



/**
 * M4205: Set Lens Name
 *
 * Parameters:
 *   The name of the lens, in double-quotes
 *
 */

static std::string __trim(const std::string& str)
{
    size_t first = str.find_first_not_of(" \t\"");
    if (std::string::npos == first)
	{
        return str; // String contains only whitespace or is empty
    }
    size_t last = str.find_last_not_of(" \t\"");
    return str.substr(first, (last - first + 1));
}

void GcodeSuite::M4205()
{
	//if (parser.seenval('T'))
	//{
	//	const int indexLens = parser.value_long();
	//	SERIAL_ECHOLNPGM("parser.value_long(): ", indexLens);
	//}

	if (parser.has_string())
	{
		MString<DragonChonk::MAX_LENS_NAME_LENGTH> lens_name = parser.string_arg;

		lens_name = __trim(std::string(&lens_name)).c_str();

		//SERIAL_ECHOLNPGM("C: parser.value_string(): ", &lens_name);
		s_settings.m_lensNames[s_settings.m_indexInstalledLens] = lens_name;
		DragonChonk_save_settings();
	}
}


// =====================================================================================================================

#if HAS_MARLINUI_U8GLIB
#include "lcd/dogm/marlinui_DOGM.h"
#include "lcd/dogm/dogm_Statusscreen.h"

FORCE_INLINE void _draw_axis_value(const uint8_t ixBase, const AxisEnum axis, const char *value, const bool blink)
{
	lcd_put_lchar(3 + ixBase, 38, AXIS_CHAR(axis));
	lcd_moveto(11 + ixBase, 38);

	if (blink)
		lcd_put_u8str(value);
	else if (axis_should_home(axis))
		while (const char c = *value++) lcd_put_lchar(c <= '.' ? c : '?');
	else if (NONE(HOME_AFTER_DEACTIVATE, DISABLE_REDUCED_ACCURACY_WARNING) && !axis_is_trusted(axis))
		lcd_put_u8str(TERN0(HAS_Z_AXIS, axis == Z_AXIS) ? F("       ") : F("    "));
	else
		lcd_put_u8str(value);
}

/**
 * Draw the Laser Status Screen for a 128x64 DOGM (U8glib) display.
 *
 * Called as needed to update the current display stripe.
 * Use the PAGE_CONTAINS macros to avoid pointless draw calls.
 */
void MarlinUI::draw_laser_status_screen()
{
	constexpr int xystorage = TERN(INCH_MODE_SUPPORT, 8, 5);
	static char xstring[TERN(LCD_SHOW_E_TOTAL, 12, xystorage)];
	//static char ystring[xystorage];
	static char zstring[8];
	static MString<DragonChonk::MAX_LENS_NAME_LENGTH> s_lens_name;

	// At the first page, generate new display values
	if (first_page)
	{
		const xyz_pos_t lpos = current_position.asLogical();

		TERN_(HAS_X_AXIS, strcpy(xstring, ftostr4sign(lpos.x)));
		//TERN_(HAS_Y_AXIS, strcpy(ystring, ftostr4sign(lpos.y)));
		TERN_(HAS_Z_AXIS, strcpy(zstring, ftostr52sp(lpos.z)));
	}

	// Status Menu Font
	set_font(FONT_STATUSMENU);

	u8g.drawBitmapP(STATUS_LOGO_X, STATUS_LOGO_Y, STATUS_LOGO_BYTEWIDTH, STATUS_LOGO_HEIGHT, status_logo_bmp);

	const bool blink = ui.get_blink();

	//
	// XYZ Coordinates
	//

	u8g.drawFrame(0, 29, 90 /*LCD_PIXEL_WIDTH*/, 11);

	uint8_t ixBase = 0;
	TERN_(HAS_X_AXIS, _draw_axis_value(ixBase, X_AXIS, xstring, blink));
	ixBase += 37;
	//TERN_(HAS_Y_AXIS, _draw_axis_value(ixBase, Y_AXIS, ystring, blink));
	//ixBase += 37;
	TERN_(HAS_Z_AXIS, _draw_axis_value(ixBase, Z_AXIS, zstring, blink));
	ixBase += 37;

	set_font(FONT_STATUSMENU);
	s_lens_name = s_settings.m_lensNames[s_settings.m_indexInstalledLens].buffer();

	lcd_put_u8str(lcd_uint_t(0), lcd_uint_t(51), &s_lens_name);

	//
	// Status line
	//
	lcd_moveto(0, LCD_PIXEL_HEIGHT - INFO_FONT_DESCENT);
	draw_status_message(blink);
}
#endif // #if HAS_MARLINUI_U8GLIB

