#include <Arduino.h>
#include <SPI.h>

/*
 * ============================================================
 * CAR-MUNICATION - HOMEMADE ECU
 * ============================================================
 *
 * Hardware:
 *   ESP32 + MCP2515 + CAN transceiver
 *
 * CAN:
 *   500 kbps
 *
 * SPI:
 *   SCK  = GPIO18
 *   MISO = GPIO19
 *   MOSI = GPIO23
 *   CS   = GPIO5
 *
 * Addressing:
 *   0x7DF -> functional request   (ACTIVE)
 *   0x7E0 -> physical req, ECM    (ACTIVE, same as 0x7DF path)
 *   0x7E8 -> physical resp, ECM   (ACTIVE)
 *   0x7E1..0x7E7 / 0x7E9..0x7EF   (DEFINED, INACTIVE - placeholder)
 *
 * Supported OBD-II PIDs (Mode 01):
 *   00 20 40 60  Supported PID bitmaps
 *   01           Monitor status
 *   03           Fuel system status
 *   04           Calculated engine load
 *   05           Coolant temperature
 *   06           Short-term fuel trim
 *   07           Long-term fuel trim
 *   0A           Fuel pressure
 *   0B           Intake manifold pressure
 *   0C           Engine RPM
 *   0D           Vehicle speed
 *   0E           Timing advance
 *   0F           Intake air temperature
 *   10           Mass air flow
 *   11           Throttle position
 *   1C           OBD standard
 *   1F           Engine runtime
 *   21           Distance with MIL on
 *   2F           Fuel level
 *   31           Distance since DTC cleared
 *   33           Barometric pressure
 *   42           Control module voltage
 *   46           Ambient air temperature
 *   51           Fuel type
 *   5C           Engine oil temperature
 *   5E           Engine fuel rate
 *
 * Supported UDS:
 *   10 01      Default Session
 *   22 01 00   Engine RPM
 *   22 F1 90   VIN
 *   19 02 xx   Read DTC by status mask
 *
 * ISO-TP:
 *   Single Frame
 *   First Frame
 *   Flow Control
 *   Consecutive Frame
 *
 * Simulation:
 *   Type 1 - one simulation[] table
 *   Advances one row every 500 ms, cyclic
 *
 * Current simulated fault:
 *   Coolant = 110 C, threshold = 105 C
 *   DTC = P0118 (fixed, always active + stored)
 *   MIL = ON (tied to DTC active)
 *
 * Padding:
 *   0x00 on all frames
 */


/* ============================================================
 * MCP2515 PINS
 * ============================================================ */

#define MCP_SCK   18
#define MCP_MISO  19
#define MCP_MOSI  23
#define MCP_CS     5


/* ============================================================
 * MCP2515 REGISTERS
 * ============================================================ */

#define CANSTAT     0x0E
#define CANCTRL     0x0F

#define CANINTF     0x2C

#define CNF1        0x2A
#define CNF2        0x29
#define CNF3        0x28

#define RXB0CTRL    0x60
#define RXB0SIDH    0x61
#define RXB0SIDL    0x62
#define RXB0DLC     0x65
#define RXB0D0      0x66

#define TXB0CTRL    0x30
#define TXB0SIDH    0x31
#define TXB0SIDL    0x32
#define TXB0DLC     0x35
#define TXB0D0      0x36


/* ============================================================
 * MCP2515 COMMANDS
 * ============================================================ */

#define MCP_RESET       0xC0
#define MCP_WRITE       0x02
#define MCP_READ        0x03
#define MCP_RTS_TX0     0x81


/* ============================================================
 * CAN IDs
 * ============================================================
 *
 * Only OBD_REQUEST_ID and ECU_RESPONSE_ID are used by this
 * firmware. The rest of the physical addressing table is
 * declared here so the sniffer side has a shared reference,
 * and so future multi-ECU emulation has a place to hook in.
 * ============================================================ */

#define OBD_REQUEST_ID   0x7DF
#define ECU_RESPONSE_ID  0x7E8

#define ECU1_REQUEST_ID  0x7E0   /* Engine Control Module      */
#define ECU1_RESPONSE_ID 0x7E8
#define ECU2_REQUEST_ID  0x7E1   /* Transmission Control Module*/
#define ECU2_RESPONSE_ID 0x7E9
#define ECU3_REQUEST_ID  0x7E2
#define ECU3_RESPONSE_ID 0x7EA
#define ECU4_REQUEST_ID  0x7E3
#define ECU4_RESPONSE_ID 0x7EB
#define ECU5_REQUEST_ID  0x7E4
#define ECU5_RESPONSE_ID 0x7EC
#define ECU6_REQUEST_ID  0x7E5
#define ECU6_RESPONSE_ID 0x7ED
#define ECU7_REQUEST_ID  0x7E6
#define ECU7_RESPONSE_ID 0x7EE
#define ECU8_REQUEST_ID  0x7E7
#define ECU8_RESPONSE_ID 0x7EF


/* ============================================================
 * VEHICLE STATE
 * ============================================================
 *
 * Every field holds the RAW OBD byte value (or 16-bit raw)
 * that will be placed on CAN. Physical meaning is documented
 * next to each field.
 *
 * This keeps the simulation table self-contained and makes
 * each row directly translatable to a CAN response.
 * ============================================================ */

struct VehicleState
{
    /* PID 04 - Calculated engine load.    A = pct * 255 / 100 */
    uint8_t  engineLoad;

    /* PID 05 - Coolant temperature.       A = C + 40          */
    uint8_t  coolantRaw;

    /* PID 06 - Short-term fuel trim.      A = pct * 128/100 + 128 */
    uint8_t  stftRaw;

    /* PID 07 - Long-term fuel trim.       A = pct * 128/100 + 128 */
    uint8_t  ltftRaw;

    /* PID 0A - Fuel pressure.             A = kPa / 3         */
    uint8_t  fuelPressure;

    /* PID 0B - Intake manifold pressure.  A = kPa             */
    uint8_t  mapRaw;

    /* PID 0C - Engine RPM.                raw = rpm * 4       */
    uint16_t rpmRaw;

    /* PID 0D - Vehicle speed.             A = km/h            */
    uint8_t  speed;

    /* PID 0E - Timing advance.            A = (deg + 64) * 2  */
    uint8_t  timingAdvRaw;

    /* PID 0F - Intake air temperature.    A = C + 40          */
    uint8_t  intakeTempRaw;

    /* PID 10 - Mass air flow.             raw = g/s * 100     */
    uint16_t mafRaw;

    /* PID 11 - Throttle position.         A = pct * 255 / 100 */
    uint8_t  throttleRaw;

    /* PID 2F - Fuel level.                A = pct * 255 / 100 */
    uint8_t  fuelLevelRaw;

    /* PID 42 - Control module voltage.    raw = V * 1000      */
    uint16_t moduleVoltageRaw;

    /* PID 5C - Engine oil temperature.    A = C + 40          */
    uint8_t  oilTempRaw;

    /* PID 5E - Engine fuel rate.          raw = L/h * 20      */
    uint16_t fuelRateRaw;
};


/*
 * ============================================================
 * SIMULATION TABLE
 * ============================================================
 *
 * One row = one vehicle state.
 *
 * Advanced one row every SIMULATION_INTERVAL (500 ms).
 * Wraps to row 0 after the last row.
 *
 * Cycle:
 *   idle -> accelerate -> cruise -> decelerate -> idle
 *
 * Constants across all rows (not in the table):
 *   PID 33 barometric pressure = 99 kPa          (A = 99)
 *   PID 46 ambient temperature = 25 C            (A = 65)
 *   PID 1C OBD standard        = 6 (EOBD)
 *   PID 51 fuel type           = 1 (Gasoline)
 *
 * Coolant is always 110 C (A = 150). Fixed fault scenario.
 *
 * Row-by-row decoded intent (for reference):
 *   row  0 : idle       800 rpm   0 km/h    5 % thr
 *   row  1 : creep     1200 rpm   5 km/h   10 % thr
 *   row  2 : accel     1600 rpm  10 km/h   15 % thr
 *   row  3 : accel     2000 rpm  20 km/h   20 % thr
 *   row  4 : accel     2500 rpm  30 km/h   28 % thr
 *   row  5 : accel     3000 rpm  40 km/h   35 % thr
 *   row  6 : accel     3500 rpm  50 km/h   42 % thr
 *   row  7 : cruise    4000 rpm  60 km/h   50 % thr
 *   row  8 : decel     3500 rpm  55 km/h   40 % thr
 *   row  9 : decel     3000 rpm  45 km/h   32 % thr
 *   row 10 : decel     2400 rpm  35 km/h   25 % thr
 *   row 11 : decel     1800 rpm  20 km/h   15 % thr
 *   row 12 : decel     1200 rpm   8 km/h    8 % thr
 *   row 13 : idle       800 rpm   0 km/h    5 % thr
 * ============================================================ */

VehicleState simulation[] =
{
    /* -------------------------------------------------------
     * row 0 - idle
     * -------------------------------------------------------
     * load 15%, coolant 110C, stft +2%, ltft +1%,
     * fp 350kPa, map 30kPa, rpm 800, speed 0,
     * timing 10deg, intake 30C, maf 2.50g/s, thr 5%,
     * fuel 78%, volt 13.8V, oil 90C, rate 1.2L/h
     * ------------------------------------------------------- */
    {
        (uint8_t)(15  * 255 / 100),   /* engineLoad         = 38  */
        (uint8_t)(110 + 40),          /* coolantRaw         = 150 */
        (uint8_t)(2   * 128 / 100 + 128), /* stftRaw        = 130 */
        (uint8_t)(1   * 128 / 100 + 128), /* ltftRaw        = 129 */
        (uint8_t)(350 / 3),           /* fuelPressure       = 116 */
        (uint8_t)(30),                /* mapRaw             = 30  */
        (uint16_t)(800 * 4),          /* rpmRaw             = 3200*/
        (uint8_t)(0),                 /* speed              = 0   */
        (uint8_t)((10 + 64) * 2),     /* timingAdvRaw       = 148 */
        (uint8_t)(30 + 40),           /* intakeTempRaw      = 70  */
        (uint16_t)(250),              /* mafRaw             = 250 */
        (uint8_t)(5  * 255 / 100),    /* throttleRaw        = 12  */
        (uint8_t)(78 * 255 / 100),    /* fuelLevelRaw       = 198 */
        (uint16_t)(13800),            /* moduleVoltageRaw   = 13800*/
        (uint8_t)(90 + 40),           /* oilTempRaw         = 130 */
        (uint16_t)(24)                /* fuelRateRaw        = 24  */
    },

    /* row 1 - creep */
    {
        (uint8_t)(20  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(360 / 3),
        (uint8_t)(35),
        (uint16_t)(1200 * 4),
        (uint8_t)(5),
        (uint8_t)((12 + 64) * 2),
        (uint8_t)(31 + 40),
        (uint16_t)(420),
        (uint8_t)(10 * 255 / 100),
        (uint8_t)(78 * 255 / 100),
        (uint16_t)(13850),
        (uint8_t)(92 + 40),
        (uint16_t)(36)
    },

    /* row 2 - accel */
    {
        (uint8_t)(28  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(0   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(380 / 3),
        (uint8_t)(42),
        (uint16_t)(1600 * 4),
        (uint8_t)(10),
        (uint8_t)((16 + 64) * 2),
        (uint8_t)(32 + 40),
        (uint16_t)(680),
        (uint8_t)(15 * 255 / 100),
        (uint8_t)(78 * 255 / 100),
        (uint16_t)(13850),
        (uint8_t)(95 + 40),
        (uint16_t)(48)
    },

    /* row 3 - accel */
    {
        (uint8_t)(36  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(0   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(400 / 3),
        (uint8_t)(48),
        (uint16_t)(2000 * 4),
        (uint8_t)(20),
        (uint8_t)((19 + 64) * 2),
        (uint8_t)(33 + 40),
        (uint16_t)(1050),
        (uint8_t)(20 * 255 / 100),
        (uint8_t)(78 * 255 / 100),
        (uint16_t)(13850),
        (uint8_t)(98 + 40),
        (uint16_t)(66)
    },

    /* row 4 - accel */
    {
        (uint8_t)(46  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(-1  * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(420 / 3),
        (uint8_t)(55),
        (uint16_t)(2500 * 4),
        (uint8_t)(30),
        (uint8_t)((23 + 64) * 2),
        (uint8_t)(34 + 40),
        (uint16_t)(1500),
        (uint8_t)(28 * 255 / 100),
        (uint8_t)(78 * 255 / 100),
        (uint16_t)(13880),
        (uint8_t)(100 + 40),
        (uint16_t)(90)
    },

    /* row 5 - accel */
    {
        (uint8_t)(52  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(-1  * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(440 / 3),
        (uint8_t)(62),
        (uint16_t)(3000 * 4),
        (uint8_t)(40),
        (uint8_t)((26 + 64) * 2),
        (uint8_t)(35 + 40),
        (uint16_t)(1850),
        (uint8_t)(35 * 255 / 100),
        (uint8_t)(78 * 255 / 100),
        (uint16_t)(13890),
        (uint8_t)(102 + 40),
        (uint16_t)(108)
    },

    /* row 6 - accel */
    {
        (uint8_t)(55  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(-1  * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(460 / 3),
        (uint8_t)(68),
        (uint16_t)(3500 * 4),
        (uint8_t)(50),
        (uint8_t)((28 + 64) * 2),
        (uint8_t)(35 + 40),
        (uint16_t)(2200),
        (uint8_t)(42 * 255 / 100),
        (uint8_t)(77 * 255 / 100),
        (uint16_t)(13900),
        (uint8_t)(103 + 40),
        (uint16_t)(162)
    },

    /* row 7 - cruise */
    {
        (uint8_t)(65  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(0   * 128 / 100 + 128),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(480 / 3),
        (uint8_t)(75),
        (uint16_t)(4000 * 4),
        (uint8_t)(60),
        (uint8_t)((32 + 64) * 2),
        (uint8_t)(38 + 40),
        (uint16_t)(3000),
        (uint8_t)(50 * 255 / 100),
        (uint8_t)(77 * 255 / 100),
        (uint16_t)(14000),
        (uint8_t)(106 + 40),
        (uint16_t)(230)
    },

    /* row 8 - decel */
    {
        (uint8_t)(55  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(460 / 3),
        (uint8_t)(68),
        (uint16_t)(3500 * 4),
        (uint8_t)(55),
        (uint8_t)((28 + 64) * 2),
        (uint8_t)(37 + 40),
        (uint16_t)(2400),
        (uint8_t)(40 * 255 / 100),
        (uint8_t)(77 * 255 / 100),
        (uint16_t)(13950),
        (uint8_t)(105 + 40),
        (uint16_t)(168)
    },

    /* row 9 - decel */
    {
        (uint8_t)(45  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(430 / 3),
        (uint8_t)(60),
        (uint16_t)(3000 * 4),
        (uint8_t)(45),
        (uint8_t)((25 + 64) * 2),
        (uint8_t)(36 + 40),
        (uint16_t)(1800),
        (uint8_t)(32 * 255 / 100),
        (uint8_t)(77 * 255 / 100),
        (uint16_t)(13900),
        (uint8_t)(104 + 40),
        (uint16_t)(120)
    },

    /* row 10 - decel */
    {
        (uint8_t)(35  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(410 / 3),
        (uint8_t)(50),
        (uint16_t)(2400 * 4),
        (uint8_t)(35),
        (uint8_t)((21 + 64) * 2),
        (uint8_t)(34 + 40),
        (uint16_t)(1300),
        (uint8_t)(25 * 255 / 100),
        (uint8_t)(77 * 255 / 100),
        (uint16_t)(13880),
        (uint8_t)(102 + 40),
        (uint16_t)(84)
    },

    /* row 11 - decel */
    {
        (uint8_t)(25  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(390 / 3),
        (uint8_t)(42),
        (uint16_t)(1800 * 4),
        (uint8_t)(20),
        (uint8_t)((17 + 64) * 2),
        (uint8_t)(32 + 40),
        (uint16_t)(850),
        (uint8_t)(15 * 255 / 100),
        (uint8_t)(76 * 255 / 100),
        (uint16_t)(13850),
        (uint8_t)(98 + 40),
        (uint16_t)(54)
    },

    /* row 12 - decel */
    {
        (uint8_t)(18  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(370 / 3),
        (uint8_t)(35),
        (uint16_t)(1200 * 4),
        (uint8_t)(8),
        (uint8_t)((13 + 64) * 2),
        (uint8_t)(31 + 40),
        (uint16_t)(480),
        (uint8_t)(8  * 255 / 100),
        (uint8_t)(76 * 255 / 100),
        (uint16_t)(13830),
        (uint8_t)(94 + 40),
        (uint16_t)(34)
    },

    /* row 13 - idle */
    {
        (uint8_t)(15  * 255 / 100),
        (uint8_t)(110 + 40),
        (uint8_t)(2   * 128 / 100 + 128),
        (uint8_t)(1   * 128 / 100 + 128),
        (uint8_t)(350 / 3),
        (uint8_t)(30),
        (uint16_t)(800 * 4),
        (uint8_t)(0),
        (uint8_t)((10 + 64) * 2),
        (uint8_t)(30 + 40),
        (uint16_t)(250),
        (uint8_t)(5  * 255 / 100),
        (uint8_t)(76 * 255 / 100),
        (uint16_t)(13800),
        (uint8_t)(90 + 40),
        (uint16_t)(24)
    }
};

const uint8_t SIMULATION_SIZE =
    sizeof(simulation) / sizeof(simulation[0]);

uint8_t currentState = 0;

VehicleState state;


/* ============================================================
 * SIMULATION TIMING
 * ============================================================ */

unsigned long lastSimulationUpdate = 0;

const unsigned long SIMULATION_INTERVAL = 500;


/* ============================================================
 * RUNTIME COUNTERS
 * ============================================================
 *
 * These are not part of simulation[] because they only ever
 * increase. They are incremented by updateSimulation().
 *
 *   PID 1F : engine runtime, seconds since "key on"
 *   PID 21 : distance with MIL on, km
 *   PID 31 : distance since DTC cleared, km
 * ============================================================ */

uint16_t engineRuntimeSeconds = 0;
uint16_t distanceWithMilKm     = 0;
uint16_t distanceSinceClearKm  = 0;


/* ============================================================
 * VIN
 * ============================================================
 *
 * WMI = VF7 (Peugeot / Citroen, France)
 * VDS = ABHN00
 * VIS = 01234567
 * ============================================================ */

const char ECU_VIN[] = "VF7ABHN0001234567";


/* ============================================================
 * DTC
 * ============================================================
 *
 * Fixed fault scenario:
 *   P0118, always active, always stored, one occurrence.
 *   MIL is ON iff the DTC is active.
 * ============================================================ */

const uint16_t DTC_COOLANT_HIGH = 0x0118;
const uint8_t  DTC_COOLANT_THRESHOLD = 105;

struct DtcRecord
{
    uint16_t code;

    bool active;
    bool stored;

    uint16_t occurrences;
};

DtcRecord coolantDtc =
{
    DTC_COOLANT_HIGH,
    false,
    false,
    0
};


/* ============================================================
 * ISO-TP
 * ============================================================ */

#define ISO_TP_BUFFER_SIZE 64

uint8_t isoTpBuffer[ISO_TP_BUFFER_SIZE];

uint16_t isoTpTotalLength = 0;
uint16_t isoTpReceivedLength = 0;

uint8_t isoTpNextSequenceNumber = 1;

bool isoTpMultiFrameActive = false;


/* ============================================================
 * SPI CHIP SELECT
 * ============================================================ */

void mcpSelect()
{
    digitalWrite(MCP_CS, LOW);
}

void mcpDeselect()
{
    digitalWrite(MCP_CS, HIGH);
}


/* ============================================================
 * MCP2515 WRITE REGISTER
 * ============================================================ */

void mcpWriteRegister(uint8_t address, uint8_t value)
{
    mcpSelect();

    SPI.transfer(MCP_WRITE);
    SPI.transfer(address);
    SPI.transfer(value);

    mcpDeselect();
}


/* ============================================================
 * MCP2515 READ REGISTER
 * ============================================================ */

uint8_t mcpReadRegister(uint8_t address)
{
    mcpSelect();

    SPI.transfer(MCP_READ);
    SPI.transfer(address);

    uint8_t value = SPI.transfer(0x00);

    mcpDeselect();

    return value;
}


/* ============================================================
 * MCP2515 RESET
 * ============================================================ */

void mcpReset()
{
    mcpSelect();

    SPI.transfer(MCP_RESET);

    mcpDeselect();

    delay(10);
}


/* ============================================================
 * MCP2515 INITIALIZATION
 * ============================================================ */

bool initMCP2515()
{
    Serial.println();
    Serial.println("Initializing MCP2515...");

    mcpReset();

    /* Configuration mode. */
    mcpWriteRegister(CANCTRL, 0x80);

    delay(10);

    uint8_t canstat = mcpReadRegister(CANSTAT);

    Serial.print("CANSTAT = 0x");
    Serial.println(canstat, HEX);

    /*
     * 8 MHz crystal, 500 kbps.
     * Same configuration as the working setup.
     */

    mcpWriteRegister(CNF1, 0x00);
    mcpWriteRegister(CNF2, 0x90);
    mcpWriteRegister(CNF3, 0x02);

    /* Receive all standard frames. */
    mcpWriteRegister(RXB0CTRL, 0x60);

    /* Clear interrupt flags. */
    mcpWriteRegister(CANINTF, 0x00);

    /* Normal mode. */
    mcpWriteRegister(CANCTRL, 0x00);

    delay(10);

    canstat = mcpReadRegister(CANSTAT);

    Serial.print("CANSTAT after NORMAL = 0x");
    Serial.println(canstat, HEX);

    if ((canstat & 0xE0) == 0x00)
    {
        Serial.println("MCP2515 is in NORMAL mode.");
        return true;
    }

    Serial.println("ERROR: MCP2515 failed to enter NORMAL mode.");
    return false;
}


/* ============================================================
 * SET TRANSMIT CAN ID
 * ============================================================ */

void setTxId(uint16_t id)
{
    uint8_t sidh = id >> 3;
    uint8_t sidl = (id & 0x07) << 5;

    mcpWriteRegister(TXB0SIDH, sidh);
    mcpWriteRegister(TXB0SIDL, sidl);
}


/* ============================================================
 * SEND CAN FRAME
 * ============================================================
 *
 * Waits for TXB0 to be free before loading, and waits for
 * the actual transmission to complete before returning.
 * This prevents consecutive frames from being dropped while
 * TXB0 is still busy from the previous frame.
 * ============================================================ */

bool sendCanFrame(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    if (dlc > 8)
    {
        dlc = 8;
    }

    /* STEP 1 - wait until TXB0 is free. TXREQ = bit 3. */
    unsigned long startTime = millis();

    while (mcpReadRegister(TXB0CTRL) & 0x08)
    {
        if (millis() - startTime > 100)
        {
            Serial.println("ERROR: TX buffer remained busy.");
            return false;
        }
        delay(1);
    }

    /* STEP 2 - CAN ID. */
    setTxId(id);

    /* STEP 3 - DLC. */
    mcpWriteRegister(TXB0DLC, dlc);

    /* STEP 4 - payload. */
    for (uint8_t i = 0; i < dlc; i++)
    {
        mcpWriteRegister(TXB0D0 + i, data[i]);
    }

    /* STEP 5 - request transmission. */
    mcpSelect();
    SPI.transfer(MCP_RTS_TX0);
    mcpDeselect();

    /* STEP 6 - wait until transmission completes. */
    startTime = millis();

    while (mcpReadRegister(TXB0CTRL) & 0x08)
    {
        if (millis() - startTime > 100)
        {
            Serial.println("ERROR: CAN transmission timeout.");
            return false;
        }
        delay(1);
    }

    return true;
}


/* ============================================================
 * RECEIVE CAN FRAME
 * ============================================================ */

bool receiveCanFrame(uint16_t &id, uint8_t *data, uint8_t &dlc)
{
    uint8_t intf = mcpReadRegister(CANINTF);

    /* RX0 interrupt flag. */
    if (!(intf & 0x01))
    {
        return false;
    }

    uint8_t sidh = mcpReadRegister(RXB0SIDH);
    uint8_t sidl = mcpReadRegister(RXB0SIDL);

    id = ((uint16_t)sidh << 3) | ((sidl >> 5) & 0x07);

    dlc = mcpReadRegister(RXB0DLC) & 0x0F;

    if (dlc > 8)
    {
        dlc = 8;
    }

    for (uint8_t i = 0; i < dlc; i++)
    {
        data[i] = mcpReadRegister(RXB0D0 + i);
    }

    /* Clear RX0 interrupt. */
    mcpWriteRegister(CANINTF, intf & ~0x01);

    return true;
}


/* ============================================================
 * PRINT CAN FRAME
 * ============================================================ */

void printCanFrame(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    Serial.print("CAN 0x");

    if (id < 0x100)
    {
        Serial.print("0");
    }

    Serial.print(id, HEX);
    Serial.print(" [");
    Serial.print(dlc);
    Serial.print("] ");

    for (uint8_t i = 0; i < dlc; i++)
    {
        if (data[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}


/* ============================================================
 * ISO-TP SINGLE FRAME
 * ============================================================ */

void sendIsoTpSingleFrame(uint16_t canId, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[8];

    /* 0x00 padding (all 8 bytes). */
    for (uint8_t i = 0; i < 8; i++)
    {
        frame[i] = 0x00;
    }

    frame[0] = length & 0x0F;

    for (uint8_t i = 0; i < length && i < 7; i++)
    {
        frame[i + 1] = payload[i];
    }

    sendCanFrame(canId, frame, 8);
}


/* ============================================================
 * ISO-TP FIRST FRAME
 * ============================================================ */

bool sendIsoTpFirstFrame(uint16_t canId, const uint8_t *payload, uint16_t length)
{
    uint8_t frame[8];

    for (uint8_t i = 0; i < 8; i++)
    {
        frame[i] = 0x00;
    }

    frame[0] = 0x10 | ((length >> 8) & 0x0F);
    frame[1] = length & 0xFF;

    /* First 6 payload bytes. */
    for (uint8_t i = 0; i < 6; i++)
    {
        frame[i + 2] = payload[i];
    }

    return sendCanFrame(canId, frame, 8);
}


/* ============================================================
 * ISO-TP CONSECUTIVE FRAME
 * ============================================================ */

bool sendIsoTpConsecutiveFrame(
    uint16_t canId,
    const uint8_t *payload,
    uint16_t offset,
    uint8_t sequenceNumber,
    uint8_t length)
{
    uint8_t frame[8];

    for (uint8_t i = 0; i < 8; i++)
    {
        frame[i] = 0x00;
    }

    frame[0] = 0x20 | (sequenceNumber & 0x0F);

    for (uint8_t i = 0; i < length && i < 7; i++)
    {
        frame[i + 1] = payload[offset + i];
    }

    return sendCanFrame(canId, frame, 8);
}


/* ============================================================
 * FORWARD DECLARATIONS
 * ============================================================ */

void evaluateCoolantFault();


/* ============================================================
 * UPDATE SIMULATION
 * ============================================================
 *
 * Advances simulation[] one row every SIMULATION_INTERVAL.
 * Also ticks the runtime counters.
 * ============================================================ */

void updateSimulation()
{
    if (millis() - lastSimulationUpdate < SIMULATION_INTERVAL)
    {
        return;
    }

    lastSimulationUpdate = millis();

    currentState++;

    if (currentState >= SIMULATION_SIZE)
    {
        currentState = 0;
    }

    state = simulation[currentState];

    /* --- Runtime counters --- */

    engineRuntimeSeconds += (SIMULATION_INTERVAL / 1000);

    /*
     * Distance with MIL (PID 21):
     * increments only while the fault is active.
     * Approx. 1 km every 5 seconds for the demo.
     */

    static uint8_t milDistanceAccumulator = 0;

    if (coolantDtc.active)
    {
        milDistanceAccumulator++;

        if (milDistanceAccumulator >= 10)
        {
            milDistanceAccumulator = 0;
            distanceWithMilKm++;
        }
    }

    /*
     * Distance since DTC clear (PID 31):
     * increments every 5 seconds regardless.
     */

    static uint8_t clearDistanceAccumulator = 0;

    clearDistanceAccumulator++;

    if (clearDistanceAccumulator >= 10)
    {
        clearDistanceAccumulator = 0;
        distanceSinceClearKm++;
    }

    /* --- Serial trace --- */

    Serial.println();
    Serial.println("----- VEHICLE STATE -----");

    Serial.print("RPM      : ");
    Serial.println(state.rpmRaw / 4);

    Serial.print("Speed    : ");
    Serial.print(state.speed);
    Serial.println(" km/h");

    Serial.print("Coolant  : ");
    Serial.print(state.coolantRaw - 40);
    Serial.println(" C");

    Serial.print("Throttle : ");
    Serial.print(state.throttleRaw * 100 / 255);
    Serial.println(" %");

    evaluateCoolantFault();
}


/* ============================================================
 * DTC EVALUATION
 * ============================================================
 *
 * Fixed scenario: coolant is always 110 C, threshold 105 C.
 * P0118 is detected on the first evaluation and never clears.
 * ============================================================ */

void evaluateCoolantFault()
{
    uint8_t coolant = state.coolantRaw - 40;

    bool faultDetected = coolant >= DTC_COOLANT_THRESHOLD;

    if (faultDetected && !coolantDtc.active)
    {
        coolantDtc.active = true;
        coolantDtc.stored = true;
        coolantDtc.occurrences++;

        Serial.println();
        Serial.println("!!! DTC FAULT DETECTED !!!");

        Serial.println("DTC      : P0118");
        Serial.print("Coolant  : ");
        Serial.print(coolant);
        Serial.println(" C");
        Serial.println("ACTIVE   : YES");
        Serial.println("STORED   : YES");
        Serial.print("OCCURRENCES : ");
        Serial.println(coolantDtc.occurrences);
        Serial.println();
    }
}


/* ============================================================
 * SUPPORTED PID BITMAPS
 * ============================================================
 *
 * OBD-II PIDs 00, 20, 40, 60 return a 4-byte bitmap where
 * bit 31 corresponds to PID 0x01 (or 0x21, 0x41, 0x61).
 *
 * We build the bitmaps by asking encodePid() whether a PID
 * is supported.
 *
 * encodePid() is declared below; forward declaration here.
 * ============================================================ */

uint8_t encodePid(uint8_t pid, uint8_t *out);

bool isPidSupported(uint8_t pid)
{
    uint8_t scratch[8];
    return encodePid(pid, scratch) > 0;
}


/* ============================================================
 * BUILD SUPPORTED PID BITMAP
 * ============================================================
 *
 * base = 0x00, 0x20, 0x40 or 0x60
 *
 * The bitmap covers PIDs base+1 .. base+32.
 * Bit 31 (MSB of first byte) corresponds to base+1.
 * ============================================================ */

void buildSupportedBitmap(uint8_t base, uint8_t *out)
{
    uint32_t mask = 0;

    for (uint8_t i = 0; i < 32; i++)
    {
        uint8_t pid = base + 1 + i;

        if (isPidSupported(pid))
        {
            /* Bit 31 = i=0, bit 0 = i=31. */
            mask |= (uint32_t)1 << (31 - i);
        }
    }

    out[0] = (mask >> 24) & 0xFF;
    out[1] = (mask >> 16) & 0xFF;
    out[2] = (mask >>  8) & 0xFF;
    out[3] =  mask        & 0xFF;
}


/* ============================================================
 * PID ENCODER
 * ============================================================
 *
 * One big switch. Returns the number of data bytes written
 * to out[] (0 = PID not supported).
 *
 * Layout matches OBD-II Mode 01 response payload:
 *   41 <pid> <data...>
 *
 * This function writes ONLY <data...>.
 * The caller wraps it in the 41 <pid> header.
 * ============================================================ */

uint8_t encodePid(uint8_t pid, uint8_t *out)
{
    switch (pid)
    {
        /* ------------------------------------------------
         * PID 00 - Supported PIDs [01-20]
         * ------------------------------------------------ */
        case 0x00:
            buildSupportedBitmap(0x00, out);
            return 4;

        /* ------------------------------------------------
         * PID 20 - Supported PIDs [21-40]
         * ------------------------------------------------ */
        case 0x20:
            buildSupportedBitmap(0x20, out);
            return 4;

        /* ------------------------------------------------
         * PID 40 - Supported PIDs [41-60]
         * ------------------------------------------------ */
        case 0x40:
            buildSupportedBitmap(0x40, out);
            return 4;

        /* ------------------------------------------------
         * PID 60 - Supported PIDs [61-80]
         * ------------------------------------------------ */
        case 0x60:
            buildSupportedBitmap(0x60, out);
            return 4;

        /* ------------------------------------------------
         * PID 01 - Monitor status
         *
         * Byte A:
         *   bit 7 = MIL on/off   (1 = ON)
         *   bits 6-0 = DTC count (max 127)
         *
         * Byte B, C, D: readiness monitors.
         * 0x00 0x07 0xE5 is a plausible "spark ignition"
         * readiness set with most monitors complete.
         * ------------------------------------------------ */
        case 0x01:
        {
            uint8_t mil = coolantDtc.active ? 0x80 : 0x00;
            uint8_t dtcCount = coolantDtc.active ? 1 : 0;

            out[0] = mil | (dtcCount & 0x7F);
            out[1] = 0x00;
            out[2] = 0x07;
            out[3] = 0xE5;
            return 4;
        }

        /* ------------------------------------------------
         * PID 03 - Fuel system status
         *
         * Byte A = system 1 status.
         * Byte B = system 2 status.
         *
         * 0x02 = closed loop, using O2 sensor.
         * 0x00 = not available / open loop.
         * ------------------------------------------------ */
        case 0x03:
            out[0] = 0x02;
            out[1] = 0x00;
            return 2;

        /* ------------------------------------------------
         * PID 04 - Calculated engine load
         * ------------------------------------------------ */
        case 0x04:
            out[0] = state.engineLoad;
            return 1;

        /* ------------------------------------------------
         * PID 05 - Coolant temperature
         * ------------------------------------------------ */
        case 0x05:
            out[0] = state.coolantRaw;
            return 1;

        /* ------------------------------------------------
         * PID 06 - Short-term fuel trim
         * ------------------------------------------------ */
        case 0x06:
            out[0] = state.stftRaw;
            return 1;

        /* ------------------------------------------------
         * PID 07 - Long-term fuel trim
         * ------------------------------------------------ */
        case 0x07:
            out[0] = state.ltftRaw;
            return 1;

        /* ------------------------------------------------
         * PID 0A - Fuel pressure
         * ------------------------------------------------ */
        case 0x0A:
            out[0] = state.fuelPressure;
            return 1;

        /* ------------------------------------------------
         * PID 0B - Intake manifold pressure
         * ------------------------------------------------ */
        case 0x0B:
            out[0] = state.mapRaw;
            return 1;

        /* ------------------------------------------------
         * PID 0C - Engine RPM
         * ------------------------------------------------ */
        case 0x0C:
            out[0] = (state.rpmRaw >> 8) & 0xFF;
            out[1] =  state.rpmRaw       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 0D - Vehicle speed
         * ------------------------------------------------ */
        case 0x0D:
            out[0] = state.speed;
            return 1;

        /* ------------------------------------------------
         * PID 0E - Timing advance
         * ------------------------------------------------ */
        case 0x0E:
            out[0] = state.timingAdvRaw;
            return 1;

        /* ------------------------------------------------
         * PID 0F - Intake air temperature
         * ------------------------------------------------ */
        case 0x0F:
            out[0] = state.intakeTempRaw;
            return 1;

        /* ------------------------------------------------
         * PID 10 - Mass air flow
         * ------------------------------------------------ */
        case 0x10:
            out[0] = (state.mafRaw >> 8) & 0xFF;
            out[1] =  state.mafRaw       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 11 - Throttle position
         * ------------------------------------------------ */
        case 0x11:
            out[0] = state.throttleRaw;
            return 1;

        /* ------------------------------------------------
         * PID 1C - OBD standard
         *
         * A = 6 -> EOBD (European OBD)
         * ------------------------------------------------ */
        case 0x1C:
            out[0] = 0x06;
            return 1;

        /* ------------------------------------------------
         * PID 1F - Engine runtime since start
         * ------------------------------------------------ */
        case 0x1F:
            out[0] = (engineRuntimeSeconds >> 8) & 0xFF;
            out[1] =  engineRuntimeSeconds       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 21 - Distance travelled with MIL on
         * ------------------------------------------------ */
        case 0x21:
            out[0] = (distanceWithMilKm >> 8) & 0xFF;
            out[1] =  distanceWithMilKm       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 2F - Fuel level
         * ------------------------------------------------ */
        case 0x2F:
            out[0] = state.fuelLevelRaw;
            return 1;

        /* ------------------------------------------------
         * PID 31 - Distance since DTC cleared
         * ------------------------------------------------ */
        case 0x31:
            out[0] = (distanceSinceClearKm >> 8) & 0xFF;
            out[1] =  distanceSinceClearKm       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 33 - Barometric pressure
         *
         * A = kPa
         * ------------------------------------------------ */
        case 0x33:
            out[0] = 99;
            return 1;

        /* ------------------------------------------------
         * PID 42 - Control module voltage
         * ------------------------------------------------ */
        case 0x42:
            out[0] = (state.moduleVoltageRaw >> 8) & 0xFF;
            out[1] =  state.moduleVoltageRaw       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * PID 46 - Ambient air temperature
         *
         * A = C + 40
         * ------------------------------------------------ */
        case 0x46:
            out[0] = 25 + 40;
            return 1;

        /* ------------------------------------------------
         * PID 51 - Fuel type
         *
         * A = 1 -> Gasoline
         * ------------------------------------------------ */
        case 0x51:
            out[0] = 0x01;
            return 1;

        /* ------------------------------------------------
         * PID 5C - Engine oil temperature
         * ------------------------------------------------ */
        case 0x5C:
            out[0] = state.oilTempRaw;
            return 1;

        /* ------------------------------------------------
         * PID 5E - Engine fuel rate
         * ------------------------------------------------ */
        case 0x5E:
            out[0] = (state.fuelRateRaw >> 8) & 0xFF;
            out[1] =  state.fuelRateRaw       & 0xFF;
            return 2;

        /* ------------------------------------------------
         * Unknown / unsupported PID.
         * ------------------------------------------------ */
        default:
            return 0;
    }
}


/* ============================================================
 * OBD-II RESPONSE
 * ============================================================ */

void sendOBDResponse(uint8_t pid)
{
    uint8_t data[8];
    uint8_t dataLength = encodePid(pid, data);

    if (dataLength == 0)
    {
        Serial.print("Unsupported OBD PID: 0x");

        if (pid < 0x10)
        {
            Serial.print("0");
        }

        Serial.println(pid, HEX);

        return;
    }

    /*
     * Payload:
     *   41 <pid> <data...>
     */

    uint8_t payload[6];
    uint8_t payloadLength = 0;

    payload[payloadLength++] = 0x41;
    payload[payloadLength++] = pid;

    for (uint8_t i = 0; i < dataLength && payloadLength < 6; i++)
    {
        payload[payloadLength++] = data[i];
    }

    sendIsoTpSingleFrame(ECU_RESPONSE_ID, payload, payloadLength);

    Serial.print("OBD PID 0x");

    if (pid < 0x10)
    {
        Serial.print("0");
    }

    Serial.print(pid, HEX);
    Serial.print(" -> ");

    for (uint8_t i = 0; i < dataLength; i++)
    {
        if (data[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(data[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}


/* ============================================================
 * SEND VIN RESPONSE
 * ============================================================ */

void sendVinResponse()
{
    /*
     * UDS positive response:
     *   62 F1 90 + 17 VIN characters = 20 bytes total.
     */

    uint8_t payload[32];
    uint16_t length = 0;

    payload[length++] = 0x62;
    payload[length++] = 0xF1;
    payload[length++] = 0x90;

    for (uint8_t i = 0; i < 17; i++)
    {
        payload[length++] = ECU_VIN[i];
    }

    Serial.println();
    Serial.println("Sending VIN response...");

    Serial.print("VIN: ");
    Serial.println(ECU_VIN);

    if (!sendIsoTpFirstFrame(ECU_RESPONSE_ID, payload, length))
    {
        Serial.println("ERROR: VIN First Frame failed.");
        return;
    }

    /* Wait for Flow Control. */
    unsigned long startTime = millis();

    while (millis() - startTime < 2000)
    {
        uint16_t id;
        uint8_t data[8];
        uint8_t dlc;

        if (receiveCanFrame(id, data, dlc))
        {
            if (id == OBD_REQUEST_ID && (data[0] & 0xF0) == 0x30)
            {
                Serial.println("VIN Flow Control received.");

                /* CF1: payload bytes 6..12 */
                if (!sendIsoTpConsecutiveFrame(
                        ECU_RESPONSE_ID, payload, 6, 1, 7))
                {
                    Serial.println("ERROR: VIN CF1 failed.");
                    return;
                }

                /* CF2: payload bytes 13..19 */
                if (!sendIsoTpConsecutiveFrame(
                        ECU_RESPONSE_ID, payload, 13, 2, 7))
                {
                    Serial.println("ERROR: VIN CF2 failed.");
                    return;
                }

                Serial.println("VIN transmission complete.");
                return;
            }
        }
    }

    Serial.println("ERROR: VIN Flow Control timeout.");
}


/* ============================================================
 * SEND DTC RESPONSE
 * ============================================================ */

bool sendIsoTpDtcResponse(uint8_t statusMask)
{
    /*
     * Payload:
     *   59 02 <statusMask> 00 01 00 <DTC hi> <DTC lo> <status>
     * = 9 bytes.
     */

    uint8_t payload[9];

    payload[0] = 0x59;
    payload[1] = 0x02;
    payload[2] = statusMask;

    payload[3] = 0x00;
    payload[4] = 0x01;
    payload[5] = 0x00;

    payload[6] = (coolantDtc.code >> 8) & 0xFF;
    payload[7] =  coolantDtc.code       & 0xFF;

    /*
     * Status byte:
     *   bit 0 = active
     *   bit 3 = stored
     */

    uint8_t dtcStatus = 0;

    if (coolantDtc.active)
    {
        dtcStatus |= 0x01;
    }

    if (coolantDtc.stored)
    {
        dtcStatus |= 0x08;
    }

    payload[8] = dtcStatus;

    Serial.println();
    Serial.println("Sending DTC response.");
    Serial.println("DTC       : P0118");

    Serial.print("Status    : 0x");

    if (dtcStatus < 0x10)
    {
        Serial.print("0");
    }

    Serial.println(dtcStatus, HEX);

    if (!sendIsoTpFirstFrame(ECU_RESPONSE_ID, payload, 9))
    {
        Serial.println("ERROR: DTC First Frame failed.");
        return false;
    }

    unsigned long startTime = millis();

    while (millis() - startTime < 2000)
    {
        uint16_t id;
        uint8_t data[8];
        uint8_t dlc;

        if (receiveCanFrame(id, data, dlc))
        {
            if (id == OBD_REQUEST_ID && (data[0] & 0xF0) == 0x30)
            {
                Serial.println("DTC Flow Control received.");

                if (!sendIsoTpConsecutiveFrame(
                        ECU_RESPONSE_ID, payload, 6, 1, 3))
                {
                    Serial.println("ERROR: DTC CF failed.");
                    return false;
                }

                Serial.println("DTC transmission complete.");
                return true;
            }
        }
    }

    Serial.println("ERROR: DTC Flow Control timeout.");
    return false;
}


/* ============================================================
 * HANDLE UDS REQUEST
 * ============================================================ */

void handleUdsRequest(uint8_t *payload, uint8_t length)
{
    if (length == 0)
    {
        return;
    }

    uint8_t service = payload[0];

    Serial.print("UDS Service: 0x");
    Serial.println(service, HEX);

    /* ========================================================
     * 0x10 - DIAGNOSTIC SESSION
     * ======================================================== */

    if (service == 0x10)
    {
        if (length >= 2)
        {
            uint8_t subFunction = payload[1];

            if (subFunction == 0x01)
            {
                uint8_t response[2];

                response[0] = 0x50;
                response[1] = 0x01;

                sendIsoTpSingleFrame(ECU_RESPONSE_ID, response, 2);

                Serial.println("UDS Default Session response sent.");
            }
        }

        return;
    }

    /* ========================================================
     * 0x22 - READ DATA BY IDENTIFIER
     * ======================================================== */

    if (service == 0x22)
    {
        if (length < 3)
        {
            return;
        }

        uint16_t did = ((uint16_t)payload[1] << 8) | payload[2];

        Serial.print("UDS DID: 0x");

        if (did < 0x1000)
        {
            Serial.print("0");
        }

        Serial.println(did, HEX);

        /* DID 0100 = RPM */
        if (did == 0x0100)
        {
            uint8_t response[4];

            response[0] = 0x62;
            response[1] = 0x01;
            response[2] = 0x00;

            /* rpm / 100, one byte */
            response[3] = (state.rpmRaw / 4) / 100;

            sendIsoTpSingleFrame(ECU_RESPONSE_ID, response, 4);

            Serial.print("UDS RPM response -> ");
            Serial.println(state.rpmRaw / 4);

            return;
        }

        /* DID F190 = VIN */
        if (did == 0xF190)
        {
            sendVinResponse();
            return;
        }

        return;
    }

    /* ========================================================
     * 0x19 - READ DTC INFORMATION
     * ======================================================== */

    if (service == 0x19)
    {
        if (length < 3)
        {
            return;
        }

        uint8_t subFunction = payload[1];

        /* 19 02 xx - Report DTC by status mask. */
        if (subFunction == 0x02)
        {
            uint8_t statusMask = payload[2];

            uint8_t dtcStatus = 0;

            if (coolantDtc.active)
            {
                dtcStatus |= 0x01;
            }

            if (coolantDtc.stored)
            {
                dtcStatus |= 0x08;
            }

            Serial.println();
            Serial.println("UDS ReadDTCInformation request.");

            Serial.print("Status mask: 0x");

            if (statusMask < 0x10)
            {
                Serial.print("0");
            }

            Serial.println(statusMask, HEX);

            Serial.print("DTC status: 0x");

            if (dtcStatus < 0x10)
            {
                Serial.print("0");
            }

            Serial.println(dtcStatus, HEX);

            if ((dtcStatus & statusMask) != 0)
            {
                sendIsoTpDtcResponse(statusMask);
            }
            else
            {
                /* No matching DTC. */
                uint8_t response[5];

                response[0] = 0x59;
                response[1] = 0x02;
                response[2] = statusMask;
                response[3] = 0x00;
                response[4] = 0x00;

                sendIsoTpSingleFrame(ECU_RESPONSE_ID, response, 5);
            }

            return;
        }

        return;
    }
}


/* ============================================================
 * HANDLE OBD REQUEST
 * ============================================================ */

void handleObdRequest(uint8_t *payload, uint8_t length)
{
    if (length < 2)
    {
        return;
    }

    if (payload[0] == 0x01)
    {
        uint8_t pid = payload[1];

        Serial.print("OBD Mode 01 PID: 0x");

        if (pid < 0x10)
        {
            Serial.print("0");
        }

        Serial.println(pid, HEX);

        sendOBDResponse(pid);
    }
}


/* ============================================================
 * PROCESS CAN FRAME
 * ============================================================ */

void processCanFrame(uint16_t id, uint8_t *data, uint8_t dlc)
{
    /*
     * We accept functional (0x7DF) and physical (0x7E0) requests.
     * Everything else is ignored for now (placeholder for
     * future multi-ECU emulation).
     */

    if (id != OBD_REQUEST_ID && id != ECU1_REQUEST_ID)
    {
        return;
    }

    if (dlc == 0)
    {
        return;
    }

    uint8_t frameType = data[0] & 0xF0;

    /* ========================================================
     * SINGLE FRAME
     * ======================================================== */

    if (frameType == 0x00)
    {
        uint8_t length = data[0] & 0x0F;

        if (length > 7)
        {
            return;
        }

        uint8_t payload[7];

        for (uint8_t i = 0; i < length; i++)
        {
            payload[i] = data[i + 1];
        }

        Serial.println();
        Serial.println("ISO-TP Single Frame received.");

        if (payload[0] == 0x01)
        {
            handleObdRequest(payload, length);
        }
        else
        {
            handleUdsRequest(payload, length);
        }

        return;
    }

    /* ========================================================
     * FIRST FRAME
     * ======================================================== */

    if (frameType == 0x10)
    {
        isoTpTotalLength = ((data[0] & 0x0F) << 8) | data[1];

        isoTpReceivedLength = 0;
        isoTpNextSequenceNumber = 1;
        isoTpMultiFrameActive = true;

        for (uint8_t i = 0; i < 6; i++)
        {
            if (isoTpReceivedLength < ISO_TP_BUFFER_SIZE)
            {
                isoTpBuffer[isoTpReceivedLength++] = data[i + 2];
            }
        }

        Serial.println();
        Serial.println("ISO-TP First Frame received.");

        Serial.print("Total payload length: ");
        Serial.println(isoTpTotalLength);

        /* Flow Control: 30 00 00 + 0x00 padding. */
        uint8_t fc[8];

        fc[0] = 0x30;
        fc[1] = 0x00;
        fc[2] = 0x00;

        for (uint8_t i = 3; i < 8; i++)
        {
            fc[i] = 0x00;
        }

        sendCanFrame(ECU_RESPONSE_ID, fc, 8);

        return;
    }

    /* ========================================================
     * CONSECUTIVE FRAME
     * ======================================================== */

    if (frameType == 0x20)
    {
        if (!isoTpMultiFrameActive)
        {
            return;
        }

        uint8_t sequenceNumber = data[0] & 0x0F;

        if (sequenceNumber != isoTpNextSequenceNumber)
        {
            Serial.println("ISO-TP sequence error.");

            isoTpMultiFrameActive = false;
            return;
        }

        isoTpNextSequenceNumber++;

        if (isoTpNextSequenceNumber > 15)
        {
            isoTpNextSequenceNumber = 0;
        }

        for (uint8_t i = 0; i < 7; i++)
        {
            if (isoTpReceivedLength < isoTpTotalLength &&
                isoTpReceivedLength < ISO_TP_BUFFER_SIZE)
            {
                isoTpBuffer[isoTpReceivedLength++] = data[i + 1];
            }
        }

        if (isoTpReceivedLength >= isoTpTotalLength)
        {
            Serial.println("ISO-TP multi-frame request complete.");

            isoTpMultiFrameActive = false;

            handleUdsRequest(isoTpBuffer, isoTpTotalLength);
        }

        return;
    }

    /* ========================================================
     * FLOW CONTROL
     * ======================================================== */

    if (frameType == 0x30)
    {
        /* Transmission functions handle FC. */
        return;
    }
}


/* ============================================================
 * CHECK CAN BUS
 * ============================================================ */

void checkCanBus()
{
    uint16_t id;
    uint8_t data[8];
    uint8_t dlc;

    if (receiveCanFrame(id, data, dlc))
    {
        printCanFrame(id, data, dlc);
        processCanFrame(id, data, dlc);
    }
}


/* ============================================================
 * INITIAL VEHICLE STATE
 * ============================================================ */

void initializeVehicleState()
{
    currentState = 0;
    state = simulation[currentState];

    engineRuntimeSeconds = 0;
    distanceWithMilKm     = 0;
    distanceSinceClearKm  = 0;

    Serial.println();
    Serial.println("Initial vehicle state:");

    Serial.print("RPM      : ");
    Serial.println(state.rpmRaw / 4);

    Serial.print("Speed    : ");
    Serial.print(state.speed);
    Serial.println(" km/h");

    Serial.print("Coolant  : ");
    Serial.print(state.coolantRaw - 40);
    Serial.println(" C");

    Serial.print("Throttle : ");
    Serial.print(state.throttleRaw * 100 / 255);
    Serial.println(" %");
}


/* ============================================================
 * INITIAL DTC
 * ============================================================ */

void initializeDtc()
{
    /*
     * Coolant is already 110 C at row 0, so P0118 is detected
     * immediately.
     */

    evaluateCoolantFault();
}


/* ============================================================
 * SETUP
 * ============================================================ */

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println();

    Serial.println("========================================");
    Serial.println(" CAR-MUNICATION HOMEMADE ECU");
    Serial.println("========================================");

    /* SPI */
    pinMode(MCP_CS, OUTPUT);
    digitalWrite(MCP_CS, HIGH);

    SPI.begin(MCP_SCK, MCP_MISO, MCP_MOSI, MCP_CS);

    /* MCP2515 */
    if (!initMCP2515())
    {
        Serial.println();
        Serial.println("MCP2515 initialization FAILED.");

        while (true)
        {
            delay(1000);
        }
    }

    /* Vehicle state */
    initializeVehicleState();

    /* DTC */
    initializeDtc();

    lastSimulationUpdate = millis();

    Serial.println();
    Serial.println("========================================");
    Serial.println(" ECU READY");
    Serial.println("========================================");

    Serial.println("CAN bitrate : 500 kbps");
    Serial.println("OBD ID      : 0x7DF");
    Serial.println("ECU ID      : 0x7E8");
    Serial.println("VIN         : VF7ABHN0001234567");
    Serial.println("Coolant     : 110 C");
    Serial.println("Threshold   : 105 C");
    Serial.println("DTC         : P0118");
    Serial.println("DTC Active  : YES");
    Serial.println("DTC Stored  : YES");
    Serial.println("Occurrences : 1");
    Serial.println("Simulation  : 500 ms / row");

    Serial.println("========================================");
}


/* ============================================================
 * LOOP
 * ============================================================ */

void loop()
{
    checkCanBus();
    updateSimulation();
    delay(1);
}