/*******************************************************************************
 * @file main.cpp
 * @brief Main application for E-Week 2026 robot control
 *
 * This is the entry point for the robot control system. It initializes all
 * subsystems, configures the watchdog, and runs the main control loop.
 *
 * @par System Overview:
 * - RC Receiver: FlySky iBUS protocol on UART1
 * - Drive Train: Differential (tank) drive
 * - Mechanisms: Scoop and Launch
 * - Safety: Watchdog timer, signal loss detection
 * - Display: SSD1306 OLED on core 1 (I2C0)
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"

// Project headers
#include "config.h"
#include "pinout.h"
#include "version.h"
#include "error_handler.h"
#include "flysky_ibus.h"
#include "drive_train_differential.h"
#include "mech_scoop.h"
#include "mech_launcher.h"

#if ENABLE_DISPLAY
#include "display_view.h"
#endif


/* Globals -------------------------------------------------------------------*/

/** @brief RC receiver interface */
static FlySkyIBus* g_pIBus = nullptr;

/** @brief Drive train controller */
static DriveTrainDifferential* g_pDriveTrain = nullptr;

/** @brief Scoop mechanism */
static MechScoop* g_pScoop = nullptr;

/** @brief Launcher mechanism */
static MechLauncher* g_pLauncher = nullptr;

#if ENABLE_DISPLAY
/** @brief Display view (owned here, runs on core 1) */
static DisplayView* g_pDisplayView = nullptr;

/** @brief Flag to track if last boot was from watchdog (captured before init) */
static bool g_watchdogRebooted = false;
#endif


/* Core 1 Entry (Display) ---------------------------------------------------*/

#if ENABLE_DISPLAY
/*******************************************************************************
 * @brief Core 1 entry point — display update loop
 *
 * Initializes the display hardware and runs the refresh loop. This function
 * never returns. Data from core 0 arrives via DisplayView::pushData().
 ******************************************************************************/
static void core1_main(void)
{
  if (g_pDisplayView == nullptr)
  {
    return;
  }

  if (!g_pDisplayView->init())
  {
#if ENABLE_DEBUG
    /* Note: printf from core 1 is safe with pico_stdlib's mutex-protected stdio */
    printf("[Display] OLED initialization failed on core 1\n");
#endif
    return;
  }

#if ENABLE_DEBUG
  printf("[Display] OLED initialized on core 1\n");
#endif

  /* Display refresh loop (never returns) */
  while (true)
  {
    g_pDisplayView->update();
    sleep_ms(TIMING_DISPLAY_REFRESH_MS);
  }
}
#endif /* ENABLE_DISPLAY */


/* Private Function Definitions ----------------------------------------------*/

/*******************************************************************************
 * @brief Initialize the differential drive motors
 *
 * Configures left and right motors with the appropriate pin assignments
 * based on the motor driver wiring mode selected in config.h.
 *
 * @param pDriveTrain Pointer to the drive train controller
 ******************************************************************************/
static void init_motors(DriveTrainDifferential* pDriveTrain)
{
  if (pDriveTrain != nullptr)
  {
#if MOTOR_DRIVER_MODE_2PWM
    pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_LEFT,
                          PIN_DIFF_MOTOR_LEFT_DIR_FWD,
                          PIN_DIFF_MOTOR_LEFT_DIR_REV,
                          PIN_DIFF_MOTOR_LEFT_ENC);
    pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_RIGHT,
                          PIN_DIFF_MOTOR_RIGHT_DIR_FWD,
                          PIN_DIFF_MOTOR_RIGHT_DIR_REV,
                          PIN_DIFF_MOTOR_RIGHT_ENC);
#elif MOTOR_DRIVER_MODE_1PWM_2DIR
    pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_LEFT,
                          PIN_DIFF_MOTOR_LEFT_ENABLE,
                          PIN_DIFF_MOTOR_LEFT_DIR_FWD,
                          PIN_DIFF_MOTOR_LEFT_DIR_REV,
                          PIN_DIFF_MOTOR_LEFT_ENC);
    pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_RIGHT,
                          PIN_DIFF_MOTOR_RIGHT_ENABLE,
                          PIN_DIFF_MOTOR_RIGHT_DIR_FWD,
                          PIN_DIFF_MOTOR_RIGHT_DIR_REV,
                          PIN_DIFF_MOTOR_RIGHT_ENC);
#endif
  }
}

/*******************************************************************************
 * @brief Initialize all system components
 *
 * @return true if all components initialized successfully
 *****************************************************************************/
static bool system_init(void)
{
  bool success = true;

  /* Initialize error handler first */
  error_handler_init();

#if ENABLE_DEBUG
  printf("\n");
  printf("========================================\n");
  printf("  E-Week 2026 Robot Controller\n");
  printf("  %s\n", BUILD_INFO_STRING);
  printf("----------------------------------------\n");
  printf("  Branch: %s\n", GIT_BRANCH);
#if GIT_IS_DIRTY
  printf("  Status: UNCOMMITTED CHANGES\n");
#endif
  printf("========================================\n\n");
#endif

#if ENABLE_WATCHDOG
  if (watchdog_caused_reboot())
  {
    ERROR_REPORT(ERROR_HW_WATCHDOG);
#if ENABLE_DISPLAY
    g_watchdogRebooted = true;
#endif
#if ENABLE_DEBUG
    printf("[WARN] System rebooted by watchdog!\n");
#endif
  }
#endif

  /* Initialize iBUS receiver */
#if ENABLE_DEBUG
  printf("[Init] Configuring iBUS receiver...\n");
#endif
  g_pIBus = new FlySkyIBus(uart1, PIN_IBUS_TX, PIN_IBUS_RX);
  if (g_pIBus == nullptr)
  {
    ERROR_REPORT(ERROR_IBUS_INIT_FAILED);
    success = false;
  }

  /* Initialize drive train */
#if ENABLE_DEBUG
  printf("[Init] Configuring Differential drive train...\n");
#endif

  g_pDriveTrain = new DriveTrainDifferential();
  init_motors(g_pDriveTrain);

  if ((g_pDriveTrain == nullptr) ||
      (g_pDriveTrain->isInitialized() == false))
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    success = false;
  }

  /* Initialize mechanisms */
#if ENABLE_DEBUG
  printf("[Init] Configuring scoop mechanism...\n");
#endif // ENABLE_DEBUG
  g_pScoop = new MechScoop();
  if (g_pScoop != nullptr)
  {
    g_pScoop->init();
  }

#if ENABLE_DEBUG
  printf("[Init] Configuring launcher mechanism...\n");
#endif
  g_pLauncher = new MechLauncher();
  if (g_pLauncher != nullptr)
  {
    g_pLauncher->init();
  }

  /* Initialize display view (constructed here, runs on core 1) */
#if ENABLE_DISPLAY
#if ENABLE_DEBUG
  printf("[Init] Constructing display view...\n");
#endif
  g_pDisplayView = new DisplayView(i2c0, PIN_DISPLAY_SDA, PIN_DISPLAY_SCL);
  if (g_pDisplayView == nullptr)
  {
#if ENABLE_DEBUG
    printf("[WARN] Display allocation failed\n");
#endif
  }
  else
  {
    /* Launch core 1 for display */
    multicore_launch_core1(core1_main);
#if ENABLE_DEBUG
    printf("[Init] Core 1 launched for display\n");
#endif
  }
#endif /* ENABLE_DISPLAY */

  /* Enable watchdog */
#if ENABLE_WATCHDOG
#if ENABLE_DEBUG
  printf("[Init] Enabling watchdog (timeout: %d ms)...\n", TIMING_WATCHDOG_TIMEOUT_MS);
#endif
  watchdog_enable(TIMING_WATCHDOG_TIMEOUT_MS, true);
#endif

#if ENABLE_DEBUG
  if (success)
  {
    printf("\n[Init] System initialization complete!\n\n");
  }
  else
  {
    printf("\n[Init] System initialization FAILED!\n\n");
  }
#endif

  return success;
}

/*******************************************************************************
 * @brief Process RC inputs and update drive train
 *
 * Called when new RC data is available.
 * 
 * Control Mapping
 * Right Stick Verical   - Throttle
 * Left Stick Horizontal - Turn
 * VRA                   - Forward steering trim
 * VRB                   - Steering multiplier
 ******************************************************************************/
static void process_rc_input(void)
{
  if ((g_pIBus == nullptr) || (g_pDriveTrain == nullptr))
  {
    return;
  }

  /* Read updated channel values */
  int speed     = g_pIBus->readChannel(FlySkyIBus::CHAN_RSTICK_VERT,  FlySkyIBus::READ_CHAN_CENTER_0); // [-500..500]
  int steer     = g_pIBus->readChannel(FlySkyIBus::CHAN_LSTICK_HORIZ, FlySkyIBus::READ_CHAN_CENTER_0); // [-500..500]
  int steerTrim = g_pIBus->readChannel(FlySkyIBus::CHAN_VRA,          FlySkyIBus::READ_CHAN_RAW);      // [1000..2000]
  int steerRate = g_pIBus->readChannel(FlySkyIBus::CHAN_VRB,          FlySkyIBus::READ_CHAN_NORM);     // [0..1000]

  /* Update modules with the new values */
  g_pDriveTrain->setSpeed(speed); // TODO: 2S governer

  float steerMultipler = static_cast<float>(steerRate) / static_cast<float>(1000.0); 
  int steerAdjusted = static_cast<int>(static_cast<float>(steer) * steerMultipler);
  g_pDriveTrain->setTurn(steerAdjusted);

  g_pDriveTrain->setForwardTrimFromChannel(steerTrim);
  g_pDriveTrain->setReverseTrimFromChannel(steerTrim);
  g_pDriveTrain->setManualTrimMode(true);

  g_pDriveTrain->update();
}

/*******************************************************************************
 * @brief Handle signal loss condition
 *
 * Stops all motors when RC signal is lost.
 ******************************************************************************/
static void handle_signal_loss(void)
{
#if ENABLE_SIGNAL_LOSS_CUTOFF
  if (g_pDriveTrain != nullptr)
  {
    g_pDriveTrain->stop();
  }

#if ENABLE_DEBUG
  static uint32_t lastWarnTime = 0;
  uint32_t now = to_ms_since_boot(get_absolute_time());
  if (now - lastWarnTime > TIMING_SIGNAL_LOSS_PRINT_MS)
  {
    printf("[WARN] RC signal lost!\n");
    lastWarnTime = now;
  }
#endif
#endif /* ENABLE_SIGNAL_LOSS_CUTOFF */
}

#if ENABLE_DISPLAY
/*******************************************************************************
 * @brief Helper to convert mechanism init state to display enum
 *
 * Since the mechanism classes are skeleton implementations with only an
 * initialized flag, we map that to IDLE or NOT_INIT. As mechanisms are
 * fleshed out with active/fault states, this function should be updated.
 *
 * @param initialized true if the mechanism has been initialized
 * @return Corresponding MechState_e value
 ******************************************************************************/
static DisplayView::MechState_e mechInitToState(bool initialized)
{
  return initialized ? DisplayView::MECH_IDLE : DisplayView::MECH_NOT_INIT;
}

/*******************************************************************************
 * @brief Populate and push display data to core 1
 *
 * Gathers telemetry from all subsystems and publishes it to the display
 * view via the thread-safe pushData() interface.
 ******************************************************************************/
static void update_display(void)
{
  if (g_pDisplayView == nullptr)
  {
    return;
  }

  DisplayView::DisplayData_t data = {};

  /* RC link status */
  if (g_pIBus != nullptr)
  {
    data.rcSignalValid  = g_pIBus->isSignalValid();
    data.rcTimeSinceMsg = g_pIBus->getTimeSinceLastMessage();
  }
  else
  {
    data.rcSignalValid  = false;
    data.rcTimeSinceMsg = 9999;
  }

  /* Drive train telemetry */
  if (g_pDriveTrain != nullptr)
  {
    data.speed         = static_cast<int16_t>(g_pDriveTrain->getSpeed());
    data.turn          = static_cast<int16_t>(g_pDriveTrain->getTurn());
    data.motorLeftPct  = static_cast<int16_t>(
      g_pDriveTrain->getMotorOutputPct(DriveTrainDifferential::MOTOR_LEFT));
    data.motorRightPct = static_cast<int16_t>(
      g_pDriveTrain->getMotorOutputPct(DriveTrainDifferential::MOTOR_RIGHT));
    data.trimFwd       = static_cast<int8_t>(g_pDriveTrain->getForwardTrimOffset());
    data.trimRev       = static_cast<int8_t>(g_pDriveTrain->getReverseTrimOffset());
    // TODO: Steering rate...
  }

  /* Mechanism states */
  data.scoopState = (g_pScoop != nullptr) ?
                     mechInitToState(g_pScoop->isInitialized()) :
                     DisplayView::MECH_NOT_INIT;

  data.launcherState = (g_pLauncher != nullptr) ?
                        mechInitToState(g_pLauncher->isInitialized()) :
                        DisplayView::MECH_NOT_INIT;

  /* System health */
  data.lastErrorCode  = static_cast<uint16_t>(error_get_last());
  data.errorCount     = error_get_count();
  data.watchdogReboot = g_watchdogRebooted;

  g_pDisplayView->pushData(data);
}
#endif /* ENABLE_DISPLAY */

/*******************************************************************************
 * @brief Application entry point
 *
 * Initializes the system and runs the main control loop.
 *
 * @return Exit code (should never return)
 ******************************************************************************/
int main(void)
{
  /* Initialize stdio for debug output */
  stdio_init_all();

#if ENABLE_STDIO_USB
  sleep_ms(TIMING_USB_STARTUP_DELAY_MS);
#endif

  if (!system_init())
  {
#if ENABLE_DEBUG
    printf("[ERROR] System initialization failed. Halting.\n");
#endif

    /* Blink LED to indicate error */
    gpio_init(PIN_LED_ONBOARD);
    gpio_set_dir(PIN_LED_ONBOARD, GPIO_OUT);

    while (true)
    {
      gpio_put(PIN_LED_ONBOARD, 1);
      sleep_ms(TIMING_ERROR_LED_BLINK_MS);
      gpio_put(PIN_LED_ONBOARD, 0);
      sleep_ms(TIMING_ERROR_LED_BLINK_MS);

#if ENABLE_WATCHDOG
      watchdog_update();
#endif
    }
  }

#if ENABLE_DEBUG
  printf("[Main] Entering main loop\n");
#endif

  /* Main control loop */
  while (true)
  {
    uint32_t loopStart = to_ms_since_boot(get_absolute_time());

#if ENABLE_WATCHDOG
    watchdog_update();
#endif

    /* Check for new RC data */
    if (g_pIBus != nullptr)
    {
      if (g_pIBus->hasNewMessage())
      {
        process_rc_input();
      }
      else if (!g_pIBus->isSignalValid())
      {
        handle_signal_loss();
      }
    }
    else
    {
      /* No iBus receiver - treat as signal loss */
      handle_signal_loss();
    }

    /* Update mechanisms */
    if (g_pScoop != nullptr)
    {
      g_pScoop->update();
    }

    if (g_pLauncher != nullptr)
    {
      g_pLauncher->update();
    }

    /* Push telemetry to display (core 1) */
#if ENABLE_DISPLAY
    update_display();
#endif

    /* Rate limiting */
#if TIMING_MAIN_LOOP_PERIOD_MS > 0
    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - loopStart;
    if (elapsed < TIMING_MAIN_LOOP_PERIOD_MS)
    {
      sleep_ms(TIMING_MAIN_LOOP_PERIOD_MS - elapsed);
    }
#endif
  }

  /* Should never reach here */
  return 0;
}


/* EOF -----------------------------------------------------------------------*/
