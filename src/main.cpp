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
 * - Mechanisms: Collection, Deposit, Launch
 * - Safety: Watchdog timer, signal loss detection
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

// Project headers
#include "config.h"
#include "pinout.h"
#include "version.h"
#include "error_handler.h"
#include "flysky_ibus.h"
#include "drive_train_differential.h"
#include "mech_collect.h"
#include "mech_deposit.h"
#include "mech_launcher.h"


/* Globals -------------------------------------------------------------------*/

/** @brief RC receiver interface */
static FlySkyIBus* g_pIBus = nullptr;

/** @brief Drive train controller */
static DriveTrainDifferential* g_pDriveTrain = nullptr;

/** @brief Collection mechanism */
static MechCollect*  g_pCollect = nullptr;

/** @brief Deposit mechanism */
static MechDeposit*  g_pDeposit = nullptr;

/** @brief Launcher mechanism */
static MechLauncher* g_pLauncher = nullptr;


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
  printf("[Init] Configuring collection mechanism...\n");
#endif
  g_pCollect = new MechCollect();
  if (g_pCollect != nullptr)
  {
    g_pCollect->init();
  }

#if ENABLE_DEBUG
  printf("[Init] Configuring deposit mechanism...\n");
#endif
  g_pDeposit = new MechDeposit();
  if (g_pDeposit != nullptr)
  {
    g_pDeposit->init();
  }

#if ENABLE_DEBUG
  printf("[Init] Configuring launcher mechanism...\n");
#endif
  g_pLauncher = new MechLauncher();
  if (g_pLauncher != nullptr)
  {
    g_pLauncher->init();
  }

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
 ******************************************************************************/
static void process_rc_input(void)
{
  if ((g_pIBus == nullptr) || (g_pDriveTrain == nullptr))
  {
    return;
  }

  /* Read normalized channel values [-500..500] */
  int speed = g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_RSTICK_VERT);
  int turn  = g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_RSTICK_HORIZ);

  g_pDriveTrain->setSpeed(speed);

  // Use SWC to determine what divider to apply to steering
  if (g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_SWC) < 250)
  {
    // Switch is in UP position, do nothing to steering
  }
  else if (g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_SWC) > 250)
  {
    // Switch is in DOWN position, Max attenuation of steering
    float newTurn = static_cast<float>(turn) / 3.0f;
    turn = static_cast<int>(newTurn);
  }
  else
  {
    // Switch is in middle position, Use medium attenuation
    float newTurn = static_cast<float>(turn) / 2.0f;
    turn = static_cast<int>(newTurn);
  }
  g_pDriveTrain->setTurn(turn);

  // Use manual trim by default, use SWA to select auto trim...eventually
  if (g_pIBus->readChannel(FlySkyIBus::CHAN_SWA) < FlySkyIBus::CHANNEL_VALUE_CENTER)
  {
    /* Read VRA/VRB for manual trim adjustment (raw values 1000-2000) */
    int vraValue = g_pIBus->readChannel(FlySkyIBus::CHAN_VRA);
    int vrbValue = g_pIBus->readChannel(FlySkyIBus::CHAN_VRB);

    g_pDriveTrain->setForwardTrimFromChannel(vraValue);
    g_pDriveTrain->setReverseTrimFromChannel(vrbValue);
    g_pDriveTrain->setManualTrimMode(true);
  }
  else
  {
    /* Switch is Down, use automatic trim from calibration */
    g_pDriveTrain->setManualTrimMode(false);
  }

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
    if (g_pCollect != nullptr)
    {
      g_pCollect->update();
    }

    if (g_pDeposit != nullptr)
    {
      g_pDeposit->update();
    }

    if (g_pLauncher != nullptr)
    {
      g_pLauncher->update();
    }

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
