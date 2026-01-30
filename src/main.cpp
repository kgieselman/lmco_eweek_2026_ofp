/*******************************************************************************
 * @file main.cpp
 * @brief Main application for E-Week 2026 robot control
 *
 * This is the entry point for the robot control system. It initializes all
 * subsystems, configures the watchdog, and runs the main control loop.
 *
 * @par System Overview:
 * - RC Receiver: FlySky iBUS protocol on UART1
 * - Drive Train: Differential or Mecanum (configurable)
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
#include "error_handler.h"
#include "flysky_ibus.h"
#include "mech_collect.h"
#include "mech_deposit.h"
#include "mech_launcher.h"

// Include appropriate drive train based on configuration
#if USE_DRIVE_TRAIN_MECANUM
#include "drive_train_mecanum.h"
#else
#include "drive_train_differential.h"
#endif


/* Globals -------------------------------------------------------------------*/

/** @brief RC receiver interface */
static FlySkyIBus* g_pIBus = nullptr;

/** @brief Drive train controller */
#if USE_DRIVE_TRAIN_MECANUM
static DriveTrainMecanum* g_pDriveTrain = nullptr;
#else
static DriveTrainDifferential* g_pDriveTrain = nullptr;
#endif

/** @brief Collection mechanism */
static MechCollect* g_pCollect = nullptr;

/** @brief Deposit mechanism */
static MechDeposit* g_pDeposit = nullptr;

/** @brief Launcher mechanism */
static MechLauncher* g_pLauncher = nullptr;


/* Private Function Definitions ----------------------------------------------*/

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
  printf("\n========================================\n");
  printf(  "  E-Week 2026 Robot Controller\n");
  printf(  "========================================\n\n");
#endif

  /* Check if this is a watchdog reboot */
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
#if USE_DRIVE_TRAIN_MECANUM
  printf("[Init] Configuring Mecanum drive train...\n");
#else
  printf("[Init] Configuring Differential drive train...\n");
#endif
#endif

#if USE_DRIVE_TRAIN_MECANUM
  g_pDriveTrain = new DriveTrainMecanum();
  if (g_pDriveTrain != nullptr)
  {
    g_pDriveTrain->addMotor(DriveTrainMecanum::MOTOR_FRONT_LEFT,
                            PIN_MECANUM_MOTOR_FL_PWM,
                            PIN_MECANUM_MOTOR_FL_DIR_FWD,
                            PIN_MECANUM_MOTOR_FL_DIR_REV);
    g_pDriveTrain->addMotor(DriveTrainMecanum::MOTOR_FRONT_RIGHT,
                            PIN_MECANUM_MOTOR_FR_PWM,
                            PIN_MECANUM_MOTOR_FR_DIR_FWD,
                            PIN_MECANUM_MOTOR_FR_DIR_REV);
    g_pDriveTrain->addMotor(DriveTrainMecanum::MOTOR_REAR_RIGHT,
                            PIN_MECANUM_MOTOR_RR_PWM,
                            PIN_MECANUM_MOTOR_RR_DIR_FWD,
                            PIN_MECANUM_MOTOR_RR_DIR_REV);
    g_pDriveTrain->addMotor(DriveTrainMecanum::MOTOR_REAR_LEFT,
                            PIN_MECANUM_MOTOR_RL_PWM,
                            PIN_MECANUM_MOTOR_RL_DIR_FWD,
                            PIN_MECANUM_MOTOR_RL_DIR_REV);
  }
#else
  g_pDriveTrain = new DriveTrainDifferential();
  if (g_pDriveTrain != nullptr)
  {
    g_pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_LEFT,
                            PIN_DIFF_MOTOR_LEFT_PWM,
                            PIN_DIFF_MOTOR_LEFT_DIR_FWD,
                            PIN_DIFF_MOTOR_LEFT_DIR_REV,
                            PIN_DIFF_MOTOR_LEFT_ENC);
    g_pDriveTrain->addMotor(DriveTrainDifferential::MOTOR_RIGHT,
                            PIN_DIFF_MOTOR_RIGHT_PWM,
                            PIN_DIFF_MOTOR_RIGHT_DIR_FWD,
                            PIN_DIFF_MOTOR_RIGHT_DIR_REV,
                            PIN_DIFF_MOTOR_RIGHT_ENC);
  }
#endif

  if ((g_pDriveTrain == nullptr) || 
      (g_pDriveTrain->isInitialized() == false))
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    success = false;
  }

  /* Initialize mechanisms */
#if ENABLE_MECH_COLLECT
#if ENABLE_DEBUG
  printf("[Init] Configuring collection mechanism...\n");
#endif
  g_pCollect = new MechCollect();
  if (g_pCollect != nullptr)
  {
    g_pCollect->init();
  }
#endif

#if ENABLE_MECH_DEPOSIT
#if ENABLE_DEBUG
  printf("[Init] Configuring deposit mechanism...\n");
#endif
  g_pDeposit = new MechDeposit();
  if (g_pDeposit != nullptr)
  {
    g_pDeposit->init();
  }
#endif

#if ENABLE_MECH_LAUNCHER
#if ENABLE_DEBUG
  printf("[Init] Configuring launcher mechanism...\n");
#endif
  g_pLauncher = new MechLauncher();
  if (g_pLauncher != nullptr)
  {
    g_pLauncher->init();
  }
#endif

  /* Enable watchdog */
#if ENABLE_WATCHDOG
#if ENABLE_DEBUG
  printf("[Init] Enabling watchdog (timeout: %d ms)...\n", WATCHDOG_TIMEOUT_MS);
#endif
  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
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

  /* Read normalized channel values (-500 to +500) */
  int speed = g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_RSTICK_VERT);
  int turn  = g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_RSTICK_HORIZ);

  /* Apply to drive train */
  g_pDriveTrain->setSpeed(speed);
  g_pDriveTrain->setTurn(turn);

#if USE_DRIVE_TRAIN_MECANUM
  int strafe = g_pIBus->readChannelNormalized(FlySkyIBus::CHAN_LSTICK_HORIZ);
  static_cast<DriveTrainMecanum*>(g_pDriveTrain)->setStrafe(strafe);
#endif

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
  if (now - lastWarnTime > 1000) // TODO: This should be a constant/define
  {
    printf("[WARN] RC signal lost!\n");
    lastWarnTime = now;
  }
#endif
#endif /* ENABLE_SIGNAL_LOSS_CUTOFF */
}

/**
 * @brief Update all mechanisms
 */
static void update_mechanisms(void)
{
#if ENABLE_MECH_COLLECT
  if (g_pCollect != nullptr)
  {
    g_pCollect->update();
  }
#endif

#if ENABLE_MECH_DEPOSIT
  if (g_pDeposit != nullptr)
  {
    g_pDeposit->update();
  }
#endif

#if ENABLE_MECH_LAUNCHER
  if (g_pLauncher != nullptr)
  {
    g_pLauncher->update();
  }
#endif
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

  /* Brief delay for USB enumeration (if enabled) */
#if ENABLE_STDIO_USB
  sleep_ms(2000); //TODO: This should be a constant/define
#endif

  /* Initialize all subsystems */
  if (!system_init())
  {
    /* Initialization failed - enter error state */
#if ENABLE_DEBUG
    printf("[ERROR] System initialization failed. Halting.\n");
#endif

    /* Blink LED to indicate error */
    gpio_init(PIN_LED_ONBOARD);
    gpio_set_dir(PIN_LED_ONBOARD, GPIO_OUT);

    while (true)
    {
      gpio_put(PIN_LED_ONBOARD, 1);
      sleep_ms(100);
      gpio_put(PIN_LED_ONBOARD, 0);
      sleep_ms(100);

#if ENABLE_WATCHDOG
      watchdog_update();
#endif
    }
  }

#if ENABLE_DEBUG
  printf("[Main] Entering main loop\n");
#endif

  /* Track loop timing */
  uint32_t lastLoopTime = to_ms_since_boot(get_absolute_time());

  /* Main control loop */
  while (true)
  {
    uint32_t loopStart = to_ms_since_boot(get_absolute_time());

    /* Feed the watchdog */
#if ENABLE_WATCHDOG
    watchdog_update();
#endif

    /* Check for new RC data */
    if (g_pIBus != nullptr && g_pIBus->hasNewMessage())
    {
      /* Process RC inputs */
      process_rc_input();
    }
    else if (!g_pIBus->isSignalValid())
    {
      /* Handle signal loss */
      handle_signal_loss();
    }

    /* Update mechanisms */
    update_mechanisms();

    /* Rate limiting */
#if MAIN_LOOP_PERIOD_MS > 0
    uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - loopStart;
    if (elapsed < MAIN_LOOP_PERIOD_MS)
    {
      sleep_ms(MAIN_LOOP_PERIOD_MS - elapsed);
    }
#endif

    lastLoopTime = loopStart;
  }

  /* Should never reach here */
  return 0;
}

/* EOF -----------------------------------------------------------------------*/
