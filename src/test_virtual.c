// Virtual robot (env:virtual): motion.c replaced by the host simulator's 16x16 maze, in real time (tests saves).
#define SIM_ON_ROBOT
#include "../test/host/sim.c"

#include <string.h>
#include "stm32f1xx_hal.h"
#include "commands.h"
#include "gpio.h"
#include "health.h"
#include "motor.h"
#include "uart.h"

#define VIRTUAL_SEED    2u      // the maze: truth_generate() + truth_competition_goal()
#define VIRTUAL_OPENINGS 30u    // walls knocked out to make loops
#define WHEEL_RAMP_PWM  4       // PWM per ms: 0 to full in 0.25 s (the profiles' accel)

// ---- Wheels (TUNE RUEDAS 1, robot on a stand) -------------------------------------------

static uint8_t wheels;
static volatile int16_t wheel_target[2], wheel_pwm[2];  // MOTOR_R, MOTOR_L

// PWM for `speed` mm/s, from the motor model (robot_config.h).
static int16_t wheel_pwm_for(float speed){
    const float pwm = speed * (MOTOR_KV_L + MOTOR_KV_R) / 2.0f + MOTOR_KS_PWM;
    return (int16_t)(pwm < CONTROL_PWM_LIMIT ? pwm : CONTROL_PWM_LIMIT);
}

static void wheels_set(int16_t left, int16_t right){
    wheel_target[MOTOR_L] = left;
    wheel_target[MOTOR_R] = right;
}

// SysTick: each wheel's PWM ramps to its target.
void motion_tick_1ms(void){
    for(uint8_t m = 0; m < 2; m++){
        const int16_t pwm = wheel_pwm[m], target = wheel_target[m];
        if(pwm == target) continue;
        const int16_t step = (int16_t)(target - pwm);
        wheel_pwm[m] = (int16_t)(pwm + (step > WHEEL_RAMP_PWM ? WHEEL_RAMP_PWM
                                        : step < -WHEEL_RAMP_PWM ? -WHEEL_RAMP_PWM : step));
        motor_set((motor_t)m, wheel_pwm[m]);
    }
}

// ---- Run control (as motion.c) ---------------------------------------------------------

static volatile uint8_t abort_flag;
static volatile uint8_t paused;
static uint8_t step_mode;
static uint8_t moved;   // a move ran since the last checkpoint
static uint8_t maze_built;

void motion_request_abort(void){ abort_flag = 1; }
uint8_t motion_abort_requested(void){ return abort_flag; }
uint8_t motion_step_mode(void){ return step_mode; }
void motion_set_paused(uint8_t on){ paused = on; }

void motion_set_step_mode(uint8_t on){
    step_mode = on;
    if(!on) paused = 0;
}

// A new run (or its end): the virtual robot stands at the start.
void motion_clear_abort(void){
    wheels_set(0, 0);
    abort_flag = 0;
    paused = 0;
    moved = 0;
    if(!maze_built){
        truth_generate(VIRTUAL_SEED, VIRTUAL_OPENINGS);
        truth_competition_goal(VIRTUAL_SEED);
        maze_built = 1;
    }
    sim_reset(0.0, 1);
}

static void poll_inputs(void){
    health_alive();
    commands_poll();
    if(button_take_press(BUTTON_START)) abort_flag = 1;
}

uint8_t motion_wait(uint32_t ms){
    const uint32_t start = HAL_GetTick();
    while(HAL_GetTick() - start < ms){
        poll_inputs();
        if(abort_flag) return 0;
    }
    return 1;
}

uint8_t motion_checkpoint(void){
    poll_inputs();
    if(step_mode && moved && !abort_flag){
        paused = 1;
        print("-- step done: RESUME to go on --\n");
    }
    moved = 0;
    while(paused && !abort_flag) poll_inputs();
    return !abort_flag;
}

// The move's time, for real; STOP or START cut the wait, the run stops at the next checkpoint.
void virtual_elapse(double seconds, int16_t speed, int8_t turn){
    moved = 1;
    if(wheels && turn){
        // In place at TURN deg/s: each wheel travels TURNTICKS / 90 ticks a degree.
        const int16_t pwm = wheel_pwm_for(params.turn_speed * params.turn_ticks / 90.0f / WHEEL_TICKS_PER_MM);
        wheels_set(turn > 0 ? pwm : (int16_t)-pwm, turn > 0 ? (int16_t)-pwm : pwm);
    }
    else if(wheels && speed){
        const int16_t pwm = wheel_pwm_for(speed);
        wheels_set(pwm, pwm);
    }
    motion_wait((uint32_t)(seconds * 1000.0));
    wheels_set(0, 0);
}

void motion_indicate(indication_t what){
    switch(what){
        case IND_GOAL:
            leds_all(1);
            motion_wait(200);
            leds_all(0);
            break;
        case IND_DONE:
            leds_blink(3, 150);
            break;
        case IND_FAIL:
            leds_blink(3, 60);
            break;
    }
}

// ---- Robot only: nothing to drive -------------------------------------------------------

void motion_stop(void){ wheels_set(0, 0); }
void motion_leg_timing_reset(void){}
void motion_leg_timing_report(void){}
float motion_ticks_per_mm(void){ return WHEEL_TICKS_PER_MM; }
void motion_curve_info(uint8_t line){ (void)line; }

void motion_reference(float *fwd_mm, float *rot_deg, uint8_t *move_id){
    *fwd_mm = 0.0f;
    *rot_deg = 0.0f;
    *move_id = 0;
}

// ---- Flash stress (TUNE STRESS n, TUNE REVERSAL n; robot on a stand): tries to wedge the flash (docs/freezes.md) --
#define STRESS_PAGE     0x0800F000u     // pages 60-61, under the store (0x0800F800)
#define STRESS_ROUND_HW 64u             // halfwords a round: 8 rounds fill a page
#define STRESS_SLOW_US  1000u           // a halfword normally takes ~56 us
#define STRESS_SLOW_MS  200u            // an erase ~22 ms
#define STRESS_PWM      700

extern uint32_t _sidata, _sdata, _edata;    // linker: the image ends at _sidata + .data

static const char *const STRESS_KIND[4] = {"turning", "after braking", "reversing", "turning in place"};

static void wheels_now(int16_t left, int16_t right){
    wheel_target[MOTOR_L] = wheel_pwm[MOTOR_L] = left;
    wheel_target[MOTOR_R] = wheel_pwm[MOTOR_R] = right;
    motor_set(MOTOR_L, left);
    motor_set(MOTOR_R, right);
}

static uint32_t stress_us(uint32_t t0){
    return (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);
}

static uint8_t hsi_kept_on;     // TUNE HSI 1

// The CPU runs on the HSI (the crystal failed): it is left alone then.
static uint8_t hsi_runs_cpu(void){
    const uint32_t cfgr = RCC->CFGR, sws = cfgr & RCC_CFGR_SWS;
    return sws == RCC_CFGR_SWS_HSI || (sws == RCC_CFGR_SWS_PLL && !(cfgr & RCC_CFGR_PLLSRC));
}

static void hsi_wait_ready(uint32_t ready){
    for(uint32_t n = 0; ((RCC->CR & RCC_CR_HSIRDY) != 0) != ready && n < 200000u; n++){}
}

// Before a scratch page operation: a freshly started HSI, as flash_store.c does.
static void stress_hsi_start(void){
    if(hsi_kept_on || hsi_runs_cpu()) return;
    RCC->CR &= ~RCC_CR_HSION;
    hsi_wait_ready(0);
    RCC->CR |= RCC_CR_HSION;
    hsi_wait_ready(1);
}

static void stress_hsi_stop(void){
    if(!hsi_kept_on && !hsi_runs_cpu()) RCC->CR &= ~RCC_CR_HSION;
}

static void stress_report(const char *what, uint32_t round, uint32_t value){
    print("!! stress: %s in round %lu (%s): %lu | RCC_CR=%08lx SR=%02lx CR=%04lx\n", what,
          (unsigned long)round, STRESS_KIND[round % 4u], (unsigned long)value, (unsigned long)RCC->CR,
          (unsigned long)FLASH->SR, (unsigned long)FLASH->CR);
}

static void flash_stress(uint32_t rounds){
    const uint32_t image_end = (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata);
    if(!wheels || image_end > STRESS_PAGE || rounds == 0){
        print("stress: TUNE WHEELS 1 first (robot on a stand); TUNE STRESS rounds\n");
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    abort_flag = 0;
    uint32_t worst_us = 0, worst_round = 0, sum_us = 0, count = 0, worst_erase_ms = 0;
    uint8_t failed = 0;
    for(uint32_t r = 0; r < rounds && !failed && !abort_flag; r++){
        const uint32_t page = STRESS_PAGE + ((r / 8u) % 2u) * FLASH_PAGE_SIZE;
        const uint32_t base = page + (r % 8u) * STRESS_ROUND_HW * 2u;
        const uint8_t kind = (uint8_t)(r % 4u);
        if(kind == 3) wheels_now(STRESS_PWM, -STRESS_PWM);
        else wheels_now(STRESS_PWM, STRESS_PWM);
        motion_wait(300);
        if(kind == 1) wheels_now(0, 0);
        else if(kind == 2) wheels_now(-STRESS_PWM, -STRESS_PWM);
        stress_hsi_start();
        if(r % 8u == 0){
            // A new page: erased with the wheels stopped (a wedged erase stalls the CPU ~200 s).
            wheels_now(0, 0);
            FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_PAGES, .Banks = FLASH_BANK_1,
                                            .PageAddress = page, .NbPages = 1};
            uint32_t page_error = 0;
            const uint32_t t0 = DWT->CYCCNT, tick0 = HAL_GetTick();
            HAL_FLASH_Unlock();
            const uint8_t ok = HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
            HAL_FLASH_Lock();
            const uint32_t ms = stress_us(t0) / 1000u;
            if(ms > worst_erase_ms) worst_erase_ms = ms;
            if(!ok || ms > STRESS_SLOW_MS || HAL_GetTick() - tick0 > STRESS_SLOW_MS){
                stress_report(ok ? "slow erase (ms)" : "failed erase (ms)", r, ms);
                failed = 1;
                break;
            }
        }
        HAL_FLASH_Unlock();
        for(uint32_t i = 0; i < STRESS_ROUND_HW && !failed; i++){
            const uint16_t value = (uint16_t)((r * STRESS_ROUND_HW + i) ^ 0xA5A5u);
            const uint32_t t0 = DWT->CYCCNT;
            const uint8_t ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + 2u * i, value) == HAL_OK;
            const uint32_t us = stress_us(t0);
            sum_us += us;
            count++;
            if(us > worst_us){
                worst_us = us;
                worst_round = r;
            }
            if(ok && us <= STRESS_SLOW_US) continue;
            stress_report(ok ? "slow write (us)" : "failed write (us)", r, us);
            failed = 1;
            // Still slow 1 s later, the wheels stopped? The HSI is left alone.
            HAL_FLASH_Lock();
            wheels_now(0, 0);
            motion_wait(1000);
            if(i + 1u < STRESS_ROUND_HW){
                HAL_FLASH_Unlock();
                const uint32_t t1 = DWT->CYCCNT;
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + 2u * (i + 1u), 0x1234u);
                stress_report("1 s later, another write (us)", r, stress_us(t1));
            }
        }
        HAL_FLASH_Lock();
        stress_hsi_stop();
        wheels_now(0, 0);
        const volatile uint16_t *hw = (const volatile uint16_t *)base;
        for(uint32_t i = 0; i < STRESS_ROUND_HW && !failed; i++){
            if(hw[i] == (uint16_t)((r * STRESS_ROUND_HW + i) ^ 0xA5A5u)) continue;
            stress_report("data written wrong (index)", r, i);
            break;
        }
        if((r + 1u) % 25u == 0 || r + 1u == rounds){
            print("stress %lu/%lu: worst %lu us (round %lu, %s), mean %lu us, worst erase %lu ms\n",
                  (unsigned long)(r + 1u), (unsigned long)rounds, (unsigned long)worst_us,
                  (unsigned long)worst_round, STRESS_KIND[worst_round % 4u],
                  (unsigned long)(sum_us / count), (unsigned long)worst_erase_ms);
        }
        motion_wait(100);
    }
    wheels_now(0, 0);
    stress_hsi_stop();      // after an erase that failed, still on
    print("stress: %s\n", failed ? "STOPPED by a failure (power cycle if the flash stayed slow)"
                                  : abort_flag ? "stopped with START" : "finished with no failure");
    abort_flag = 0;
}

static void reversal_stress(uint32_t blocks){
    const uint32_t page = STRESS_PAGE + FLASH_PAGE_SIZE;
    if(!wheels || blocks == 0 || blocks > FLASH_PAGE_SIZE / 2u){
        print("reversal: TUNE WHEELS 1 first (robot on a stand); TUNE REVERSAL blocks (1-512)\n");
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    abort_flag = 0;
    wheels_now(0, 0);
    FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_PAGES, .Banks = FLASH_BANK_1,
                                    .PageAddress = page, .NbPages = 1};
    uint32_t page_error = 0;
    stress_hsi_start();
    const uint32_t t0 = DWT->CYCCNT;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    stress_hsi_stop();
    const uint32_t erase_ms = stress_us(t0) / 1000u;
    if(erase_ms > STRESS_SLOW_MS){
        stress_report("slow erase before (ms)", 0, erase_ms);
        return;
    }
    uint32_t worst_us = 0;
    uint8_t failed = 0;
    for(uint32_t b = 0; b < blocks && !failed && !abort_flag; b++){
        for(uint8_t k = 0; k < 10u; k++){
            wheels_now(STRESS_PWM, STRESS_PWM);
            motion_wait(150);
            wheels_now(-STRESS_PWM, -STRESS_PWM);
            motion_wait(150);
        }
        wheels_now(0, 0);
        motion_wait(1000);
        stress_hsi_start();
        HAL_FLASH_Unlock();
        const uint32_t t1 = DWT->CYCCNT;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page + 2u * b, (uint16_t)(b ^ 0x5A5Au));
        const uint32_t us = stress_us(t1);
        HAL_FLASH_Lock();
        stress_hsi_stop();
        if(us > worst_us) worst_us = us;
        if(us > STRESS_SLOW_US){
            print("!! reversal: slow write after %lu blocks of 10 reversals: %lu us | RCC_CR=%08lx\n",
                  (unsigned long)(b + 1u), (unsigned long)us, (unsigned long)RCC->CR);
            failed = 1;
        }
        else if((b + 1u) % 10u == 0 || b + 1u == blocks){
            print("reversal %lu/%lu: worst %lu us\n", (unsigned long)(b + 1u), (unsigned long)blocks,
                  (unsigned long)worst_us);
        }
    }
    wheels_now(0, 0);
    print("reversal: %s\n", failed ? "STOPPED: the flash went slow with no write while they turned"
                                     : abort_flag ? "stopped with START" : "finished with no failure");
    abort_flag = 0;
}

void motion_tune_list(void){
    print("wheels %s (TUNE WHEELS 1: they turn with every move, robot on a stand only; 0: still)\n",
          wheels ? "TURNING" : "still");
    print("HSI %s (TUNE HSI 1: always on, as before 10-04; 0: for the flash only)\n",
          hsi_kept_on ? "ALWAYS ON" : "for the flash only");
}

void motion_tune_set(const char *name, float value){
    if(strcmp(name, "STRESS") == 0){
        flash_stress(value > 0.0f ? (uint32_t)value : 0u);
        return;
    }
    if(strcmp(name, "REVERSAL") == 0){
        reversal_stress(value > 0.0f ? (uint32_t)value : 0u);
        return;
    }
    if(strcmp(name, "HSI") == 0){
        hsi_kept_on = 0;
        stress_hsi_stop();      // also the cure of a crawling HSI, by hand
        if(value != 0.0f){
            RCC->CR |= RCC_CR_HSION;
            hsi_wait_ready(1);
            hsi_kept_on = 1;
        }
    }
    if(strcmp(name, "WHEELS") == 0){
        wheels = value != 0.0f;
        if(!wheels) wheels_set(0, 0);
    }
    motion_tune_list();
}

move_result_t motion_drive_straight(int16_t speed, int32_t mm){
    (void)speed;
    (void)mm;
    return MOVE_ABORTED;
}
