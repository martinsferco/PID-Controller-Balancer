/**
  ******************************************************************************
  * @file    selftest.c
  * @brief   Casos de prueba dummy para Kalman y PID (on-target, opt-in).
  *          Todo el archivo queda vacio si APP_RUN_SELFTESTS == 0.
  ******************************************************************************
  */

#include "app_config.h"

#if APP_RUN_SELFTESTS

#include "selftest.h"
#include "kalman.h"
#include "pid.h"
#include <stdio.h>

/* --- Utilitarios (sin %f: newlib-nano no lo imprime por defecto) --- */

static float f_abs(float x) { return (x < 0.0f) ? -x : x; }

static void print_f(float v)
{
    if (v < 0.0f) { printf("-"); v = -v; }
    int ip = (int)v;
    int fp = (int)((v - (float)ip) * 1000.0f + 0.5f);
    if (fp >= 1000) { ip += 1; fp -= 1000; }
    printf("%d.%03d", ip, fp);
}

static int g_pass = 0, g_fail = 0;

static void check(const char *name, int cond, float val)
{
    if (cond) { g_pass++; } else { g_fail++; }
    printf("[%s] %s (", cond ? "PASS" : "FAIL", name);
    print_f(val);
    printf(")\r\n");
}

/* ============================ KALMAN ============================ */

static void test_kalman(void)
{
    printf("\r\n--- Kalman ---\r\n");

    /* 1) Convergencia con entrada constante. */
    {
        Kalman_t kf; Kalman_Init(&kf, 0.1f, 0.01f, 0.09f, 0.0f);
        float est = 0.0f;
        for (int i = 0; i < 60; i++) { est = Kalman_Update(&kf, 10.0f); }
        check("convergencia z=10", f_abs(est - 10.0f) < 0.5f, est);
    }

    /* 2) Atenuacion de un outlier. */
    {
        Kalman_t kf; Kalman_Init(&kf, 0.1f, 0.01f, 0.09f, 10.0f);
        for (int i = 0; i < 40; i++) { (void)Kalman_Update(&kf, 10.0f); }
        float est = Kalman_Update(&kf, 30.0f);   /* pico brusco */
        check("outlier atenuado (<15)", est < 15.0f, est);
    }

    /* 3) Seguimiento de rampa (z sube 1.0 por paso). */
    {
        Kalman_t kf; Kalman_Init(&kf, 0.1f, 0.5f, 0.09f, 0.0f);
        float z = 0.0f, est = 0.0f;
        for (int i = 0; i < 60; i++) { z += 1.0f; est = Kalman_Update(&kf, z); }
        check("rampa: sigue a z", f_abs(est - z) < 1.5f, est);
    }

    /* 4) R grande = mas suave; R chico = mas responsivo (ante un escalon). */
    {
        Kalman_t soft; Kalman_Init(&soft, 0.1f, 0.01f, 5.0f,  0.0f);
        Kalman_t fast; Kalman_Init(&fast, 0.1f, 0.01f, 0.02f, 0.0f);
        float es = 0.0f, ef = 0.0f;
        for (int i = 0; i < 5; i++) { es = Kalman_Update(&soft, 10.0f);
                                      ef = Kalman_Update(&fast, 10.0f); }
        /* el de R chico debe estar mas cerca del nuevo valor (10) */
        check("R chico mas responsivo", (10.0f - ef) < (10.0f - es), ef - es);
    }
}

/* ============================== PID ============================== */

static void test_pid(void)
{
    printf("\r\n--- PID ---\r\n");

    /* 1) Solo-P exacto: kp=2, err=6 -> 12. */
    {
        PID_t p; PID_Init(&p, 2.0f, 0.0f, 0.0f, 0.1f);
        float u = PID_Compute(&p, 10.0f, 4.0f);
        check("solo-P (=12)", f_abs(u - 12.0f) < 1e-3f, u);
    }

    /* 2) Saturacion: error enorme -> satura al maximo. */
    {
        PID_t p; PID_Init(&p, 100.0f, 0.0f, 0.0f, 0.1f);
        PID_SetLimits(&p, -20.0f, 20.0f);
        float u = PID_Compute(&p, 100.0f, 0.0f);
        check("satura a out_max (=20)", f_abs(u - 20.0f) < 1e-3f, u);
    }

    /* 3) Sin derivative-kick: escalon de setpoint con meas constante -> D no dispara. */
    {
        PID_t p; PID_Init(&p, 0.0f, 0.0f, 5.0f, 0.1f);
        (void)PID_Compute(&p, 0.0f, 3.0f);   /* asienta prev_meas */
        float u = PID_Compute(&p, 50.0f, 3.0f); /* salta setpoint, meas igual */
        check("sin derivative-kick (~0)", f_abs(u) < 1e-3f, u);
    }

    /* 4) Anti-windup: sale antes de saturacion que la variante sin AW. */
    {
        PID_t aw;   PID_Init(&aw,   1.0f, 2.0f, 0.0f, 0.1f);
        PID_SetLimits(&aw, -10.0f, 10.0f);
        PID_t noaw; PID_Init(&noaw, 1.0f, 2.0f, 0.0f, 0.1f);
        PID_SetLimits(&noaw, -10.0f, 10.0f);
        noaw.anti_windup = 0;

        /* Fase de windup: error grande positivo, saturado, muchos pasos. */
        for (int i = 0; i < 60; i++) {
            (void)PID_Compute(&aw,   100.0f, 0.0f);
            (void)PID_Compute(&noaw, 100.0f, 0.0f);
        }
        /* Invertir el error: ahora el setpoint es negativo. */
        float u_aw = 0.0f, u_no = 0.0f;
        for (int i = 0; i < 5; i++) {
            u_aw = PID_Compute(&aw,   -100.0f, 0.0f);
            u_no = PID_Compute(&noaw, -100.0f, 0.0f);
        }
        /* El de AW ya debe haber bajado; el sin AW sigue arrastrando integral. */
        check("anti-windup sale antes", u_aw < u_no, u_no - u_aw);
    }

    /* 5) Lazo simulado: planta de juguete pos += k*u*dt converge al setpoint. */
    {
        PID_t p; PID_Init(&p, 1.2f, 0.4f, 0.15f, 0.1f);
        PID_SetLimits(&p, -30.0f, 30.0f);
        const float sp = 12.0f, k = 0.8f, dt = 0.1f;
        float pos = 0.0f;
        for (int i = 0; i < 400; i++) {
            float u = PID_Compute(&p, sp, pos);
            pos += k * u * dt;
        }
        check("lazo simulado converge", f_abs(pos - sp) < 0.5f, pos);
    }
}

void SelfTest_Run(void)
{
    g_pass = 0; g_fail = 0;
    printf("\r\n===== SELF-TESTS =====\r\n");
    test_kalman();
    test_pid();
    printf("\r\n===== RESULTADO: %d PASS, %d FAIL =====\r\n", g_pass, g_fail);
}

#endif /* APP_RUN_SELFTESTS */
