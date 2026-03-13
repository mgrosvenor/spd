#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>

static int g_tests_run    = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define CHECK(cond) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        g_tests_failed++; \
        fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ(a, b) CHECK((a) == (b))

#define TEST_SUMMARY() do { \
    printf("Tests: %d run, %d passed, %d failed\n", \
           g_tests_run, g_tests_passed, g_tests_failed); \
    return (g_tests_failed > 0) ? 1 : 0; \
} while (0)

#endif /* TEST_FRAMEWORK_H */
