/* Минимальная обвязка для тестов: без внешних зависимостей, чтобы хостовая
 * сборка оставалась однокомандной и не тянула фреймворков. */
#ifndef IV_TEST_H
#define IV_TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int iv_test_failures = 0;

#define IV_CHECK(cond)                                                        \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("  ПРОВАЛ %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            iv_test_failures++;                                               \
        }                                                                     \
    } while (0)

#define IV_CHECK_NEAR(a, b, eps)                                              \
    do {                                                                      \
        double _a = (a), _b = (b);                                            \
        if (fabs(_a - _b) > (eps)) {                                          \
            printf("  ПРОВАЛ %s:%d: %s=%g, ожидалось %g (±%g)\n",             \
                   __FILE__, __LINE__, #a, _a, _b, (double)(eps));            \
            iv_test_failures++;                                               \
        }                                                                     \
    } while (0)

#define IV_TEST_END()                                                         \
    do {                                                                      \
        if (iv_test_failures) {                                               \
            printf("%s: провалов %d\n", __FILE__, iv_test_failures);          \
            return EXIT_FAILURE;                                              \
        }                                                                     \
        printf("%s: ок\n", __FILE__);                                         \
        return EXIT_SUCCESS;                                                  \
    } while (0)

#endif /* IV_TEST_H */
