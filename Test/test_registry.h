#ifndef TEST_REGISTRY_H__
#define TEST_REGISTRY_H__

#include <check.h>

#if defined(__cplusplus)
extern "C" {
#endif

void tr_add_test(const char* tsuite, const char* tcase, const TTest* tf);
Suite** tr_get_registered_suites(void);
size_t tr_get_registered_suite_count(void);

#if defined(__cplusplus)
}
#endif

#endif /* TEST_REGISTRY_H__ */