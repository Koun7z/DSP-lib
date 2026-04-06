#include "test_registry.h"

#if defined(__cplusplus)
extern "C" {
#endif


#include <check.h>
#include <stdlib.h>

int main()
{
    Suite** suites     = tr_get_registered_suites();
    size_t suite_count = tr_get_registered_suite_count();

    SRunner* runner = srunner_create(suites[0]);
    for(size_t i = 1; i < suite_count; i++)
    {
        srunner_add_suite(runner, suites[i]);
    }

    srunner_run_all(runner, CK_VERBOSE);
    int number_failed = srunner_ntests_failed(runner);
    srunner_free(runner);

    free(suites);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}

#if defined(__cplusplus)
}
#endif