#include "test_registry.h"

#include <map>
#include <string>
#include <cstdlib>

std::map<std::string, Suite*> suite_registry;
std::map<std::string, TCase*> tcase_registry;

void tr_add_test(const char* tsuite, const char* tcase, const TTest* tf)
{
    if(suite_registry.find(tsuite) == suite_registry.end())
    {
        suite_registry[tsuite] = suite_create(tsuite);
    }
    Suite* s = suite_registry[tsuite];

    TCase* tc = nullptr;
    if(tcase_registry.find(tcase) == tcase_registry.end())
    {
        tcase_registry[tcase] = tcase_create(tcase);
        tc                    = tcase_registry[tcase];
        suite_add_tcase(s, tc);
    }
    else
    {
        tc = tcase_registry[tcase];
    }

    tcase_add_test(tc, tf);
}

Suite** tr_get_registered_suites(void)
{
    size_t count  = tr_get_registered_suite_count();
    auto** suites = new Suite*[count];

    size_t i = 0;
    for(auto& pair : suite_registry)
    {
        suites[i++] = pair.second;
    }
    return suites;
}

size_t tr_get_registered_suite_count(void)
{
    return suite_registry.size();
}