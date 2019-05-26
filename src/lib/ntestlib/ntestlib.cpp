#include "ntestlib.h"
#include <cstring>

TestBase::Result TestBase::run()
{
	pthread_t runner_thread;

	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, PX4_STACK_ADJUSTED(1000));
	pthread_create(&runner_thread, &attr, TestBase::run_trampoline, this);
	pthread_attr_destroy(&attr);

	pthread_join(runner_thread, NULL);

	return _result;
}

void *TestBase::run_trampoline(void *self)
{
	reinterpret_cast<TestBase*>(self)->run_testbody();
	return nullptr;
}

void TestBase::abort_test()
{
	pthread_exit(NULL);
}

const char *TestBase::extractFilename(const char *path)
{
	const char *pos = strrchr (path, '/');
	if (!pos) {
		return nullptr;
	}

	return pos + 1;
}

const char *str(const TestBase::Result result)
{
    switch (result) {
        case TestBase::Result::Success: return "Success";
        case TestBase::Result::Failed: return "Failed";
        case TestBase::Result::Timeout: return "Timeout";
        case TestBase::Result::NotImplemented: return "Not implemented";
	default: return "Unknown";
    }
}

TestFactory& TestFactory::instance()
{
    static TestFactory instance;
    return instance;
}

void TestFactory::registerTest(ITestRegistrar* registrar, const char *name)
{
	auto *entry = new ListEntry();
	entry->testname = name;
	entry->testbase = registrar->instantiateTest();
	_registry.add(entry);
}

TestBase* TestFactory::getTest(const char *name)
{
	for (const auto &entry: _registry) {
		if (strcmp(entry->testname, name) == 0) {
			return entry->testbase;
		}
	}
	return nullptr;
}

List<TestFactory::ListEntry *> &TestFactory::getAllTests()
{
	return _registry;
}

