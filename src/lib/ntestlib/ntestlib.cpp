#include "ntestlib.h"
#include <cstring>

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
	entry->registrar = registrar;
	_registry.add(entry);
	printf("Adding %s\n", name);
}

TestBase* TestFactory::getTest(const char *name)
{
	printf("in get test\n");
	for (const auto &entry: _registry) {
		printf("Comparing: %s and %s\n", entry->testname, name);
		if (strcmp(entry->testname, name) == 0) {
			return reinterpret_cast<TestBase *>(entry->registrar);
		}
	}

	return nullptr;
}
