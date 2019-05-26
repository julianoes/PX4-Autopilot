#pragma once

#include <px4_posix.h>
#include <cstdio>
#include <containers/List.hpp>


/**
 * @class TestBase
 * Base class for all tests
 */
class TestBase
{
public:
    enum class Result { Success = 0, Failed, Timeout, NotImplemented };

    TestBase() = default;
    virtual ~TestBase() = default;

    Result run();

    static const char *str(const Result result);

    template <typename T>
    void expectEq(const T& a, const T& b, const char *a_str, const char *b_str,
                  const char *file, int line)
    {
        if (a == b) {
            return;
        }

        printf("Expect at %s:%s failed\n", extractFilename(file), line);
        printf("%s != %s\n", a_str, b_str);

        _result = Result::Failed;
    }

    template <typename T>
    void assertEq(const T& a, const T& b, const char *a_str, const char *b_str,
                  const char *file, int line)
    {
        if (a == b) {
            return;
        }

        printf("Assert at %s:%s failed\n", extractFilename(file), line);
        printf("%s != %s\n", a_str, b_str);

	_result = Result::Failed;
	abort_test();
    }

#define EXPECT_EQ(a_, b_) expectEq((a_), (b_), #a_, #b_, __FILE__, __LINE__)
#define ASSERT_EQ(a_, b_) assertEq((a_), (b_), #a_, #b_, __FILE__, __LINE__)

#define EXPECT_TRUE(_expr) \
	EXPECT_EQ(_expr, true);
#define ASSERT_TRUE(_expr) \
	ASSERT_EQ(_expr, true);

#define EXPECT_FALSE(_expr) \
	EXPECT_EQ(_expr, false);
#define ASSERT_FALSE(_expr) \
	ASSERT_EQ(_expr, false);

protected:
    virtual void run_testbody() = 0;
    Result _result{Result::Success};

private:
    static void *run_trampoline(void *self);
    const char *extractFilename(const char *path);
    void abort_test();
};
/**
 * Base class for TestRegistrar
 * See TestRegistrar below for explanations
 */
class ITestRegistrar
{
public:
    virtual TestBase *instantiateTest() = 0;
};

/**
 * This is the factory, the common interface to "tests".
 * Tests registers themselves here and the factory can serve them on demand.
 * It is a Singleton.
 */
class TestFactory
{
public:
    static TestFactory& instance();

    TestFactory(TestFactory const&) = delete;
    void operator=(TestFactory const&) = delete;

    struct ListEntry : public ListNode<ListEntry *> {
	const char *testname;
	TestBase *testbase;
    };

    TestBase *getTest(const char *name);
    List<ListEntry *> &getAllTests();


private:
    void registerTest(ITestRegistrar* registrar, const char *name);

    TestFactory() = default;

    List<ListEntry *> _registry; /**< Holds pointers to test registrars */

    template <typename TTest>
    friend class TestRegistrar;
};

/**
 * Helper class that registers a test upon construction.
 */
template <class TTest>
class TestRegistrar : public ITestRegistrar
{
public:
    explicit TestRegistrar(const char *classname);
    TestBase *instantiateTest() override;

private:
    const char *_classname;
};

template <class TTest>
TestRegistrar<TTest>::TestRegistrar(const char *classname) : _classname(classname)
{
    TestFactory& factory = TestFactory::instance();
    factory.registerTest(this, classname);
}

template <class TTest>
TestBase *TestRegistrar<TTest>::instantiateTest()
{
    return new TTest();
}

#define REGISTER_TEST(CONCATNAME) \
	class CONCATNAME : public TestBase \
	{ \
	public: \
	    CONCATNAME() = default; \
	    virtual ~CONCATNAME() override = default; \
	    virtual void run_testbody() override; \
	}; \
	namespace { \
		static const char CONCATNAME##name[] = #CONCATNAME; \
		static TestRegistrar<CONCATNAME> CONCATNAME##_registrar(CONCATNAME##name); \
	} \
	void CONCATNAME::run_testbody()

#define TEST(_name, _case) \
	REGISTER_TEST(_name##_case)

