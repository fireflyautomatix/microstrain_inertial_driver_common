// Copyright (c) 2022, FireFly Automatix
// This code is licensed under MIT license (see LICENSE file for details)

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "microstrain_inertial_driver_common/microstrain_defs.h"

const char *get_format(const char *format...)
{
    return format;
}

#undef MICROSTRAIN_DEBUG
#define MICROSTRAIN_DEBUG(NODE, ...) debug(get_format(__VA_ARGS__))

#undef MICROSTRAIN_INFO
#define MICROSTRAIN_INFO(NODE, ...) info(get_format(__VA_ARGS__))

#undef MICROSTRAIN_WARN
#define MICROSTRAIN_WARN(NODE, ...) warn(get_format(__VA_ARGS__))

#undef MICROSTRAIN_ERROR
#define MICROSTRAIN_ERROR(NODE, ...) error(get_format(__VA_ARGS__))

#include "microstrain_inertial_driver_common/microstrain_parser.h"

namespace microstrain
{

    class MockMicrostrainParser : public MicrostrainParser
    {
    public:
        MOCK_METHOD(
            void,
            publishNMEASentence,
            (const std::string &),
            (override));

        MOCK_METHOD(
            void,
            debug,
            (const char *));

        MOCK_METHOD(
            void,
            info,
            (const char *));

        MOCK_METHOD(
            void,
            warn,
            (const char *));

        MOCK_METHOD(
            void,
            error,
            (const char *));
    };

}; // namespace microstrain

using namespace ::testing;

class TestMicrostrainParser : public Test
{
public:
    microstrain::MockMicrostrainParser parser_;
};

TEST_F(TestMicrostrainParser, IgnoreAuxMipMessage)
{
    // This is a real data capture from a GQ7 aux port
    const std::string aux_string = "$GPGGA,223821.50,4044.62613,N,11158.82446,W,4,28,0.5,1295.7,M,18.8,M,,*44\r\nue\223441A 024 313 300\0\0\0\0\b261\0\0\0\0\0\0>\201\006%>\266\310\264\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

    EXPECT_CALL(parser_, publishNMEASentence(HasSubstr("GPGGA"))).Times(1);
    EXPECT_CALL(parser_, warn(HasSubstr("Malformed NMEA sentence"))).Times(0);

    parser_.parseAuxString(aux_string);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);

    bool all_successful = RUN_ALL_TESTS();

    return all_successful;
}