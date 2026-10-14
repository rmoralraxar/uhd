//
// Copyright 2026 Raxar Technologies
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "ad9361_device.h"
#include <uhd/exception.hpp>
#include <boost/test/unit_test.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace uhd { namespace usrp {

class ad9361_fastlock_test_access
{
public:
    static void set_tx_frequency(ad9361_device_t& device, double frequency_hz)
    {
        device._tx_freq = frequency_hz;
    }

    static void set_profile(ad9361_device_t& device,
        std::size_t slot,
        double frequency_hz,
        uint8_t original_word_15,
        uint8_t programmed_word_15)
    {
        auto& profile                      = device._tx_fastlock_profiles[slot];
        profile.valid                      = true;
        profile.frequency_hz               = frequency_hz;
        profile.profile_word_15_original   = original_word_15;
        profile.profile_word_15_programmed = programmed_word_15;
    }

    static void set_active_profile(ad9361_device_t& device, std::size_t slot)
    {
        device._tx_fastlock_active_profile = slot;
        device._tx_fastlock_prepared       = true;
    }

    static bool profile_valid(const ad9361_device_t& device, std::size_t slot)
    {
        return device._tx_fastlock_profiles[slot].valid;
    }

    static double profile_frequency(const ad9361_device_t& device, std::size_t slot)
    {
        return device._tx_fastlock_profiles[slot].frequency_hz;
    }

    static uint8_t original_word_15(const ad9361_device_t& device, std::size_t slot)
    {
        return device._tx_fastlock_profiles[slot].profile_word_15_original;
    }

    static uint8_t programmed_word_15(const ad9361_device_t& device, std::size_t slot)
    {
        return device._tx_fastlock_profiles[slot].profile_word_15_programmed;
    }

    static bool prepared(const ad9361_device_t& device)
    {
        return device._tx_fastlock_prepared;
    }

    static std::size_t active_profile(const ad9361_device_t& device)
    {
        return device._tx_fastlock_active_profile;
    }

    static double tx_frequency(const ad9361_device_t& device)
    {
        return device._tx_freq;
    }

    static double requested_tx_frequency(const ad9361_device_t& device)
    {
        return device._req_tx_freq;
    }

    static uint8_t previous_ensm_config_2(const ad9361_device_t& device)
    {
        return device._tx_fastlock_previous_ensm_config_2;
    }

    static uint8_t previous_tx_pfd_config(const ad9361_device_t& device)
    {
        return device._tx_fastlock_previous_tx_pfd_config;
    }
};

namespace {

constexpr uint32_t REG_RFPLL_DIVIDERS                = 0x005;
constexpr uint32_t REG_ENSM_CONFIG_2                 = 0x015;
constexpr uint32_t REG_TX_PFD_CONFIG                 = 0x270;
constexpr uint32_t REG_TX_INTEGER_BYTE_0             = 0x271;
constexpr uint32_t REG_TX_INTEGER_BYTE_1             = 0x272;
constexpr uint32_t REG_TX_FRACT_BYTE_0               = 0x273;
constexpr uint32_t REG_TX_FRACT_BYTE_1               = 0x274;
constexpr uint32_t REG_TX_FRACT_BYTE_2               = 0x275;
constexpr uint32_t REG_TX_FORCE_ALC                  = 0x276;
constexpr uint32_t REG_TX_FORCE_VCO_TUNE_0           = 0x277;
constexpr uint32_t REG_TX_FORCE_VCO_TUNE_1           = 0x278;
constexpr uint32_t REG_TX_ALC_VARACTOR               = 0x279;
constexpr uint32_t REG_TX_CP_CURRENT                 = 0x27B;
constexpr uint32_t REG_TX_LOOP_FILTER_1              = 0x27E;
constexpr uint32_t REG_TX_LOOP_FILTER_2              = 0x27F;
constexpr uint32_t REG_TX_LOOP_FILTER_3              = 0x280;
constexpr uint32_t REG_TX_VCO_BIAS_1                 = 0x282;
constexpr uint32_t REG_TX_CP_OVERRANGE_VCO_LOCK      = 0x287;
constexpr uint32_t REG_TX_VCO_VARACTOR_CTRL_0        = 0x290;
constexpr uint32_t REG_TX_VCO_VARACTOR_CTRL_1        = 0x291;
constexpr uint32_t REG_TX_FAST_LOCK_SETUP            = 0x29A;
constexpr uint32_t REG_TX_FAST_LOCK_SETUP_INIT_DELAY = 0x29B;
constexpr uint32_t REG_TX_FAST_LOCK_PROGRAM_ADDR     = 0x29C;
constexpr uint32_t REG_TX_FAST_LOCK_PROGRAM_DATA     = 0x29D;
constexpr uint32_t REG_TX_FAST_LOCK_PROGRAM_CTRL     = 0x29F;
constexpr uint8_t FASTLOCK_PROGRAM_START             = 0x03;
constexpr uint8_t FASTLOCK_PROGRAM_DISABLE           = 0x00;
constexpr std::size_t TX_FASTLOCK_ALC_PROFILE_WORD   = 15;
constexpr std::size_t TX_FASTLOCK_NO_ACTIVE_PROFILE  = 8;

struct register_write
{
    uint32_t address;
    uint8_t value;
};

class mock_ad9361_params : public ad9361_params
{
public:
    digital_interface_delays_t get_digital_interface_timing() override
    {
        return {0, 0, 0, 0};
    }

    digital_interface_mode_t get_digital_interface_mode() override
    {
        return AD9361_DDR_FDD_LVDS;
    }

    clocking_mode_t get_clocking_mode() override
    {
        return clocking_mode_t::AD9361_XTAL_P_CLK_PATH;
    }

    double get_band_edge(frequency_band_t) override
    {
        return 0.0;
    }
};

class mock_ad9361_io : public ad9361_io
{
public:
    uint8_t peek8(uint32_t address) override
    {
        const auto it = registers.find(address);
        return it == registers.end() ? 0 : it->second;
    }

    void poke8(uint32_t address, uint8_t value) override
    {
        const auto current_call = poke_call_count++;
        if (current_call == fail_on_poke_call) {
            throw uhd::runtime_error("Injected SPI write failure");
        }

        writes.push_back({address, value});
        registers[address] = value;
    }

    void fail_on_next_poke(std::size_t offset = 0)
    {
        fail_on_poke_call = poke_call_count + offset;
    }

    void clear_writes()
    {
        writes.clear();
    }

    std::map<uint32_t, uint8_t> registers;
    std::vector<register_write> writes;

private:
    std::size_t poke_call_count   = 0;
    std::size_t fail_on_poke_call = std::numeric_limits<std::size_t>::max();
};

struct fastlock_fixture
{
    fastlock_fixture()
        : params(std::make_shared<mock_ad9361_params>())
        , io(std::make_shared<mock_ad9361_io>())
        , device(params, io)
    {
    }

    std::shared_ptr<mock_ad9361_params> params;
    std::shared_ptr<mock_ad9361_io> io;
    ad9361_device_t device;
};

void check_write(const register_write& actual, uint32_t address, uint8_t value)
{
    BOOST_CHECK_EQUAL(actual.address, address);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(actual.value), static_cast<unsigned>(value));
}

void check_profile_word_write(const std::vector<register_write>& writes,
    std::size_t offset,
    std::size_t slot,
    std::size_t word,
    uint8_t value)
{
    const auto profile_address =
        static_cast<uint8_t>((static_cast<uint8_t>(slot) << 4) | word);

    check_write(writes.at(offset), REG_TX_FAST_LOCK_PROGRAM_ADDR, profile_address);
    check_write(writes.at(offset + 1), REG_TX_FAST_LOCK_PROGRAM_DATA, value);
    check_write(
        writes.at(offset + 2), REG_TX_FAST_LOCK_PROGRAM_CTRL, FASTLOCK_PROGRAM_START);
    check_write(
        writes.at(offset + 3), REG_TX_FAST_LOCK_PROGRAM_CTRL, FASTLOCK_PROGRAM_DISABLE);
}

void load_complete_profile_scheme(ad9361_device_t& device)
{
    for (std::size_t slot = 0; slot < 8; ++slot) {
        ad9361_fastlock_test_access::set_profile(
            device, slot, 2.4e9 + static_cast<double>(slot) * 10e6, 0x20, 0x20);
    }
}

} // namespace

BOOST_FIXTURE_TEST_CASE(store_captures_and_programs_complete_profile, fastlock_fixture)
{
    constexpr std::size_t slot = 3;
    constexpr double frequency = 2.45e9;
    const std::array<uint8_t, 16> expected_profile{0x10,
        0x11,
        0x12,
        0x13,
        0x14,
        0x5A,
        0xAB,
        0x2B,
        0x88,
        0x66,
        0x34,
        0x77,
        0x5A,
        0x69,
        0xBC,
        0x55};

    io->registers[REG_TX_CP_OVERRANGE_VCO_LOCK] = 0x02;
    io->registers[REG_TX_INTEGER_BYTE_0]        = 0x10;
    io->registers[REG_TX_INTEGER_BYTE_1]        = 0x11;
    io->registers[REG_TX_FRACT_BYTE_0]          = 0x12;
    io->registers[REG_TX_FRACT_BYTE_1]          = 0x13;
    io->registers[REG_TX_FRACT_BYTE_2]          = 0x14;
    io->registers[REG_TX_VCO_BIAS_1]            = 0x15;
    io->registers[REG_TX_ALC_VARACTOR]          = 0x0A;
    io->registers[REG_TX_CP_CURRENT]            = 0x2B;
    io->registers[REG_TX_LOOP_FILTER_1]         = 0x43;
    io->registers[REG_TX_LOOP_FILTER_2]         = 0x76;
    io->registers[REG_TX_LOOP_FILTER_3]         = 0x08;
    io->registers[REG_TX_VCO_VARACTOR_CTRL_0]   = 0x50;
    io->registers[REG_TX_VCO_VARACTOR_CTRL_1]   = 0x09;
    io->registers[REG_RFPLL_DIVIDERS]           = 0xA0;
    io->registers[REG_TX_FORCE_VCO_TUNE_1]      = 0x31;
    io->registers[REG_TX_FORCE_ALC]             = 0x2A;
    io->registers[REG_TX_FORCE_VCO_TUNE_0]      = 0xBC;
    ad9361_fastlock_test_access::set_tx_frequency(device, frequency);

    device.tx_fastlock_store(slot);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 65);
    check_write(
        io->writes.front(), REG_TX_FAST_LOCK_PROGRAM_CTRL, FASTLOCK_PROGRAM_DISABLE);
    for (std::size_t word = 0; word < expected_profile.size(); ++word) {
        check_profile_word_write(
            io->writes, 1 + word * 4, slot, word, expected_profile[word]);
    }

    BOOST_CHECK(ad9361_fastlock_test_access::profile_valid(device, slot));
    BOOST_CHECK_EQUAL(
        ad9361_fastlock_test_access::profile_frequency(device, slot), frequency);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(
                          ad9361_fastlock_test_access::original_word_15(device, slot)),
        static_cast<unsigned>(expected_profile[TX_FASTLOCK_ALC_PROFILE_WORD]));
    BOOST_CHECK_EQUAL(static_cast<unsigned>(
                          ad9361_fastlock_test_access::programmed_word_15(device, slot)),
        static_cast<unsigned>(expected_profile[TX_FASTLOCK_ALC_PROFILE_WORD]));
}

BOOST_FIXTURE_TEST_CASE(store_failure_invalidates_complete_scheme, fastlock_fixture)
{
    constexpr std::size_t slot                  = 2;
    load_complete_profile_scheme(device);
    io->registers[REG_TX_CP_OVERRANGE_VCO_LOCK] = 0x02;
    io->fail_on_next_poke(9);

    BOOST_CHECK_THROW(device.tx_fastlock_store(slot), uhd::runtime_error);
    for (std::size_t profile = 0; profile < 8; ++profile) {
        BOOST_CHECK(!ad9361_fastlock_test_access::profile_valid(device, profile));
    }
}

BOOST_FIXTURE_TEST_CASE(prepare_rejects_invalid_profile_before_io, fastlock_fixture)
{
    BOOST_CHECK_THROW(device.tx_fastlock_prepare(8), uhd::value_error);

    for (std::size_t slot = 0; slot < 7; ++slot) {
        ad9361_fastlock_test_access::set_profile(
            device, slot, 2.4e9 + static_cast<double>(slot) * 10e6, 0x20, 0x20);
    }
    BOOST_CHECK_THROW(device.tx_fastlock_prepare(0), uhd::runtime_error);
    BOOST_CHECK(io->writes.empty());
}

BOOST_FIXTURE_TEST_CASE(
    prepare_configures_fastlock_without_alc_correction, fastlock_fixture)
{
    constexpr std::size_t slot = 2;
    constexpr double frequency = 2.46e9;
    load_complete_profile_scheme(device);
    ad9361_fastlock_test_access::set_profile(device, slot, frequency, 0x20, 0x20);
    io->registers[REG_TX_FORCE_ALC]  = 0x11;
    io->registers[REG_ENSM_CONFIG_2] = 0xA0;
    io->registers[REG_TX_PFD_CONFIG] = 0x40;

    device.tx_fastlock_prepare(slot);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 6);
    check_write(io->writes[0], REG_TX_FAST_LOCK_SETUP_INIT_DELAY, 0x00);
    check_write(io->writes[1], REG_TX_FAST_LOCK_SETUP, 0x41);
    check_write(io->writes[2], REG_TX_FAST_LOCK_PROGRAM_CTRL, 0x00);
    check_write(io->writes[3], REG_ENSM_CONFIG_2, 0xA1);
    check_write(io->writes[4], REG_TX_PFD_CONFIG, 0x41);
    check_write(io->writes[5], REG_TX_FAST_LOCK_SETUP, 0x41);

    BOOST_CHECK(ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), slot);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::tx_frequency(device), frequency);
    BOOST_CHECK_EQUAL(
        ad9361_fastlock_test_access::requested_tx_frequency(device), frequency);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(
                          ad9361_fastlock_test_access::previous_ensm_config_2(device)),
        0xA0U);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(
                          ad9361_fastlock_test_access::previous_tx_pfd_config(device)),
        0x40U);
}

BOOST_FIXTURE_TEST_CASE(
    prepare_corrects_alc_conflict_before_configuration, fastlock_fixture)
{
    constexpr std::size_t slot = 1;
    load_complete_profile_scheme(device);
    ad9361_fastlock_test_access::set_profile(device, slot, 2.41e9, 0x2A, 0x2A);
    io->registers[REG_TX_FORCE_ALC] = 0x15;

    device.tx_fastlock_prepare(slot);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 10);
    check_profile_word_write(io->writes, 0, slot, TX_FASTLOCK_ALC_PROFILE_WORD, 0x2C);
    check_write(io->writes[4], REG_TX_FAST_LOCK_SETUP_INIT_DELAY, 0x00);
    check_write(io->writes[5], REG_TX_FAST_LOCK_SETUP, 0x21);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(
                          ad9361_fastlock_test_access::programmed_word_15(device, slot)),
        0x2CU);
}

BOOST_FIXTURE_TEST_CASE(prepare_failure_restores_registers_and_state, fastlock_fixture)
{
    constexpr std::size_t slot = 1;
    load_complete_profile_scheme(device);
    ad9361_fastlock_test_access::set_profile(device, slot, 2.41e9, 0x2A, 0x2A);
    io->registers[REG_TX_FORCE_ALC]  = 0x01;
    io->registers[REG_ENSM_CONFIG_2] = 0xA0;
    io->registers[REG_TX_PFD_CONFIG] = 0x40;
    io->fail_on_next_poke(3);

    BOOST_CHECK_THROW(device.tx_fastlock_prepare(slot), uhd::runtime_error);

    BOOST_CHECK(!ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device),
        TX_FASTLOCK_NO_ACTIVE_PROFILE);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(io->registers[REG_TX_FAST_LOCK_SETUP]), 0U);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(io->registers[REG_TX_PFD_CONFIG]), 0x40U);
    BOOST_CHECK_EQUAL(static_cast<unsigned>(io->registers[REG_ENSM_CONFIG_2]), 0xA0U);
    BOOST_CHECK_EQUAL(
        static_cast<unsigned>(io->registers[REG_TX_FAST_LOCK_PROGRAM_CTRL]), 0U);
}

BOOST_FIXTURE_TEST_CASE(
    recall_without_alc_conflict_only_selects_profile, fastlock_fixture)
{
    ad9361_fastlock_test_access::set_profile(device, 0, 2.41e9, 0x20, 0x20);
    ad9361_fastlock_test_access::set_profile(device, 1, 2.42e9, 0x24, 0x24);
    ad9361_fastlock_test_access::set_active_profile(device, 0);

    device.tx_fastlock_recall(1);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 1);
    check_write(io->writes[0], REG_TX_FAST_LOCK_SETUP, 0x21);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), 1U);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::tx_frequency(device), 2.42e9);
    BOOST_CHECK_EQUAL(
        ad9361_fastlock_test_access::requested_tx_frequency(device), 2.42e9);
}

BOOST_FIXTURE_TEST_CASE(
    recall_with_alc_conflict_corrects_then_selects_profile, fastlock_fixture)
{
    ad9361_fastlock_test_access::set_profile(device, 0, 2.41e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_profile(device, 2, 2.43e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_active_profile(device, 0);

    device.tx_fastlock_recall(2);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 5);
    check_profile_word_write(io->writes, 0, 2, TX_FASTLOCK_ALC_PROFILE_WORD, 0x2C);
    check_write(io->writes[4], REG_TX_FAST_LOCK_SETUP, 0x41);
    BOOST_CHECK_EQUAL(
        static_cast<unsigned>(ad9361_fastlock_test_access::programmed_word_15(device, 2)),
        0x2CU);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), 2U);
}

BOOST_FIXTURE_TEST_CASE(
    recall_restores_original_alc_word_after_prior_correction, fastlock_fixture)
{
    ad9361_fastlock_test_access::set_profile(device, 0, 2.41e9, 0x2C, 0x2C);
    ad9361_fastlock_test_access::set_profile(device, 2, 2.43e9, 0x2A, 0x2C);
    ad9361_fastlock_test_access::set_active_profile(device, 0);

    device.tx_fastlock_recall(2);

    BOOST_REQUIRE_EQUAL(io->writes.size(), 5);
    check_profile_word_write(io->writes, 0, 2, TX_FASTLOCK_ALC_PROFILE_WORD, 0x2A);
    BOOST_CHECK_EQUAL(
        static_cast<unsigned>(ad9361_fastlock_test_access::programmed_word_15(device, 2)),
        0x2AU);
}

BOOST_FIXTURE_TEST_CASE(recall_programming_failure_preserves_state, fastlock_fixture)
{
    ad9361_fastlock_test_access::set_profile(device, 0, 2.41e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_profile(device, 2, 2.43e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_active_profile(device, 0);
    ad9361_fastlock_test_access::set_tx_frequency(device, 2.41e9);
    io->fail_on_next_poke();

    BOOST_CHECK_THROW(device.tx_fastlock_recall(2), uhd::runtime_error);

    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), 0U);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::tx_frequency(device), 2.41e9);
    BOOST_CHECK_EQUAL(
        static_cast<unsigned>(ad9361_fastlock_test_access::programmed_word_15(device, 2)),
        0x2AU);
}

BOOST_FIXTURE_TEST_CASE(recall_selection_failure_keeps_active_profile, fastlock_fixture)
{
    ad9361_fastlock_test_access::set_profile(device, 0, 2.41e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_profile(device, 2, 2.43e9, 0x2A, 0x2A);
    ad9361_fastlock_test_access::set_active_profile(device, 0);
    ad9361_fastlock_test_access::set_tx_frequency(device, 2.41e9);
    io->fail_on_next_poke(4);

    BOOST_CHECK_THROW(device.tx_fastlock_recall(2), uhd::runtime_error);

    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), 0U);
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::tx_frequency(device), 2.41e9);
    BOOST_CHECK_EQUAL(
        static_cast<unsigned>(ad9361_fastlock_test_access::programmed_word_15(device, 2)),
        0x2CU);
}

BOOST_FIXTURE_TEST_CASE(exit_without_prepare_does_not_access_hardware, fastlock_fixture)
{
    device.tx_fastlock_exit();

    BOOST_CHECK(io->writes.empty());
    BOOST_CHECK(!ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device),
        TX_FASTLOCK_NO_ACTIVE_PROFILE);
}

BOOST_FIXTURE_TEST_CASE(exit_restores_configuration_and_preserves_profiles, fastlock_fixture)
{
    constexpr std::size_t slot = 2;
    constexpr double frequency = 2.43e9;
    load_complete_profile_scheme(device);
    ad9361_fastlock_test_access::set_profile(device, slot, frequency, 0x2A, 0x2A);
    io->registers[REG_TX_FORCE_ALC]  = 0x01;
    io->registers[REG_ENSM_CONFIG_2] = 0xA0;
    io->registers[REG_TX_PFD_CONFIG] = 0x40;

    device.tx_fastlock_prepare(slot);
    io->clear_writes();

    device.tx_fastlock_exit();

    BOOST_REQUIRE_EQUAL(io->writes.size(), 4);
    check_write(io->writes[0], REG_TX_FAST_LOCK_SETUP, 0x00);
    check_write(io->writes[1], REG_TX_PFD_CONFIG, 0x40);
    check_write(io->writes[2], REG_ENSM_CONFIG_2, 0xA0);
    check_write(io->writes[3], REG_TX_FAST_LOCK_PROGRAM_CTRL, 0x00);
    BOOST_CHECK(!ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device),
        TX_FASTLOCK_NO_ACTIVE_PROFILE);
    BOOST_CHECK(ad9361_fastlock_test_access::profile_valid(device, slot));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::tx_frequency(device), frequency);
    BOOST_CHECK_EQUAL(
        ad9361_fastlock_test_access::requested_tx_frequency(device), frequency);
}

BOOST_FIXTURE_TEST_CASE(exit_allows_preparing_stored_profiles_again, fastlock_fixture)
{
    constexpr std::size_t slot = 1;
    load_complete_profile_scheme(device);
    ad9361_fastlock_test_access::set_profile(device, slot, 2.42e9, 0x2A, 0x2A);
    io->registers[REG_TX_FORCE_ALC] = 0x01;

    device.tx_fastlock_prepare(slot);
    device.tx_fastlock_exit();
    io->clear_writes();

    device.tx_fastlock_prepare(slot);

    BOOST_CHECK(ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), slot);
    BOOST_CHECK(!io->writes.empty());
}

BOOST_FIXTURE_TEST_CASE(exit_failure_preserves_prepared_state, fastlock_fixture)
{
    constexpr std::size_t slot = 1;
    load_complete_profile_scheme(device);
    io->registers[REG_TX_FORCE_ALC] = 0x01;

    device.tx_fastlock_prepare(slot);
    io->clear_writes();
    io->fail_on_next_poke(1);

    BOOST_CHECK_THROW(device.tx_fastlock_exit(), uhd::runtime_error);

    BOOST_CHECK(ad9361_fastlock_test_access::prepared(device));
    BOOST_CHECK_EQUAL(ad9361_fastlock_test_access::active_profile(device), slot);
    for (std::size_t profile = 0; profile < 8; ++profile) {
        BOOST_CHECK(ad9361_fastlock_test_access::profile_valid(device, profile));
    }
}

}} // namespace uhd::usrp
