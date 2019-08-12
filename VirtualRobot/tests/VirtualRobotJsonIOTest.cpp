/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2018 Rainer Kartmann
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJsonIOTest

#include <VirtualRobot/VirtualRobotTest.h>

#include <filesystem>

#include <VirtualRobot/Util/json/io.h>


namespace fs = std::filesystem;

struct Fixture
{
    const std::string FILENAME = "JsonIOTest.json";
    
    nlohmann::json testj;
    
    
    Fixture()
    {
        testj["s"] = "answer";
        testj["i"] = 42;
        
        if (fs::is_regular_file(FILENAME))
        {
            fs::remove(FILENAME);
        }
    }
    ~Fixture()
    {
        if (fs::is_regular_file(FILENAME))
        {
            fs::remove(FILENAME);
        }
    }
};


BOOST_FIXTURE_TEST_SUITE(VirtualRobotIOConversionTest, Fixture)


BOOST_AUTO_TEST_CASE(test_read_json_existent)
{
    // Write the JSON file.
    {
        std::ofstream ofs(FILENAME);
        ofs << testj.dump();
        // Ensure file exists.
    }
    BOOST_CHECK(fs::exists(FILENAME));
    
    // Test reading.
    const nlohmann::json j = nlohmann::read_json(FILENAME);
    
    BOOST_CHECK_EQUAL(j, testj);
    BOOST_CHECK_EQUAL(j.at("s").get<std::string>(), testj.at("s").get<std::string>());
    BOOST_CHECK_EQUAL(j.at("i").get<int>(), testj.at("i").get<int>());
}


BOOST_AUTO_TEST_CASE(test_read_json_nonexistent)
{
    // Ensure file does not exist.
    BOOST_CHECK(!fs::exists(FILENAME));
    
    // Test reading.
    nlohmann::json j;
    BOOST_CHECK_THROW(j = nlohmann::read_json(FILENAME), std::ios_base::failure);
}


BOOST_AUTO_TEST_CASE(test_write_json_valid)
{
    // Ensure file does not exist.
    BOOST_CHECK(!fs::exists(FILENAME));
    
    // Test writing.
    nlohmann::write_json(FILENAME, testj);
    // Check that something has been written.
    BOOST_CHECK(fs::exists(FILENAME));
}


BOOST_AUTO_TEST_CASE(test_write_json_invalid)
{
    BOOST_CHECK(!fs::exists(FILENAME));
    
    // Make it a directory, i.e. not writable.
    fs::create_directory(FILENAME);
    BOOST_CHECK(fs::exists(FILENAME));
    
    // Test writing.
    BOOST_CHECK_THROW(nlohmann::write_json(FILENAME, testj);, std::ios_base::failure);
    
    // Clean up.
    fs::remove(FILENAME);
    BOOST_CHECK(!fs::exists(FILENAME));
}


BOOST_AUTO_TEST_CASE(test_read_after_write_json)
{
    // Ensure file does not exist.
    BOOST_CHECK(!fs::exists(FILENAME));
    
    // Test writing.
    nlohmann::write_json(FILENAME, testj);
    
    // Test reading.
    nlohmann::json j;
    BOOST_CHECK_NO_THROW(j = nlohmann::read_json(FILENAME));
    
    BOOST_CHECK_EQUAL(j, testj);
    BOOST_CHECK_EQUAL(j.at("s").get<std::string>(), testj.at("s").get<std::string>());
    BOOST_CHECK_EQUAL(j.at("i").get<int>(), testj.at("i").get<int>());
}



BOOST_AUTO_TEST_SUITE_END()
