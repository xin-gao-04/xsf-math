#include <xsf_common/log.hpp>
#include <iostream>

int main() {
    XSF_LOG_INFO("log test started");
    XSF_LOG_DEBUG("debug message: {} + {} = {}", 1, 2, 3);
    XSF_LOG_WARN("warning message");
    XSF_LOG_ERROR("error message");

    xsf::log::set_level(xsf::log::level::trace);
    XSF_LOG_TRACE("trace after level change");

    xsf::log::set_level(xsf::log::level::off);
    XSF_LOG_INFO("this should be suppressed");

    std::cout << "log_test_passed" << std::endl;
    return 0;
}
