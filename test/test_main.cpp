#include <boost/ut.hpp>

int main(int argc, const char *argv[]) {
  boost::ut::detail::cfg::parse(argc, argv);
  return 0;
}
