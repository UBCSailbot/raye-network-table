// Copyright 2017 UBC Sailbot

#include <iostream>

#include <boost/program_options.hpp>

int main(int argc, char const *argv[]) {
  try {
    boost::program_options::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen");

    boost::program_options::variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);

    if (vm.count("help") || vm.size() == 0) {
      // Show help
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    // TODO(tbd): Do something.
  } catch (const boost::program_options::error &ex) {
    std::cerr << ex.what() << std::endl;
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error:" << std::endl;
    std::cerr << ex.what() << std::endl;
  }
}
