project('access-computer', 'cpp',
  version: '0.1.0',
  default_options: [
    'cpp_std=c++17',
    'warning_level=2',
    'default_library=static',
  ],
)

sdk_sp = subproject('satellite-plugin-sdk')
satellite_common_dep = sdk_sp.get_variable('satellite_common_dep')

nlohmann_json_sp = subproject('nlohmann_json')
json_dep = nlohmann_json_sp.get_variable('nlohmann_json_dep')

common_inc = include_directories('src')

planner_lib = static_library(
  'access_computer',
  files(
    'src/planner/time_model.cpp',
    'src/planner/validate.cpp',
    'src/planner/contact_windows.cpp',
    'src/planner/run_planner.cpp',
    'src/geometry/window_merger.cpp',
    'src/gmat/gmat_backend.cpp',
  ),
  include_directories: common_inc,
  dependencies: [satellite_common_dep, json_dep],
  cpp_args: ['-DACCESS_COMPUTER_SOURCE_ROOT="@0@"'.format(meson.current_source_dir())],
)

executable(
  'access-computer',
  'src/access-computer/main.cpp',
  include_directories: common_inc,
  link_with: planner_lib,
  dependencies: [satellite_common_dep, json_dep],
  install: true,
)

executable(
  'ac004-harness',
  'src/test/ac004_harness.cpp',
  include_directories: common_inc,
  link_with: planner_lib,
  dependencies: [satellite_common_dep, json_dep],
  install: false,
)

executable(
  'ac007-harness',
  'src/test/ac007_harness.cpp',
  include_directories: common_inc,
  link_with: planner_lib,
  dependencies: [satellite_common_dep, json_dep],
  install: false,
)

install_data(
  'schemas/remote_sensing_access.input.schema.json',
  'schemas/remote_sensing_access.output.schema.json',
  install_dir: get_option('datadir') / 'access-computer/schemas',
)

install_data(
  'templates/optical_access.script.in',
  'templates/downlink_contact.script.in',
  install_dir: get_option('datadir') / 'access-computer/templates',
)

install_data(
  'configs/plugins/access-computer.json',
  install_dir: get_option('datadir') / 'satellite/plugins',
)

install_data(
  'samples/remote_sensing_access.json',
  'samples/attitude_estimation.json',
  'samples/downlink_window.json',
  'samples/short_access.json',
  install_dir: get_option('datadir') / 'access-computer/samples',
)

summary('SDK via subproject', 'satellite-plugin-sdk', section: 'Dependencies')
